#include "logic_science.h"
#include "logic.h"
#include "hardware.h"
#include "sdcard.h"
#include "packet_protocol.h"
#include <string.h>

static science_phase_t current_phase = SCIENCE_PHASE_STANDBY;
static uint32_t science_start_tick = 0;
static uint8_t sd_retry_count = 0;
static bool cleanup_done = false;

static uint8_t data_buffer[4096];
static uint32_t data_length = 0;

static science_error_t Logic_Science_Phase_Prep(void);
static science_error_t Logic_Science_Phase_Collect(void);
static science_error_t Logic_Science_Phase_Process(void);
static science_error_t Logic_Science_Phase_Clean(void);

science_error_t Logic_Science_Init(void) 
{
  current_phase = SCIENCE_PHASE_PREP;
  science_start_tick = HAL_GetTick();
  sd_retry_count = 0;
  cleanup_done = false;
  data_length = 0;
  Logic_Log("SCIENCE: Init\r\n");
  return SCIENCE_OK;
}

science_error_t Logic_Science_Process(void)
{
  uint32_t total_timer = HAL_GetTick() - science_start_tick;
  
  // Timeout check - NO fixed time limits per phase, only total
  if (total_timer > TIMEOUT_SCIENCE_TOTAL)
  {
    Logic_Log("SCIENCE: Total timeout\r\n");
    return SCIENCE_ERR_TIMEOUT;
  }
  
  // Battery critical check
  cubesat_status_t* status = Logic_GetStatus();
  if (HAL_GetTick() - status->last_battery_check >= BATTERY_CHECK_INTERVAL)
  {
    status->last_battery_check = HAL_GetTick();
    if (Logic_Battery_Check_Critical())
    {
      Logic_Log("SCIENCE: Battery critical\r\n");
      return SCIENCE_ERR_BATTERY_CRITICAL;
    }
  }
  
  // Emergency command check
  if (Logic_Command_Get_Pending() == CMD_START_COMM)
  {
    Logic_Log("SCIENCE: COMM command\r\n");
    return SCIENCE_ERR_EMERGENCY_CMD;
  }
  
  switch (current_phase)
  {
  case SCIENCE_PHASE_PREP:
    if (Logic_Science_Phase_Prep() == SCIENCE_OK) {
      current_phase = SCIENCE_PHASE_COLLECT;
      Logic_Log("SCIENCE: PREP done\r\n");
    }
    break;
        
  case SCIENCE_PHASE_COLLECT:
    if (Logic_Science_Phase_Collect() == SCIENCE_OK) {
      current_phase = SCIENCE_PHASE_PROCESS;
      Logic_Log("SCIENCE: COLLECT done\r\n");
    }
    break;
        
  case SCIENCE_PHASE_PROCESS:
    if (Logic_Science_Phase_Process() == SCIENCE_OK) {
      current_phase = SCIENCE_PHASE_CLEAN;
      Logic_Log("SCIENCE: PROCESS done\r\n");
    }
    break;
        
  case SCIENCE_PHASE_CLEAN:
    if (Logic_Science_Phase_Clean() == SCIENCE_OK) {
      current_phase = SCIENCE_PHASE_COMPLETE;
    }
    break;
        
  case SCIENCE_PHASE_COMPLETE:
    current_phase = SCIENCE_PHASE_STANDBY;
    return SCIENCE_OK;
        
  default:
    break;
  }
    
  return SCIENCE_IN_PROGRESS;
}

static science_error_t Logic_Science_Phase_Prep(void)
{
  static bool sensors_powered = false;
  static uint32_t power_on_tick = 0;
  
  if (!sensors_powered)
  {
    if (Logic_Battery_Check_Low())
    {
      Logic_Log("SCIENCE PREP: Battery low\r\n");
      return SCIENCE_ERR_BATTERY_LOW;
    }

    HW_Power_GPS_On();
    HW_Power_IMU_On();        // MPU6050 on I2C4
    HW_Power_Magnetometer_On(); // HMC5883L on I2C4
    HW_Power_Camera_On();      // OV2640 on I2C2

    sensors_powered = true;
    power_on_tick = HAL_GetTick();
    Logic_Log("SCIENCE Prep: Sensors powered on\r\n");    
  }
    
  // Wait 2s for sensors to stabilize
  if (HAL_GetTick() - power_on_tick > 2000) {
    sensors_powered = false;
    power_on_tick = 0;
    return SCIENCE_OK;  
  }
  
  return SCIENCE_IN_PROGRESS;
}

static science_error_t Logic_Science_Phase_Collect(void)
{
  static uint32_t last_battery_read = 0;
  system_health_t* health = Logic_GetHealth();
  int count_sensor_ok = 0;

  if (HAL_GetTick() - last_battery_read >= BATTERY_CHECK_INTERVAL)
  {
    last_battery_read = HAL_GetTick();
    if (health->battery_percent < BATTERY_CRITICAL)
    {
      Logic_Log("Battery LOW: %.1f%%\r\n", health->battery_percent);
      return SCIENCE_ERR_BATTERY_LOW;
    }
  }
  
  extern neo8m_handle_t gps_handle;
  extern mpu6050_handle_t mpu_handle;
  extern hmc5883l_handle_t mag_handle;
  
  // Read MPU6050 (I2C4) - includes temperature
  if (mpu6050_read_all(&mpu_handle) != MPU6050_OK)
  {
    health->sensor_fail_count[SENSOR_MPU6050]++;
  } else
  {
    health->sensor_fail_count[SENSOR_MPU6050] = 0;
    count_sensor_ok++;
  }
  
  // Read HMC5883L (I2C4)
  if (hmc5883l_read_mag(&mag_handle) != HMC5883L_OK)
  {
    health->sensor_fail_count[SENSOR_HMC5883L]++;
  } else
  {
    health->sensor_fail_count[SENSOR_HMC5883L] = 0;
    count_sensor_ok++;
  }
  
  // Read GPS
  if (neo8m_get_data(&gps_handle))
  {
    health->sensor_fail_count[SENSOR_GPS]++;
  } else
  {
    health->sensor_fail_count[SENSOR_GPS] = 0;
    count_sensor_ok++;
  }
  
  // Read Battery (INA219 on I2C1)
  extern ina219_t ina219;
  if (Checkbattery(&ina219) == battery_OK)
  {
    health->battery_percent = ina219_BatteryLife(&ina219);
    count_sensor_ok++;
  }
  
  /* if (INA219_ReadAll(&ina219) == INA219_OK) {
    health->battery_percent = (ina219.voltage - 3.0) / (4.2 - 3.0) * 100.0;
    count_sensor_ok++;
  } */
  
  // Camera capture (I2C2) - handled in next phase
  
  // Mark dead sensors
  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    if (health->sensor_fail_count[i] > RETRY_SENSOR)
    {
      Logic_Sensor_Mark_Dead((sensor_id_t)i);
    }
  }

  if (Logic_Sensor_Check_Critical_Fail())
  {
    Logic_Log("SCIENCE COLLECT: Sensor critical fail\r\n");
    return SCIENCE_ERR_SENSOR_CRITICAL;
  } 

  if (count_sensor_ok >= 3)
  {
    return SCIENCE_OK;
  }

  return SCIENCE_IN_PROGRESS;
}

static science_error_t Logic_Science_Phase_Process(void) 
{
    static uint8_t save_step = 0;
    extern neo8m_handle_t gps_handle;
    extern mpu6050_handle_t mpu_handle;
    extern hmc5883l_handle_t mag_handle;
    extern ina219_t ina219;
    
    switch (save_step) {
    case 0:  // Package sensor data
        {
            uint32_t timestamp = HAL_GetTick();
            uint8_t* ptr = data_buffer;
            
            // Header
            memcpy(ptr, "CUBESAT", 7);
            ptr += 7;
            
            // Timestamp
            memcpy(ptr, &timestamp, sizeof(timestamp));
            ptr += sizeof(timestamp);
            
            // MPU6050 data (Accel + Gyro + Temp)
            memcpy(ptr, &mpu_handle.accel_scaled.x, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.accel_scaled.y, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.accel_scaled.z, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.gyro_scaled.x, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.gyro_scaled.y, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.gyro_scaled.z, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mpu_handle.temp_scaled, sizeof(float)); ptr += sizeof(float); // Temperature from IMU
            
            // HMC5883L data
            memcpy(ptr, &mag_handle.mag.x, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mag_handle.mag.y, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &mag_handle.mag.z, sizeof(float)); ptr += sizeof(float);
            
            // GPS data
            memcpy(ptr, &gps_handle.data.latitude, sizeof(double)); ptr += sizeof(double);
            memcpy(ptr, &gps_handle.data.longitude, sizeof(double)); ptr += sizeof(double);
            memcpy(ptr, &gps_handle.data.altitude, sizeof(float)); ptr += sizeof(float);
            
            // Battery data (INA219)
            memcpy(ptr, &ina219.BusVoltage, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &ina219.Current, sizeof(float)); ptr += sizeof(float);
            memcpy(ptr, &ina219.Power, sizeof(float)); ptr += sizeof(float);
            
            data_length = ptr - data_buffer;
            
            // CRC16
            uint16_t crc = Packet_Calculate_CRC(data_buffer, data_length);
            memcpy(ptr, &crc, sizeof(crc));
            data_length += sizeof(crc);
            
            Logic_Log("SCIENCE: Data packaged (%lu bytes)\r\n", data_length);
            save_step = 1;
        }
        break;
        
    case 1:  // Check SD space
        if (SD_GetFreeKB() < SD_MIN_FREE_KB) {
            Logic_Log("SCIENCE: SD low space, deleting oldest\r\n");
            SD_DeleteOldest();
        }
        save_step = 2;
        break;
        
    case 2:  // Write to SD card (not flash) with retry
        if (SD_WriteScience(data_buffer, data_length) == SD_OK) {
            // Verify CRC after write
            uint8_t verify_buf[4096];
            uint32_t verify_len = sizeof(verify_buf);
            
            if (SD_ReadScience(verify_buf, &verify_len) == SD_OK) {
                uint16_t stored_crc, calc_crc;
                memcpy(&stored_crc, &verify_buf[verify_len - 2], sizeof(stored_crc));
                calc_crc = Packet_Calculate_CRC(verify_buf, verify_len - 2);
                
                if (stored_crc == calc_crc) {
                    Logic_Log("SCIENCE: SD write verified\r\n");
                    save_step = 0;
                    sd_retry_count = 0;
                    return SCIENCE_OK;  
                } else {
                    Logic_Log("SCIENCE: CRC mismatch!\r\n");
                }
            }
        }
        
        // Retry logic with increasing delay
        sd_retry_count++;
        HAL_Delay(100 * sd_retry_count);
        
        if (sd_retry_count >= RETRY_FLASH) {
            Logic_Log("SCIENCE: SD failed %d times\r\n", sd_retry_count);
            save_step = 0;
            sd_retry_count = 0;
            return SCIENCE_ERR_FLASH_FAIL;
        }
        break;
    }
    
    return SCIENCE_IN_PROGRESS;
}

static science_error_t Logic_Science_Phase_Clean(void)
{
  if (!cleanup_done)
  {
    HW_Power_GPS_Off();
    HW_Power_IMU_Off();
    HW_Power_Magnetometer_Off();
    HW_Power_Camera_Off();

    cleanup_done = true;
    data_length = 0;
    Logic_Log("SCIENCE CLEANUP: Sensors off\r\n");
    return SCIENCE_OK;
  }
  return SCIENCE_IN_PROGRESS;  
}

science_error_t Logic_Science_Abort(void)
{
    HW_Power_GPS_Off();
    HW_Power_IMU_Off();
    HW_Power_Magnetometer_Off();
    HW_Power_Camera_Off();
    
    current_phase = SCIENCE_PHASE_STANDBY;
    cleanup_done = false;
    data_length = 0;
    Logic_Log("SCIENCE: Aborted\r\n");
    return SCIENCE_OK;
}

science_phase_t Logic_Science_GetPhase(void)
{
    return current_phase;
}

uint32_t Logic_Science_GetPhaseTimer(void)
{
    return HAL_GetTick() - science_start_tick;
}
