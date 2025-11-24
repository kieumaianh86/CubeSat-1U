#include "logic_science.h"
#include "logic.h"
#include "hardware.h"

//extern system_status_t system_status;
static science_phase_t current_phase = SCIENCE_PHASE_STANDBY;
//static uint32_t phase_start_tick = 0;
static uint32_t science_start_tick = 0;
static uint8_t flash_retry_count = 0;
static bool cleanup_done = false; //phase cleanup

// Private function declarations
static science_error_t Logic_Science_Phase_Prep(uint32_t ms);
static science_error_t Logic_Science_Phase_Collect(uint32_t ms);
static science_error_t Logic_Science_Phase_Process(uint32_t ms);
static science_error_t Logic_Science_Phase_Clean(uint32_t ms);

science_error_t Logic_Science_Init(void) 
{
  current_phase = SCIENCE_PHASE_PREP;
  //phase_start_tick = HAL_GetTick();
  science_start_tick = HAL_GetTick();
  flash_retry_count = 0;
  Logic_Log("SCIENCE: Init\r\n");
  return SCIENCE_OK;
}

science_error_t Logic_Science_Process(uint32_t ms)
{
  science_error_t result = SCIENCE_OK;
  //uint32_t phase_timer = HAL_GetTick() - phase_start_tick;
  uint32_t total_timer = HAL_GetTick() - science_start_tick;
  //timeout check
  if (total_timer > TIMEOUT_SCIENCE_TOTAL)
  {
    Logic_Log("SCIENCE: Total timeout\r\n");
    return SCIENCE_ERR_TIMEOUT;
  }
  //battery critical check every 10s
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
  //emergency command check
  if (Logic_Command_Get_Pending() == CMD_START_COMM)
  {
    Logic_Log("SCIENCE: COMM command\r\n");
    return SCIENCE_ERR_EMERGENCY_CMD;
  }
  
  switch (current_phase)
  {
  case SCIENCE_PHASE_PREP:
    result = Logic_Science_Phase_Prep(total_timer);
    if (result == SCIENCE_OK && total_timer >= SCIENCE_PHASE1_PREP_MS)
    {
      current_phase = SCIENCE_PHASE_COLLECT;
      //phase_start_tick = HAL_GetTick();
      Logic_Log("SCIENCE: PREP complete\r\n");
      result = SCIENCE_IN_PROGRESS;
    }
    break;
    
  case SCIENCE_PHASE_COLLECT:
    result = Logic_Science_Phase_Collect(total_timer);
    if (result == SCIENCE_OK && total_timer >= SCIENCE_PHASE2_PROCESS_MS)
    {
      current_phase = SCIENCE_PHASE_PROCESS;
      //phase_start_tick = HAL_GetTick();
      Logic_Log("SCIENCE: COLLECT complete\r\n");
      result = SCIENCE_IN_PROGRESS;
    }
    break;
  case SCIENCE_PHASE_PROCESS:
    result = Logic_Science_Phase_Process(total_timer);
    if (result == SCIENCE_OK && total_timer >= SCIENCE_PHASE3_CLEAN_MS)
    {
      current_phase = SCIENCE_PHASE_CLEAN;
      //phase_start_tick = HAL_GetTick();
      Logic_Log("SCIENCE: PROCESS complete!\r\n");
      result = SCIENCE_IN_PROGRESS;
    }
    break;
  case SCIENCE_PHASE_CLEAN:
    result = Logic_Science_Phase_Clean(total_timer);
    if (result == SCIENCE_OK)
    {
      current_phase = SCIENCE_PHASE_COMPLETE;
      Logic_Log("SCIENCE: CLEAN complete!\r\n");
    }
    break;
  case SCIENCE_PHASE_COMPLETE:
    current_phase = SCIENCE_PHASE_STANDBY;
    Logic_Log("SCIENCE: Complete\r\n");
    return SCIENCE_OK;
  default:
    break;
  }
  return result;
}

static science_error_t Logic_Science_Phase_Prep(uint32_t ms)
{
  static bool sensors_powered = false;
  if (!sensors_powered && ms < SCIENCE_PHASE1_PREP_MS)
  {
    if (Logic_Battery_Check_Low())
    {
      Logic_Log("SCIENCE PREP: Battery low\r\n");
      return SCIENCE_ERR_BATTERY_LOW;
    }

    HW_Power_GPS_On();
    HW_Power_IMU_On();
    HW_Power_Magnetometer_On();
    HW_Power_Temperature_On();
    HW_Power_Camera_On();

    sensors_powered = true;
    Logic_Log("SCIENCE Prep: Sensors powered on\r\n");    
  }

  if (ms >= SCIENCE_PHASE1_PREP_MS)
  {
    sensors_powered = false;
    return SCIENCE_OK;
  }
  return SCIENCE_IN_PROGRESS;
}

static science_error_t Logic_Science_Phase_Collect(uint32_t ms)
{
  static uint32_t last_battery_read = 0;
  system_health_t* health = Logic_GetHealth();

  if (HAL_GetTick() - last_battery_read >= BATTERY_CHECK_INTERVAL)
  {
    last_battery_read = HAL_GetTick();
    if (health->battery_percent < BATTERY_CRITICAL)
    {
      Logic_Log("Battery LOW: %.1f \r\n", health->battery_percent);
      return SCIENCE_ERR_BATTERY_LOW;
    }
  }
  extern neo8m_handle_t gps_handle;
  extern mpu6050_handle_t mpu_handle;
  extern hmc5883l_handle_t mag_handle;
  //extern OV2640_Handle cam_handle;
  static int count_sensor_ok = 0;
  if (mpu6050_read_all(&mpu_handle) != MPU6050_OK)
  {
    health->sensor_fail_count[SENSOR_MPU6050]++;
  } else
  {
    health->sensor_fail_count[SENSOR_MPU6050] = 0;
    count_sensor_ok++;
  }
  if (hmc5883l_read_mag(&mag_handle) != HMC5883L_OK)
  {
    health->sensor_fail_count[SENSOR_HMC5883L]++;
  } else
  {
    health->sensor_fail_count[SENSOR_HMC5883L] = 0;
    count_sensor_ok++;
  }
  if (neo8m_get_data(&gps_handle))
  {
    health->sensor_fail_count[SENSOR_GPS]++;
  } else
  {
    health->sensor_fail_count[SENSOR_GPS] = 0;
    count_sensor_ok++;
  }
  //camera function
  
  
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

static science_error_t Logic_Science_Phase_Process(uint32_t ms)
{
/*   static uint8_t save_step = 0;

  switch (save_step)
  {
  case 0:
    
    break;
  
  default:
    break;
  } */
}


static science_error_t Logic_Science_Phase_Clean(uint32_t ms)
{
  if (!cleanup_done)
  {
    HW_Power_GPS_Off();
    HW_Power_IMU_Off();
    HW_Power_Magnetometer_Off();
    HW_Power_Temperature_Off();
    HW_Power_Camera_Off();

    cleanup_done = true;
    Logic_Log("SCIENCE CLEANUP: Sensors off, RAM free\r\n");
    return SCIENCE_OK;
  }
  return SCIENCE_IN_PROGRESS;  
}

science_error_t Logic_Science_Abort(void)
{
    HW_Power_GPS_Off();
    HW_Power_IMU_Off();
    HW_Power_Magnetometer_Off();
    HW_Power_Temperature_Off();
    HW_Power_Camera_Off();
    
    current_phase = SCIENCE_PHASE_STANDBY;
    Logic_Log("SCIENCE: Aborted\r\n");
    return SCIENCE_OK;
}

science_phase_t Logic_Science_GetPhase(void)
{
    return current_phase;
}

uint32_t Logic_Science_GetPhaseTimer(void)
{
    return science_start_tick - HAL_GetTick();
}