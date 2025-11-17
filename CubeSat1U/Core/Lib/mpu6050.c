#include "mpu6050.h"

//private definitions
//địa chỉ thanh ghi
#define MPU6050_REG_WHO_AM_I         0x75
#define MPU6050_REG_PWR_MGMT_1       0x6B
#define MPU6050_REG_GYRO_CONFIG      0x1B
#define MPU6050_REG_ACCEL_CONFIG     0x1C
#define MPU6050_REG_ACCEL_XOUT_H     0x3B
#define MPU6050_REG_GYRO_XOUT_H      0x43
#define MPU6050_REG_TEMP_OUT_H       0x41

//hằng số
#define MPU6050_WHO_AM_I_VALUE       0x68
#define MPU6050_DEFAULT_TIMEOUT_MS   100
#define MPU6050_WAKEUP_DELAY_MS      10      // Thời gian đợi sau wake-up

//bit config
#define ACCEL_CONFIG_BIT_POS         3
#define GYRO_CONFIG_BIT_POS          3
#define PWR_MGMT_1_RESET_BIT         7
#define PWR_MGMT_1_SLEEP_BIT         6

//private variable
static uint32_t s_default_timeout = MPU6050_DEFAULT_TIMEOUT_MS;

//private function
static mpu6050_status_t mpu6050_read_register(mpu6050_handle_t *mpu, uint8_t reg, uint8_t *data, uint32_t timeout);
static mpu6050_status_t mpu6050_write_register(mpu6050_handle_t *mpu, uint8_t reg, uint8_t data, uint32_t timeout);
static mpu6050_status_t mpu6050_read_registers(mpu6050_handle_t *mpu, uint8_t reg, uint8_t *data, uint8_t length, uint32_t timeout);
static void mpu6050_update_sensitivity(mpu6050_handle_t *mpu, mpu6050_accel_scale_t accel_scale, mpu6050_gyro_scale_t gyro_scale );

mpu6050_status_t mpu6050_init(mpu6050_handle_t *mpu, const mpu6050_config_t *config)
{
  //
  if (mpu == NULL || config == NULL || config->hi2c == NULL)
  {
    return MPU6050_INVALID_PARAM;
  }

  //khoi tao struct
  mpu->i2c = config->hi2c;
  mpu->device_addr = config->device_addr;
  mpu->init_state = MPU6050_INIT_STATE_UNINITIALIZED;
  mpu->wakeup_timestamp = 0;

  uint32_t timeout = (config->i2c_timeout_ms > 0) ? config->i2c_timeout_ms : s_default_timeout;

  //1. kiem tra ket noi i2c
  if (HAL_I2C_IsDeviceReady(mpu->i2c, mpu->device_addr << 1, 3, timeout) != HAL_OK)
  {
    return MPU6050_DEVICE_NOT_FOUND;
  }

  //2. kiem tra who am i
  uint8_t who_am_i;
  if (mpu6050_read_register(mpu, MPU6050_REG_WHO_AM_I, &who_am_i, timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  
  if (who_am_i != MPU6050_WHO_AM_I_VALUE)
  {
    return MPU6050_INVALID_WHO_AM_I;
  }

  //3. wakeup mpu (clear sleep bit)
  if (mpu6050_write_register(mpu, MPU6050_REG_PWR_MGMT_1, 0x00, timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }

  //save config
  mpu->init_state = MPU6050_INIT_STATE_WAKING_UP;
  mpu->wakeup_timestamp = HAL_GetTick();

  //save scale
  mpu6050_update_sensitivity(mpu, config->accel_scale, config->gyro_scale);
  mpu->pending_config.accel_scale = config->accel_scale;
  mpu->pending_config.gyro_scale = config->gyro_scale;
  mpu->pending_config.timeout = timeout;
  
  return MPU6050_NOT_READY;

}

mpu6050_status_t mpu6050_init_process(mpu6050_handle_t *mpu)
{
  if (mpu == NULL)
  {
    return MPU6050_INVALID_PARAM;
  }

  if (mpu->init_state == MPU6050_INIT_STATE_UNINITIALIZED)
  {
    return MPU6050_ERROR;
  }

  if (mpu->init_state == MPU6050_INIT_STATE_WAKING_UP)
  {
    uint32_t ms = HAL_GetTick() - mpu->wakeup_timestamp;
    if (ms < MPU6050_WAKEUP_DELAY_MS)
    {
      return MPU6050_NOT_READY;
    }

    mpu6050_accel_scale_t accel_scale = mpu->pending_config.accel_scale;
    mpu6050_gyro_scale_t gyro_scale = mpu->pending_config.gyro_scale;
    uint32_t timeout = mpu->pending_config.timeout;

    //config gyro
    if (mpu6050_write_register(mpu, MPU6050_REG_GYRO_CONFIG, gyro_scale << GYRO_CONFIG_BIT_POS, timeout) != MPU6050_OK)
    {
      return MPU6050_I2C_ERROR;
    }
    //config accel
    if (mpu6050_write_register(mpu, MPU6050_REG_ACCEL_CONFIG, (uint8_t) accel_scale << ACCEL_CONFIG_BIT_POS, timeout) != MPU6050_OK)
    {
      return MPU6050_I2C_ERROR;
    }
    
    mpu->init_state = MPU6050_INIT_STATE_READY;
    return MPU6050_OK;

  }
  
  return MPU6050_ERROR;
  
  
}

uint8_t mpu6050_is_ready(const mpu6050_handle_t *mpu)
{
  if (mpu == NULL)
  {
    return 0;
  }
  return (mpu->init_state == MPU6050_INIT_STATE_READY) ? 1 : 0;
  
}

mpu6050_status_t mpu6050_read_all(mpu6050_handle_t *mpu)
{
  if (mpu == NULL || mpu->init_state != MPU6050_INIT_STATE_READY)
  {
    return MPU6050_ERROR;
  }
  uint8_t buffer[14];

  //read 14 byteses: 6 accel + 2 temp + 6 gyro
  if (mpu6050_read_registers(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer, 14, s_default_timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  
  //parse raw data
  mpu->accel_raw.x = (int16_t) (buffer[0] << 8) | buffer[1];
  mpu->accel_raw.y = (int16_t) (buffer[2] << 8) | buffer[3];
  mpu->accel_raw.z = (int16_t) (buffer[4] << 8) | buffer[5];
  mpu->temp_raw = (int16_t) (buffer[6] << 8) | buffer[7];
  mpu->gyro_raw.x = (int16_t) (buffer[8] << 8) | buffer[9];
  mpu->gyro_raw.y = (int16_t) (buffer[10] << 8) | buffer[11];
  mpu->gyro_raw.z = (int16_t) (buffer[12] << 8) | buffer[13];

  //convert to physical unit
  mpu->accel_scaled.x = (float) mpu->accel_raw.x/mpu->accel_sensitivity;
  mpu->accel_scaled.y = (float) mpu->accel_raw.y/mpu->accel_sensitivity;
  mpu->accel_scaled.z = (float) mpu->accel_raw.z/mpu->accel_sensitivity;
   
  mpu->gyro_scaled.x = (float) mpu->gyro_raw.x/mpu->gyro_sensitivity;
  mpu->gyro_scaled.y = (float) mpu->gyro_raw.y/mpu->gyro_sensitivity;
  mpu->gyro_scaled.z = (float) mpu->gyro_raw.z/mpu->gyro_sensitivity;

  mpu->temp_scaled = (float) (mpu->temp_raw / 340) + 36.53f;
  return MPU6050_OK;
  
}

mpu6050_status_t mpu6050_read_accel (mpu6050_handle_t *mpu)
{
  if (mpu == NULL || mpu->init_state != MPU6050_INIT_STATE_READY)
  {
    return MPU6050_ERROR;
  }
  uint8_t buffer[6];
  if (mpu6050_read_registers(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer, 6, s_default_timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  mpu->accel_raw.x = (int16_t) (buffer[0] << 8) | buffer[1];
  mpu->accel_raw.y = (int16_t) (buffer[2] << 8) | buffer[3];
  mpu->accel_raw.z = (int16_t) (buffer[4] << 8) | buffer[5];
  //convert to physical unit
  mpu->accel_scaled.x = (float) mpu->accel_raw.x/mpu->accel_sensitivity;
  mpu->accel_scaled.y = (float) mpu->accel_raw.y/mpu->accel_sensitivity;
  mpu->accel_scaled.z = (float) mpu->accel_raw.z/mpu->accel_sensitivity;
  
  return MPU6050_OK;
  
  
}

mpu6050_status_t mpu6050_read_gyro(mpu6050_handle_t *mpu)
{
  if (mpu == NULL || mpu->init_state != MPU6050_INIT_STATE_READY)
  {
    return MPU6050_ERROR;
  }
  uint8_t buffer[6];
  if (mpu6050_read_registers(mpu, MPU6050_REG_GYRO_XOUT_H, buffer, 6, s_default_timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  mpu->gyro_raw.x = (int16_t) (buffer[0] << 8) | buffer[1];
  mpu->gyro_raw.y = (int16_t) (buffer[2] << 8) | buffer[3];
  mpu->gyro_raw.z = (int16_t) (buffer[4] << 8) | buffer[5];
  //convert to physical unit
  mpu->gyro_scaled.x = (float) mpu->gyro_raw.x/mpu->gyro_sensitivity;
  mpu->gyro_scaled.y = (float) mpu->gyro_raw.y/mpu->gyro_sensitivity;
  mpu->gyro_scaled.z = (float) mpu->gyro_raw.z/mpu->gyro_sensitivity;
  
  return MPU6050_OK;
}

mpu6050_status_t mpu6050_read_temp(mpu6050_handle_t *mpu)
{
  if (mpu == NULL || mpu->init_state != MPU6050_INIT_STATE_READY)
  {
    return MPU6050_ERROR;
  }
  uint8_t buffer[2];
  if (mpu6050_read_registers(mpu, MPU6050_REG_TEMP_OUT_H, buffer, 6, s_default_timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  mpu->temp_raw = (int16_t) (buffer[0] << 8) | buffer[1];
  mpu->temp_scaled = (float) (mpu->temp_raw / 340) + 36.53f;
  return MPU6050_OK;
}

mpu6050_status_t mpu6050_reset(mpu6050_handle_t *mpu)
{
  if (mpu == NULL)
  {
    return MPU6050_INVALID_PARAM;
  }

  //set reset bit
  if (mpu6050_write_register(mpu, MPU6050_REG_PWR_MGMT_1, (1 << PWR_MGMT_1_RESET_BIT), s_default_timeout) != MPU6050_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  mpu->init_state = MPU6050_INIT_STATE_UNINITIALIZED;
  return MPU6050_OK;
  
  
}

static mpu6050_status_t mpu6050_read_register(mpu6050_handle_t *mpu, uint8_t reg, uint8_t *data, uint32_t timeout)
{
  if (HAL_I2C_Mem_Read(mpu->i2c, mpu->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 1, timeout) != HAL_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  return MPU6050_OK;
}

static mpu6050_status_t mpu6050_write_register(mpu6050_handle_t *mpu, uint8_t reg, uint8_t data, uint32_t timeout)
{
  if (HAL_I2C_Mem_Write(mpu->i2c, mpu->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout ) != HAL_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  return MPU6050_OK;
}

/**
 * @brief hàm đọc nhiều thanh ghi cùng 1 lúc -> phục vụ read all
 * 
 * @param mpu 
 * @param reg: bắt đầu từ thanh ghi reg và tăng dần theo length
 * @param data 
 * @param length : tổng thanh ghi muốn đọc
 * @param timeout 
 * @return mpu6050_status_t 
 */
static mpu6050_status_t mpu6050_read_registers(mpu6050_handle_t *mpu, uint8_t reg, uint8_t *data, uint8_t length, uint32_t timeout)
{
  if (HAL_I2C_Mem_Read(mpu->i2c, mpu->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, timeout) != HAL_OK)
  {
    return MPU6050_I2C_ERROR;
  }
  return MPU6050_OK;
}

static void mpu6050_update_sensitivity(mpu6050_handle_t *mpu, mpu6050_accel_scale_t accel_scale, mpu6050_gyro_scale_t gyro_scale)
{
  switch (accel_scale)
  {
  case MPU6050_ACCEL_SCALE_2G:
    mpu->accel_sensitivity = 16384.0f;
    break;
  case MPU6050_ACCEL_SCALE_4G:
    mpu->accel_sensitivity = 8192.0f;
    break;
  case MPU6050_ACCEL_SCALE_8G:
    mpu->accel_sensitivity = 4096.0f;
    break;
  case MPU6050_ACCEL_SCALE_16G:
    mpu->accel_sensitivity = 2048.0f;
  default:
    mpu->accel_sensitivity = 16384.0f;
    break;
  }

  switch (gyro_scale)
  {
  case MPU6050_GYRO_SCALE_250DPS:
    mpu->gyro_sensitivity = 131.0f;
    break;
  case MPU6050_GYRO_SCALE_500DPS:
    mpu->gyro_sensitivity = 65.5f;
    break;
  case MPU6050_GYRO_SCALE_1000DPS:
    mpu->gyro_sensitivity = 32.8f;
    break;
  case MPU6050_GYRO_SCALE_2000DPS:
    mpu->gyro_sensitivity = 16.4f;
    break;
  default:
    mpu->gyro_sensitivity = 131.0f;
    break;
  }
}

