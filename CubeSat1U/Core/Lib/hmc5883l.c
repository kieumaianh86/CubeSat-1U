#include "hmc5883l.h"
#include "math.h"

//register
#define HMC5883L_REG_CONFIG_A     0x00
#define HMC5883L_REG_CONFIG_B     0x01
#define HMC5883L_REG_MODE         0x02
#define HMC5883L_REG_DATA_X_MSB   0x03
#define HMC5883L_REG_DATA_Y_MSB   0x05
#define HMC5883L_REG_DATA_Z_MSB   0x07
#define HMC5883L_REG_STATUS       0x09
#define HMC5883L_REG_ID_A         0x0A
#define HMC5883L_REG_ID_B         0x0B
#define HMC5883L_REG_ID_C         0x0C

//ID value
#define HMC5883L_ID_A_VALUE       0x48 //H
#define HMC5883L_ID_B_VALUE       0x34 //4
#define HMC5883L_ID_C_VALUE       0x33 //3

//default value
#define HMC5883L_DEFAULT_ADDR     0x1E
#define HMC5883L_DEFAULT_TIMEOUT  100

//status register bits
#define HMC5883L_STATUS_RDY       0x01
#define HMC5883L_STATUS_LOCK      0x02

//mode bits
#define HMC5883L_MODE_MS          0x80 //high speed i2c

//bit position config
#define SAMPLES_CONFIG_BIT_POS  5
#define DATA_OUT_RATE_BIT_POS   2
#define GAIN_BIT_POS            5

//Sample
#define SAMPLES_TIME 10

//pi
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

//bảng gain scale (LSB/Gauss)
static const float gain_table[] ={
  1370.0f, //0.88Ga
  1090.0f, //1.3Ga
  820.0f,  //1.9Ga
  660.0f, //2.5Ga
  440.0f, //4.0Ga
  390.0f, //4.7Ga
  330.0f, //5.6Ga
  230.0f //8.1Ga
};

static uint32_t s_default_timeout = HMC5883L_DEFAULT_TIMEOUT;
#define i2c_trial 3

//private function
static hmc5883l_status_t hmc5883l_read_register(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t *data, uint32_t timeout);
static hmc5883l_status_t hmc5883l_write_register(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t data, uint8_t timeout);
static hmc5883l_status_t hmc5883l_read_registers(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t *data, uint8_t length, uint32_t timeout);


hmc5883l_status_t hmc5883l_init(hmc5883l_handle_t *hmc, const hmc5883l_config_t *config)
{
  if (hmc == NULL || config == NULL || config->hi2c == NULL)
  {
    return HMC5883L_INVALID_PARAM;
  }

  //init handle
  hmc->i2c = config->hi2c;
  hmc->device_addr = config->device_addr;
  hmc->init_state = HMC5883L_INIT_STATE_UNINITIALIZED;
  hmc->last_read_time = 0;
  hmc->mode = config->mode;

  uint32_t timeout = (config->i2c_timeout_ms > 0) ? config->i2c_timeout_ms : s_default_timeout;

  //kiểm tra kết nối
  if (HAL_I2C_IsDeviceReady(hmc->i2c, hmc->device_addr << 1, i2c_trial, timeout) != HAL_OK)
  {
    return HMC5883L_DEVICE_NOT_FOUND;
  }

  //check id
  uint8_t id[3];
  if (hmc5883l_read_register(hmc, HMC5883L_REG_ID_A, &id[0], timeout) != HMC5883L_OK ||
      hmc5883l_read_register(hmc, HMC5883L_REG_ID_B, &id[1], timeout) != HMC5883L_OK ||
      hmc5883l_read_register(hmc, HMC5883L_REG_ID_C, &id[2], timeout) != HMC5883L_OK )
  {
      return HMC5883L_I2C_ERROR;
  }

  if (id[0] != HMC5883L_ID_A_VALUE ||
      id[1] != HMC5883L_ID_B_VALUE ||
      id[2] != HMC5883L_ID_C_VALUE)
  {
      return HMC5883L_INVALID_ID;
  }
  
  //cấu hình configuration reg A
  //7:6: MA (smaple average)
  //4:2: DO (data rate)
  //1:0: MS (measurement)
  uint8_t config_a = ((uint8_t) config->samples_avg << SAMPLES_CONFIG_BIT_POS ) | 
                      ((uint8_t) config->data_rate << DATA_OUT_RATE_BIT_POS);
  uint8_t config_b = (uint8_t) config->gain << GAIN_BIT_POS;

  if (hmc5883l_write_register(hmc, HMC5883L_REG_CONFIG_A, config_a, timeout) != HMC5883L_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  if (hmc5883l_write_register(hmc, HMC5883L_REG_CONFIG_B, config_b, timeout) != HMC5883L_OK)
  {
    return HMC5883L_I2C_ERROR;
  }

  //save gain scale
  hmc->gain_scale = gain_table[config->gain];

  //cấu hình mode
  if (hmc5883l_write_register(hmc, HMC5883L_REG_MODE, (uint8_t)config->mode, timeout) != HMC5883L_OK)
  {
    return HMC5883L_I2C_ERROR;
  }

  hmc->init_state = HMC5883L_INIT_STATE_READY;
  return HMC5883L_OK;
}



/* hmc5883l_status_t hmc5883l_init_process(hmc5883l_handle_t *hmc)
{
  if (hmc == NULL)
  {
    return HMC5883L_INVALID_PARAM;
  }
  if (hmc->init_state != HMC5883L_INTI_STATE_READY)
  {
    return HMC5883L_ERROR;
  }
  return HMC5883L_OK;
  
} */

uint8_t hmc5883l_is_ready(const hmc5883l_handle_t *hmc)
{
  if (hmc == NULL)
  {
    return 0;
  }
  return (hmc->init_state == HMC5883L_INIT_STATE_READY) ? 1: 0;
}

hmc5883l_status_t hmc5883l_read_mag(hmc5883l_handle_t *hmc)
{
  if (hmc == NULL || hmc->init_state != HMC5883L_INIT_STATE_READY)
  {
    return HMC5883L_ERROR;
  }
  uint8_t buffer[6];
  //đọc 6 byte x-z-y
  if (hmc5883l_read_registers(hmc, HMC5883L_REG_DATA_X_MSB, buffer, 6, s_default_timeout) != HMC5883L_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  
  //parse raw data
  hmc->raw.x = (int16_t) (buffer[0] << 8) | (buffer[1]);
  hmc->raw.z = (int16_t) (buffer[2] << 8) | (buffer[3]);
  hmc->raw.y = (int16_t) (buffer[4] << 8) | (buffer[5]);

  //convert to gauss
  hmc->mag.x = (float) hmc->raw.x / hmc->gain_scale;
  hmc->mag.z = (float) hmc->raw.z / hmc->gain_scale;
  hmc->mag.y = (float) hmc->raw.y / hmc->gain_scale;

  hmc->last_read_time = HAL_GetTick();
  return HMC5883L_OK;
}

hmc5883l_status_t hmc5883l_read_mag_blocking(hmc5883l_handle_t *hmc)
{
  if (hmc == NULL || hmc->init_state != HMC5883L_INIT_STATE_READY)
  {
    return HMC5883L_ERROR;
  }

  //nếu đang ở single mode, trigger measurement
  if (hmc->mode == HMC5883L_MODE_SINGLE)
  {
    if (hmc5883l_write_register(hmc, HMC5883L_REG_MODE, HMC5883L_MODE_SINGLE, s_default_timeout) != HMC5883L_OK)
    {
      return HMC5883L_I2C_ERROR;
    }
    //đợi data ready
    uint32_t start = HAL_GetTick();
    while (!hmc5883l_is_data_ready(hmc))
    {
      if (HAL_GetTick() - start > 100)
      {
        return HMC5883L_TIMEOUT;
      } 
    }
  }
  return hmc5883l_read_mag(hmc);
  
}

uint8_t hmc5883l_is_data_ready(hmc5883l_handle_t *hmc) 
{
  if (hmc == NULL)
  {
    return 0;
  }

  uint8_t status;
  if (hmc5883l_read_register(hmc, HMC5883L_REG_STATUS, &status, s_default_timeout) != HMC5883L_OK)
  {
    return 0;
  }

  return (status & HMC5883L_STATUS_RDY) ? 1 : 0;

}

hmc5883l_status_t hmc5883l_set_mode(hmc5883l_handle_t *hmc, hmc5883l_mode_t mode) 
{
  if (hmc == NULL)
  {
    return HMC5883L_INVALID_PARAM;
  }
  if (hmc5883l_write_register(hmc, HMC5883L_REG_MODE, (uint8_t) mode, s_default_timeout) != HMC5883L_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  hmc->mode = mode;
  return HMC5883L_OK;
  
}

hmc5883l_status_t hmc5883l_calibrate(hmc5883l_handle_t *hmc, axis3_t *offset, axis3_t *scale, uint32_t ms)
{
  if (hmc == NULL || offset == NULL || scale == NULL)
  {
    return HMC5883L_INVALID_PARAM;
  }
  
  axis3_t min = {999999, 999999, 999999};
  axis3_t max = {-999999, -999999, -999999};

  uint32_t start = HAL_GetTick();
  uint32_t samples = 0;
  uint32_t last_read = 0;
  //read min/max in duration(ms)
  while (HAL_GetTick() - start < ms)
  {
    uint32_t now = HAL_GetTick();
    if (now - last_read >= 10)
    {
      if (hmc5883l_read_mag(hmc) == HMC5883L_OK)
      {
        if (hmc->mag.x < min.x) min.x = hmc->mag.x;
        if (hmc->mag.y < min.y) min.y = hmc->mag.y;
        if (hmc->mag.z < min.z) min.z = hmc->mag.z;
        
        if (hmc->mag.x > min.x) min.x = hmc->mag.x;
        if (hmc->mag.y > min.y) min.y = hmc->mag.y;
        if (hmc->mag.z > min.z) min.z = hmc->mag.z;

        samples++;
      }
    }
    last_read = now;   
    
  }
  if (samples < SAMPLES_TIME)
  {
    return HMC5883L_ERROR;
  }

  //calculate offset (hard iron)
  offset->x = (max.x + min.x) / 2.0f;
  offset->y = (max.y + min.y) / 2.0f;
  offset->z = (max.z + min.z) / 2.0f;
  
  //calculate scale (soft iron)
  float avg_delta = ((max.x - min.x) + (max.y - min.y) + (max.z - min.z)) / 3.0f;
  scale->x = avg_delta / (max.x - min.x);
  scale->y = avg_delta / (max.y - min.y);
  scale->z = avg_delta / (max.z - min.z);

  return HMC5883L_OK;
  
}

float hmc5883l_get_heading(const hmc5883l_handle_t *hmc)
{
  if (hmc == NULL)
  {
    return 0.0f;
  }

  //calculate heading
  float heading = atan2f(hmc->mag.y, hmc->mag.x);
  // Declination angle (ví dụ: +15.87° cho Hà Nội)
  // Cần điều chỉnh theo vị trí địa lý
  // float declination = 15.87f * M_PI / 180.0f;
  // heading += declination;

  //chuyen ve 0-360 (0 - 2pi)
  if (heading < 0)
  {
    heading += 2 * M_PI;
  }

  return heading * 180.0f / M_PI;  
}

float hmc5883l_get_tilt_compensated_heading(const hmc5883l_handle_t *hmc, float pitch, float roll)
{
  if (hmc == NULL)
  {
    return 0.0f;
  }

  //chuyen pitch/roll snag radian
  float pitch_rad = pitch * M_PI / 180.0f;
  float roll_rad = roll * M_PI / 180.0f;

  //tilt compensation (bu nghieng khi cam bien khong nam ngang)
  float cos_pitch = cosf(pitch_rad);
  float sin_pitch = sinf(pitch_rad);
  float cos_roll = cosf(roll_rad);
  float sin_roll = sinf(roll_rad);

  float mag_x_comp = hmc->mag.x * cos_pitch + hmc->mag.z * sin_pitch;
  float mag_y_comp = hmc->mag.x * sin_roll *sin_pitch + hmc->mag.y * cos_roll - hmc->mag.z * sin_roll * cos_pitch;
  float heading = atan2f(mag_y_comp, mag_x_comp);

  if (heading < 0)
  {
    heading += 2 * M_PI;
  }
  return heading * 180.0f / M_PI;
  
}

static hmc5883l_status_t hmc5883l_read_register(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t *data, uint32_t timeout)
{ 
  if (HAL_I2C_Mem_Read(hmc->i2c, hmc->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 1, timeout) != HAL_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  return HMC5883L_OK;
}

static hmc5883l_status_t hmc5883l_write_register(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t data, uint8_t timeout)
{ 
  if (HAL_I2C_Mem_Write(hmc->i2c, hmc->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout) != HAL_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  return HMC5883L_OK;
}

static hmc5883l_status_t hmc5883l_read_registers(hmc5883l_handle_t *hmc, uint8_t reg, uint8_t *data, uint8_t length, uint32_t timeout)
{ 
  if (HAL_I2C_Mem_Read(hmc->i2c, hmc->device_addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, timeout) != HAL_OK)
  {
    return HMC5883L_I2C_ERROR;
  }
  return HMC5883L_OK;
}

