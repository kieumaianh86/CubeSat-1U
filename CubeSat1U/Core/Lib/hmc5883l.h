#ifndef HMC5883L_H
#define HMC5883L_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "axis.h"

/**
 * @brief mã trạng thái/lỗi
 * 
 */
typedef enum { 
  HMC5883L_OK = 0,
  HMC5883L_ERROR,
  HMC5883L_I2C_ERROR,
  HMC5883L_DEVICE_NOT_FOUND,
  HMC5883L_INVALID_ID,
  HMC5883L_TIMEOUT,
  HMC5883L_INVALID_PARAM,
  HMC5883L_NOT_READY
} hmc5883l_status_t;

/**
 * @brief sample rate (Hz)
 * 
 */
typedef enum {
  HMC5883L_RATE_0_75HZ = 0,
  HMC5883L_RATE_1_5hZ,
  HMC5883L_RATE_3HZ,
  HMC5883L_RATE_7_5HZ,
  HMC5883L_RATE_15HZ,
  HMC5883L_RATE_30HZ,
  HMC5883L_RATE_75HZ
} hmc5883l_rate_t;

/**
 * @brief số mẫu trung bình
 * 
 */
typedef enum{
  HMC5883L_SAMPLES_1 = 0,
  HMC5883L_SAMPLES_2,
  HMC5883L_SAMPLES_4,
  HMC5883L_SAMPLES_8
} hmc5883l_samples_t;

/**
 * @brief gain/range
 * 
 */

 typedef enum {
  HMC5883L_GAIN_1370 = 0, //0.88 Ga
  HMC5883L_GAIN_1090, //1.3 Ga
  HMC5883L_GAIN_820, //1.9 Ga
  HMC5883L_GAIN_660, //2.5 Ga
  HMC5883L_GAIN_440, //4.0 Ga
  HMC5883L_GAIN_390, //4.7 Ga
  HMC5883L_GAIN_330, //5.6 Ga
  HMC5883L_GAIN_230 //8.1 Ga
 } hmc5883l_gain_t;

 /**
  * @brief mode
  * 
  */
 typedef enum {
  HMC5883L_MODE_CONTINUOUS = 0,
  HMC5883L_MODE_SINGLE,
  HMC5883L_MODE_IDLE
 } hmc5883l_mode_t;

 /**
  * @brief init state
  * 
  */
 typedef enum {
  HMC5883L_INIT_STATE_UNINITIALIZED = 0,
  HMC5883L_INIT_STATE_CONFIGURING,
  HMC5883L_INIT_STATE_READY
 } hmc5883l_init_state_t;

 /**
  * @brief main handle
  * 
  */
 typedef struct
 {
  //hardware
  I2C_HandleTypeDef *i2c;
  uint8_t device_addr;

  //data
  axis3_t raw;
  axis3_t mag;

  //config
  float gain_scale;
  hmc5883l_mode_t mode;

  //state
  hmc5883l_init_state_t init_state;
  uint32_t last_read_time;
 } hmc5883l_handle_t;
 
/**
 * @brief config khoi tao
 * 
 */
typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t device_addr;
  hmc5883l_rate_t data_rate;
  hmc5883l_samples_t samples_avg;
  hmc5883l_gain_t gain;
  hmc5883l_mode_t mode;
  uint32_t i2c_timeout_ms;
}hmc5883l_config_t;


/**
 * @brief khoi tao hmc5883l
 * 
 */
hmc5883l_status_t hmc5883l_init(hmc5883l_handle_t *hmc, const hmc5883l_config_t *config);

/**
 * @brief tiep tuc khoi tao
 * 
 */
hmc5883l_status_t hmc5883l_init_process(hmc5883l_handle_t *hmc);

/**
 * @brief kiem tra san sang chua
 * 
 */
uint8_t hmc5883l_is_ready(const hmc5883l_handle_t *hmc);

/**
 * @brief doc du lieu cam bien
 * 
 */
hmc5883l_status_t hmc5883l_read_mag(hmc5883l_handle_t *hmc);

/**
 * @brief đọc dữ liệu blocking (single mode)
 * 
 */
hmc5883l_status_t hmc5883l_read_mag_blocking(hmc5883l_handle_t *hmc);

/**
 * @brief kiểm tra dữ liệu có sẵn chưa
 * 
 */
uint8_t hmc5883l_is_data_ready(hmc5883l_handle_t *hmc);

/**
 * @brief đặt chế độ đo
 * 
 * @param hmc 
 * @param mode 
 * @return hmc5883l_status_t 
 */
hmc5883l_status_t hmc5883l_set_mode(hmc5883l_handle_t *hmc, hmc5883l_mode_t mode);

/**
 * @brief calibration (đo min/max)
 * 
 * @param hmc 
 * @param offset 
 * @param scale 
 * @param duration_ms 
 * @return hmc5883l_status_t 
 */
hmc5883l_status_t hmc5883l_calibrate(hmc5883l_handle_t *hmc, axis3_t *offset, axis3_t *scale, uint32_t duration_ms);
/**
 * @brief tính góc heading
 * 
 * @param hmc 
 * @return float 
 */
float hmc5883l_get_heading(const hmc5883l_handle_t *hmc);
/**
 * @brief tính tilt-compensated heading
 * 
 * @param hmc 
 * @param pitch 
 * @param roll 
 * @return float 
 */

float hmc5883l_get_tilt_compensated_heading(const hmc5883l_handle_t *hmc, float pitch, float roll);




#endif