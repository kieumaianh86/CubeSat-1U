#ifndef MPU_6050_H
#define MPU_6050_H

#include "stm32h7xx_hal.h"
#include "stdint.h"
#include "axis.h"



typedef enum {
  MPU6050_OK = 0,
  MPU6050_ERROR,
  MPU6050_I2C_ERROR,
  MPU6050_DEVICE_NOT_FOUND,
  MPU6050_INVALID_WHO_AM_I,
  MPU6050_TIMEOUT,
  MPU6050_INVALID_PARAM,
  MPU6050_NOT_READY //chua san sang sau khi wakeup
} mpu6050_status_t;

//config accel
typedef enum {
  MPU6050_ACCEL_SCALE_2G = 0,
  MPU6050_ACCEL_SCALE_4G,
  MPU6050_ACCEL_SCALE_8G,
  MPU6050_ACCEL_SCALE_16G
}mpu6050_accel_scale_t;

//config gyro
typedef enum {
  MPU6050_GYRO_SCALE_250DPS = 0,
  MPU6050_GYRO_SCALE_500DPS,
  MPU6050_GYRO_SCALE_1000DPS,
  MPU6050_GYRO_SCALE_2000DPS  
}mpu6050_gyro_scale_t;

typedef enum {
  MPU6050_INIT_STATE_UNINITIALIZED = 0,
  MPU6050_INIT_STATE_WAKING_UP,
  MPU6050_INIT_STATE_READY
} mpu6050_initstate_t;

typedef struct 
{
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
}mpu6050_data_t;

typedef struct 
{
  I2C_HandleTypeDef *i2c;
  uint8_t device_addr;

  //data
  axis3_t accel_raw;
  axis3_t gyro_raw;
  int16_t temp_raw;

  faxis3_t accel_scaled;
  faxis3_t gyro_scaled;
  float temp_scaled;

  //config and state
  float  accel_sensitivity;
  float gyro_sensitivity;

  mpu6050_initstate_t init_state;
  uint32_t wakeup_timestamp;
  uint8_t is_initialized;
  union 
  {
    struct 
    {
      mpu6050_accel_scale_t accel_scale;
      mpu6050_gyro_scale_t gyro_scale;
      uint32_t timeout;
    }pending_config;
    
  } ;
  
}mpu6050_handle_t;

typedef struct 
{
  I2C_HandleTypeDef *hi2c;
  uint8_t device_addr;
  mpu6050_accel_scale_t accel_scale;
  mpu6050_gyro_scale_t gyro_scale;
  uint32_t i2c_timeout_ms;
}mpu6050_config_t;


/**
 * @brief Khởi tạo mpu6050 với cấu hình mặc định
 * @param mpu: con trỏ tới mpu6050 handle
 * @param config: cấu hình khởi tạo
 * @return mpu6050_status_t: mã trạng thái
 * @note hàm này bắt đầu quá trình wakeup không chặn. Cần gọi mpu_init_process() 
 * sau đó đến khi trả về mpu_ok
 */
mpu6050_status_t mpu6050_init(mpu6050_handle_t *mpu ,const mpu6050_config_t *config);

/**
 * @brief tiếp tục quá trình khởi tạo non-blocking * 
 * @param mpu: con trỏ tới mpu_handle_t
 * @return mpu6050_status_t mpu_ok nếu hoàn tất khởi tạo, mpu_not_ready nếu cần gọi lại
 * hoặc mã lỗi nếu có vấn đề
 */
mpu6050_status_t mpu6050_init_process(mpu6050_handle_t *mpu);

/**
 * @brief kiểm tra xem mpu6050 đã sẵn sàng chưa
 * @param mpu: con trỏ trỏ tới mpu_handle_t 
 * @return uint8_t: 1 nếu sẵn sàng, 0 nếu chưa
 */
uint8_t mpu6050_is_ready(const mpu6050_handle_t *mpu);

/**
 * @brief đọc toàn bộ dữ liệu của mpu gồm accel, gyro, temp
 * @param mpu: con trỏ trỏ tới mpu_handle_t
 * @return mpu6050_status_t 
 */
mpu6050_status_t mpu6050_read_all (mpu6050_handle_t *mpu);

/**
 * @brief đọc dữ liệu gia tốc kế
 * 
 * @param mpu: con trỏ trỏ tới mpu_handle_t
 * @return mpu6050_status_t 
 */
mpu6050_status_t mpu6050_read_accel(mpu6050_handle_t *mpu);

/**
 * @brief đọc dữ liệu con quay hồi chuyển
 * 
 * @param mpu 
 * @return mpu6050_status_t 
 */
mpu6050_status_t mpu6050_read_gyro(mpu6050_handle_t *mpu);

/**
 * @brief đọc dữ liệu nhiệt độ
 * 
 * @param mpu 
 * @return mpu6050_status_t 
 */
mpu6050_status_t mpu6050_read_temp(mpu6050_handle_t *mpu);

/**
 * @brief reset thiết bị mpu6050
 * 
 * @param mpu 
 * @return mpu6050_status_t 
 * @note sau khi reset, cần gọi lại mpu_init và mpu_init_process
 */
mpu6050_status_t mpu6050_reset(mpu6050_handle_t *mpu);


#endif