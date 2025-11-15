#include "hardware.h"
#include "stdio.h"
#include "stdarg.h"

#define MPU6050_ADDR 0x68
#define HMC5883L_ADDR 0x1E
hw_status_t hardware_init(hardware_t *hw)
{
  if (hw == NULL)
  {
    return HW_ERROR;
  }

  //clear hardware structure
  memset(hw, 0, sizeof(hardware_t));

  hw_status_t status = HW_OK;
  //1.initialize mpu6050
  mpu6050_config_t mpu_config = {
    .hi2c = &hi2c4,
    .device_addr = MPU6050_ADDR,
    .accel_scale = MPU6050_ACCEL_SCALE_2G,
    .gyro_scale = MPU6050_GYRO_SCALE_250DPS,
    .i2c_timeout_ms = 100
  };

  mpu6050_status_t mpu_status = mpu6050_init(&hw->mpu, &mpu_config);
  if (mpu_status == MPU6050_OK)
  {
    hw->mpu_ready = 1;
  }
  else if (mpu_status == MPU6050_NOT_READY)
  {
    hw->mpu_ready = 0;
  }
  else {
    hw->mpu_ready = 0;
    status = HW_IN_PROGRESS;
  }

  //2. initialize hmc5883l
  hmc5883l_config_t mag_config = {
    .hi2c = &hi2c4,
    .device_addr = HMC5883L_ADDR,
    .samples_avg = HMC5883L_SAMPLES_8,
    .data_rate = HMC5883L_RATE_15HZ,
    .gain = HMC5883L_GAIN_330,
    .mode = HMC5883L_MODE_CONTINUOUS,
    .i2c_timeout_ms = 100
  };

  hmc5883l_status_t mag_status = hmc5883l_init(&hw->mag, &mag_config);
  if (mag_status == HMC5883L_OK)
  {
    hw->mag_ready = 1;
  } else if (mag_status == HMC5883L_NOT_READY) 
  {
    hw->mag_ready = 0;
  } else 
  {
    hw->mag_ready = 0;
    status = HW_IN_PROGRESS;
  }

  //3. initialize gps
  neo8m_config_t gps_config = {
    .huart = &huart3
  };
  neo8m_status_t gps_status = neo8m_init(&hw->gps, &gps_config);
  if (gps_status == NEO8M_OK)
  {
    hw->gps_ready = 1;
  } else if (gps_status == NEO8M_ERROR)
  {
    hw->gps_ready = 0;
  } else
  {
    hw->gps_ready = 0;
    status = HW_IN_PROGRESS;
  }
  return status;  
}

//bien dem sensor
int count = 0;
//process hardware initialization
hw_status_t hardware_init_process(hardware_t *hw)
{
  if (hw == NULL)
  {
    return HW_ERROR;
  }
  
  //check if mpu6050 need wakeup processing
  if (!hw->mpu_ready && hw->mpu.init_state == MPU6050_INIT_STATE_WAKING_UP)
  {
    mpu6050_status_t status = mpu6050_init_process(&hw->mpu);
    if (status == MPU6050_OK)
    {
      hw->mpu_ready = 1;
    }
    
  }

  //check overall readiness
  if (hw->mag_ready)
  {
    count++;
  } else 
  {
    Hardware_UART_Printf("Mag ERROR\r\n");
  }
  if (hw->mpu_ready)
  {
    count++;
  } 
  else
  {
    Hardware_UART_Printf("Mpu ERROR\r\n");
  }
  if (hw->gps_ready)
  {
    count++;
  }
  else 
  {
    Hardware_UART_Printf("GPS ERROR\r\n");
  }
  //...
  if (count >= 3)
  {
    Hardware_UART_Printf("Sensors OK: %d\r\n", count);
    return HW_OK;
  }
  else
  {
    Hardware_UART_Printf("Sensors Fail: %d\r\n", count);
    return HW_ERROR;
  }
  return HW_IN_PROGRESS;
}

uint8_t hardware_is_ready(const hardware_t *hw)
{
  if (hw == NULL)
  {
    return 0;
  }
  return (count >= 3) ? 1 : 0;
}

hw_status_t hardware_read_imu(hardware_t *hw)
{
  if (hw == NULL || !hw->mpu_ready)
  {
    return HW_ERROR;
  }
  if (mpu6050_read_all(&hw->mpu) != MPU6050_OK)
  {
    Hardware_UART_Printf("MPU read ERROR\r\n");
    return HW_MPU_ERROR;
  }
  return HW_OK;
  
}

hw_status_t hardware_read_mag(hardware_t *hw)
{
  if (hw == NULL || !hw->mag_ready)
  {
    return HW_ERROR;
  }
  if (hmc5883l_read_mag(&hw->mag) != HMC5883L_OK)
  {
    Hardware_UART_Printf("MAG read ERROR\r\n");
    return HW_MAG_ERROR;
  }
  return HW_OK;
}

hw_status_t hardware_process_gps(hardware_t *hw, uint8_t byte)
{
  if (hw == NULL || !hw->gps_ready)
  {
    return HW_ERROR;
  }
  if (neo8m_process(&hw->gps, byte) != NEO8M_OK)
  {
    Hardware_UART_Printf("GPS read ERROR\r\n");
    return HW_ERROR;
  }
  return HW_OK;  
}

const mpu6050_handle_t* hardware_get_mpu(const hardware_t *hw)
{
  if (hw == NULL)
  {
    return NULL;
  }
  return &hw->mpu;
  
}
const hmc5883l_handle_t* hardware_get_mag(const hardware_t *hw)
{
  if (hw == NULL)
  {
    return NULL;
  }
  return &hw->mag;
  
}
const neo8m_handle_t* hardware_get_gps(const hardware_t *hw)
{
  if (hw == NULL)
  {
    return NULL;
  }
  return &hw->gps;
  
}




void Hardware_UART_Printf(const char *format,...)
{
  char buffer[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), 100);
  
}