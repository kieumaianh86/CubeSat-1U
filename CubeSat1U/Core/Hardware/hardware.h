#ifndef HARDWARE_H
#define HARDWARE_H

#include "mpu6050.h"
#include "hmc5883l.h"
#include "neo8m.h"

typedef enum {
  HW_OK = 0,
  HW_ERROR,
  HW_MPU_ERROR,
  HW_MAG_ERROR,
  HW_GPS_ERROR,
  HW_IN_PROGRESS
} hw_status_t;

//hardware handle structure
typedef struct
{
  mpu6050_handle_t mpu;
  hmc5883l_handle_t mag;
  neo8m_handle_t gps;

  uint8_t mpu_ready;
  uint8_t mag_ready;
  uint8_t gps_ready;
} hardware_t;

//external peripheral handles
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3, huart2;
//extern DCMI_HandleTypeDef hdcmi;

//initialization functions
hw_status_t hardware_init(hardware_t *hw);
hw_status_t hardware_init_process(hardware_t *hw);
uint8_t hardware_is_ready(const hardware_t *hw);

//sensor read functions
hw_status_t hardware_read_imu(hardware_t *hw);
hw_status_t hardware_read_mag(hardware_t *hw);
hw_status_t hardware_process_gps(hardware_t *hw, uint8_t byte);

//get data pointers
const mpu6050_handle_t* hardware_get_mpu(const hardware_t *hw);
const hmc5883l_handle_t* hardware_get_mag(const hardware_t *hw);
const neo8m_handle_t* hardware_get_gps(const hardware_t *hw);

void Hardware_UART_Printf(const char *format, ...);

//power control
void HW_Power_GPS_On(void);
void HW_Power_GPS_Off(void);

void HW_Power_Camera_On(void);
void HW_Power_Camera_Off(void);

void HW_Power_LoRa_On(void);
void HW_Power_LoRa_Off(void);

void HW_Power_IMU_On(void);
void HW_Power_IMU_Off(void);

void HW_Power_Magnetometer_On(void);
void HW_Power_Magnetometer_Off(void);

void HW_Power_Temperature_On(void);
void HW_Power_Temperature_Off(void);

void HW_Power_All_Sensors_Off(void);
#endif