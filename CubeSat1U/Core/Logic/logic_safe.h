#ifndef LOGIC_SAFE_H
#define LOGIC_SAFE_H

#include "cubesat_data.h"
#define SAFE_TOTAL_TIMEOUT    60000
#define SAFE_CHECK_INTERVAL   10000
#define SAFE_BATTERY_CHECK_TIME   0
#define SAFE_TEMP_CHECK_TIME    1000
#define SAFE_SENSOR_CHECK_TIME  2000
#define SAFE_FLASH_CHECK_TIME   7000
#define SAFE_LORA_CHECK_TIME    7000

typedef enum {
  SAFE_OK = 0,
  SAFE_IN_PROGRESS,
  SAFE_EXIT_TO_STANDBY,
  SAFE_EXIT_TO_SLEEP,
} safe_error_t;

safe_error_t Logic_Safe_Init(void);
safe_error_t Logic_Safe_Process(uint32_t dt_ms);
safe_error_t Logic_Safe_Reset(void);
uint32_t Logic_Safe_GetTimer(void);


#endif