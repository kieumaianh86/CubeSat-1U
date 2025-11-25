#ifndef LOGIC_SLEEP_H
#define LOGIC_SLEEP_H

#include "cubesat_data.h"
#include "stm32h7xx_hal.h"

#define SLEEP_WAKE_INTERVAL_HIGH        600000
#define SLEEP_WAKE_INTERVAL_LOW         1800000
#define SLEEP_WAKEUP_BATTERY_CHECK      0
#define SLEEP_WAKEUP_SENSOR_CHECK       1000
#define SLEEP_WAKEUP_FLASH_CHECK        3000
#define SLEEP_WAKEUP_RTC_CHECK          5000
#define SLEEP_WAKEUP_DECISION_TIME      6000

typedef enum {
    WAKE_SOURCE_RTC = 0,
    WAKE_SOURCE_PIN_CHARGE,
    WAKE_SOURCE_WATCHDOG,
    WAKE_SOURCE_UNKNOWN
} wake_source_t;

typedef enum {
    SLEEP_OK = 0,
    SLEEP_STAY_ASLEEP,
    SLEEP_EXIT_TO_STANDBY,
    SLEEP_EXIT_TO_SAFE
} sleep_error_t;

sleep_error_t Logic_Sleep_Enter(void);
sleep_error_t Logic_Sleep_WakeUp(wake_source_t source);
uint32_t Logic_Sleep_GetWakeInterval(void);

#endif
