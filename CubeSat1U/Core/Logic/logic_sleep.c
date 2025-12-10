#include "logic_sleep.h"
#include "logic.h"
#include "hardware.h"
#include "sdcard.h"

extern RTC_HandleTypeDef hrtc;
extern void SystemClock_Config(void);

sleep_error_t Logic_Sleep_Enter(void)
{
    HW_Power_All_Sensors_Off();
    uint32_t wake_ms = Logic_Sleep_GetWakeInterval();
    Logic_Log("SLEEP: Entering, wake in %lu ms\r\n", wake_ms);

    RTC_TimeTypeDef current_time;
    RTC_DateTypeDef current_date;
    RTC_AlarmTypeDef alarm = {0};
    
    HAL_RTC_GetTime(&hrtc, &current_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &current_date, RTC_FORMAT_BIN);
    
    uint32_t total_seconds = current_time.Hours * 3600 + 
                            current_time.Minutes * 60 + 
                            current_time.Seconds + (wake_ms / 1000);
    
    alarm.AlarmTime.Hours = (total_seconds / 3600) % 24;
    alarm.AlarmTime.Minutes = (total_seconds / 60) % 60;
    alarm.AlarmTime.Seconds = total_seconds % 60;
    alarm.AlarmTime.SubSeconds = 0;
    alarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
    alarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    alarm.Alarm = RTC_ALARM_A;
    
    HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();
    SystemClock_Config();
    return SLEEP_OK;
}

wake_source_t Logic_Sleep_DetectWakeSource(void)
{
    if (__HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF)) {
        __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
        return WAKE_SOURCE_RTC;
    }
    return WAKE_SOURCE_WATCHDOG;
}

sleep_error_t Logic_Sleep_WakeUp(wake_source_t source)
{
    system_health_t* health = Logic_GetHealth();
    cubesat_status_t* status = Logic_GetStatus();
    Logic_Log("SLEEP: Wake-up from %d\r\n", source);

    // IMPROVED: Handle wake sources with ACTUAL ACTIONS
    switch (source) {
    case WAKE_SOURCE_RTC:
        Logic_Log("SLEEP: RTC alarm (scheduled)\r\n");
        // Normal scheduled wake, no action needed
        break;
        
    case WAKE_SOURCE_PIN_CHARGE:
        Logic_Log("SLEEP: Battery charging!\r\n");
        health->charging = true;
        // Fast exit if charged
        uint8_t quick_bat = Logic_Battery_Read();
        if (quick_bat >= BATTERY_LOW) {
            Logic_Log("SLEEP: Charged %d%%, fast exit\r\n", quick_bat);
            return SLEEP_EXIT_TO_STANDBY;
        }
        break;
        
    case WAKE_SOURCE_WATCHDOG:
        Logic_Log("SLEEP: Watchdog reset!\r\n");
        status->error_code = 0x50;
        // Log to SD for debugging
        char err[64];
        snprintf(err, sizeof(err), "WDT,%lu,%d\r\n", HAL_GetTick(), health->battery_percent);
        SD_WriteScience((uint8_t*)err, strlen(err));
        return SLEEP_EXIT_TO_SAFE;  // Force SAFE after watchdog
    }
    
    uint8_t battery = Logic_Battery_Read();
    health->battery_percent = battery;

    if (battery < BATTERY_CRITICAL) {
        Logic_Log("SLEEP: Battery <20%%, stay asleep\r\n");
        return SLEEP_STAY_ASLEEP;
    }
    
    uint8_t working = Logic_Sensor_Count_Working();
    health->working_sensor_count = working;
    //bool sd_ok = Logic_Flash_Check_Min_Free();
    
    Logic_Log("SLEEP: bat=%d%%, sensors=%d\r\n", battery, working);
    
    if (battery >= BATTERY_LOW && working >= SENSOR_MIN_WORKING) {
        return SLEEP_EXIT_TO_STANDBY;
    }
    
    if (battery >= BATTERY_CRITICAL && working < SENSOR_MIN_WORKING) {
        return SLEEP_EXIT_TO_SAFE;
    }
    
    return SLEEP_STAY_ASLEEP;
}

uint32_t Logic_Sleep_GetWakeInterval(void)
{
    system_health_t* health = Logic_GetHealth();
    if (health->battery_percent >= BATTERY_SLEEP && 
        health->battery_percent < BATTERY_LOW) {
        return SLEEP_WAKE_INTERVAL_HIGH;
    }
    return SLEEP_WAKE_INTERVAL_LOW;
}
