#include "logic_sleep.h"
#include "logic.h"
#include "hardware.h"

sleep_error_t Logic_Sleep_Enter(void)
{
    system_health_t* health = Logic_GetHealth();
    
    HW_Power_All_Sensors_Off();
    
    uint32_t wake_interval = Logic_Sleep_GetWakeInterval();
    
    Logic_Log("SLEEP: Entering, wake in %lu ms\r\n", wake_interval);
    
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    
    return SLEEP_OK;
}

sleep_error_t Logic_Sleep_WakeUp(wake_source_t source)
{
    system_health_t* health = Logic_GetHealth();
    
    Logic_Log("SLEEP: Wake-up from %d\r\n", source);
    
    // Đọc pin
    uint8_t battery = Logic_Battery_Read();
    health->battery_percent = battery;
    
    // Pin <20% -> SLEEP lại
    if (battery < BATTERY_CRITICAL)
    {
        Logic_Log("SLEEP: Battery < 20%%, sleep again\r\n");
        return SLEEP_STAY_ASLEEP;
    }
    
    // Check WHO_AM_I sensors
    uint8_t working = Logic_Sensor_Count_Working();
    health->working_sensor_count = working;
    
    // Check flash, RTC
    bool flash_ok = Logic_Flash_Check_Min_Free();
    // rtc_check()
    
    // Ghi log wake-up
    Logic_Log("SLEEP: bat=%d%%, sensors=%d\r\n", battery, working);
    
    // Pin ≥30% + sensors OK -> STANDBY
    if (battery >= BATTERY_LOW && working >= SENSOR_MIN_WORKING)
    {
        Logic_Log("SLEEP: Exit to STANDBY\r\n");
        return SLEEP_EXIT_TO_STANDBY;
    }
    
    // Pin ≥20% + sensors lỗi -> SAFE
    if (battery >= BATTERY_CRITICAL && working < SENSOR_MIN_WORKING)
    {
        Logic_Log("SLEEP: Exit to SAFE (sensor fail)\r\n");
        return SLEEP_EXIT_TO_SAFE;
    }
    
    // Default: stay asleep
    return SLEEP_STAY_ASLEEP;
}

uint32_t Logic_Sleep_GetWakeInterval(void)
{
    system_health_t* health = Logic_GetHealth();
    
    // 15–30% → wake mỗi 10 phút
    if (health->battery_percent >= BATTERY_SLEEP && 
        health->battery_percent < BATTERY_LOW)
    {
        return SLEEP_WAKE_INTERVAL_HIGH;
    }
    
    // <15% → wake mỗi 30 phút
    return SLEEP_WAKE_INTERVAL_LOW;
}
