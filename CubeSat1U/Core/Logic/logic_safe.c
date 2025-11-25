#include "logic_safe.h"
#include "cubesat_data.h"
#include "logic.h"
#include "hardware.h"

static uint32_t safe_start_tick = 0;

safe_error_t Logic_Safe_Init(void)
{
    safe_start_tick = HAL_GetTick();
    
    cubesat_status_t* status = Logic_GetStatus();
    status->sensors_recovered = false;
    status->flash_recovered = false;
    status->lora_recovered = false;
    status->temp_recovered = false;
    
    HW_Power_All_Sensors_Off();
    
    Logic_Log("SAFE: Entered - error code %d\r\n", status->error_code);
    
    return SAFE_OK;
}

safe_error_t Logic_Safe_Process(uint32_t dt_ms)
{
    uint32_t total_timer = HAL_GetTick() - safe_start_tick;
    
    cubesat_status_t* status = Logic_GetStatus();
    system_health_t* health = Logic_GetHealth();
    
    // Chu kỳ 10s kiểm tra
    if (total_timer >= SAFE_CHECK_INTERVAL)
    {        
        // Đọc pin
        uint8_t battery = Logic_Battery_Read();
        health->battery_percent = battery;
        
        // Pin <15% -> SLEEP
        if (battery < BATTERY_SLEEP)
        {
            Logic_Log("SAFE: Battery < 15%%, go to SLEEP\r\n");
            return SAFE_EXIT_TO_SLEEP;
        }
        
        // Kiểm tra nhiệt độ
        float temp = Logic_Temp_Read();
        health->temperature = temp;
        if (Logic_Temp_Check_Safe())
        {
            status->temp_recovered = true;
        }
        
        // Check WHO_AM_I sensors
        uint8_t working = Logic_Sensor_Count_Working();
        health->working_sensor_count = working;
        if (working >= SENSOR_MIN_WORKING)
        {
            status->sensors_recovered = true;
        }
        
        // Test flash
        if (Logic_Flash_Check_Min_Free())
        {
            status->flash_recovered = true;
        }
        
        // Test LoRa
        // if lora_test_ok()
        {
            status->lora_recovered = true;
            health->lora_ok = true;
        }
        
        Logic_Log("SAFE: Check - bat=%d%%, temp=%.1f, sensors=%d\r\n", 
                  battery, temp, working);
    }
    
    // Exit conditions
    // Pin ≥30%, sensors ≥3, flash OK, nhiệt độ OK -> STANDBY
    if (health->battery_percent >= BATTERY_LOW &&
        status->sensors_recovered &&
        status->flash_recovered &&
        status->temp_recovered)
    {
        Logic_Log("SAFE: All recovered, exit to STANDBY\r\n");
        return SAFE_EXIT_TO_STANDBY;
    }
    
    // Timeout 60s + pin ≥25% -> STANDBY
    if (total_timer >= SAFE_TOTAL_TIMEOUT && 
        health->battery_percent >= BATTERY_COMM_MIN)
    {
        Logic_Log("SAFE: Timeout 60s + battery >= 25%%, exit to STANDBY\r\n");
        return SAFE_EXIT_TO_STANDBY;
    }
    
    return SAFE_IN_PROGRESS;
}

safe_error_t Logic_Safe_Reset(void)
{
    safe_start_tick = 0;
    
    cubesat_status_t* status = Logic_GetStatus();
    status->sensors_recovered = false;
    status->flash_recovered = false;
    status->lora_recovered = false;
    status->temp_recovered = false;
    
    Logic_Log("SAFE: Reset\r\n");
    return SAFE_OK;
}

uint32_t Logic_Safe_GetTimer(void)
{
    return safe_start_tick - HAL_GetTick();
}
