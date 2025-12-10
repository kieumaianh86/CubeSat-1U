#include "logic.h"
#include "INA219.h"
#include "mpu6050.h"
#include "hardware.h"
#include "sdcard.h"
#include <stdarg.h>
#include <stdio.h>

extern ina219_t hina219;  // FIXED: Was INA219_HandleTypeDef
extern mpu6050_handle_t mpu_handle;  // For temperature
extern system_health_t health;
extern cubesat_status_t status;

system_health_t* Logic_GetHealth(void) { return &health; }
cubesat_status_t* Logic_GetStatus(void) { return &status; }

uint8_t Logic_Battery_Read(void)
{
    // FIXED: Access struct fields directly, not via function with pointers
    /* if (INA219_ReadAll(&hina219) == INA219_OK)
    {
        float voltage_v = hina219.voltage;
        
        // 4.2V=100%, 3.0V=0%
        if (voltage_v >= 4.2f) return 100;
        if (voltage_v <= 3.0f) return 0;
        return (uint8_t)((voltage_v - 3.0f) / 1.2f * 100.0f);
    } */
   if (Checkbattery(&hina219) == battery_OK)
   {
    health.battery_percent = ina219_BatteryLife(&hina219);
   }
   
    return health.battery_percent;
}

bool Logic_Battery_Check_Critical(void)
{
    uint8_t battery = Logic_Battery_Read();
    health.battery_percent = battery;
    return (battery < BATTERY_CRITICAL);
}

bool Logic_Battery_Check_Low(void)
{
    uint8_t battery = Logic_Battery_Read();
    health.battery_percent = battery;
    return (battery < BATTERY_LOW);
}

bool Logic_Battery_Check_Comm(void)
{
    uint8_t battery = Logic_Battery_Read();
    health.battery_percent = battery;
    return (battery >= BATTERY_COMM_MIN);
}

float Logic_Temp_Read(void)
{
    // FIXED: Read from MPU6050, not DS18B20
    if (mpu6050_read_all(&mpu_handle) == MPU6050_OK)
    {
        return mpu_handle.temp_scaled;
    }
    return health.temperature;  // Return last known
}

bool Logic_Temp_Check_Safe(void)
{
    float temp = Logic_Temp_Read();
    health.temperature = temp;
    return (temp >= TEMP_MIN_SAFE && temp <= TEMP_MAX_SAFE);
}

uint8_t Logic_Sensor_Count_Working(void)
{
    uint8_t count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (health.sensor_status[i] == SENSOR_STATUS_OK)
            count++;
    health.working_sensor_count = count;
    return count;
}

void Logic_Sensor_Mark_Dead(sensor_id_t sensor)
{
    if (sensor < SENSOR_COUNT)
        health.sensor_status[sensor] = SENSOR_STATUS_DEAD;
}

bool Logic_Sensor_Check_Critical_Fail(void)
{
    uint8_t dead_count = 0;
    for (int i = 0; i < SENSOR_COUNT; i++)
        if (health.sensor_status[i] == SENSOR_STATUS_DEAD)
            dead_count++;
    return (dead_count >= SENSOR_CRITICAL_FAIL);
}

bool Logic_Flash_Check_Min_Free(void)
{
    
    health.flash_free_kb = SD_GetFreeKB();
    return (health.flash_free_kb >= SD_MIN_FREE_KB);
}

uint32_t Logic_Flash_Get_Free_KB(void)
{
    return health.flash_free_kb;
}

gcs_command_t Logic_Command_Get_Pending(void) { return status.pending; }
void Logic_Command_Set(gcs_command_t cmd) { status.pending = cmd; }
void Logic_Command_Clear(void) { status.pending = CMD_NONE; }

void Logic_Log(const char *format, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args); 
    va_end(args);
    
    Hardware_UART_Printf("%s", buffer);
}

uint32_t Logic_GPS_GetUnixTime(void) 
{
    extern neo8m_handle_t gps_handle;
    struct tm t = {
        .tm_year = gps_handle.data.date.year - 1900,
        .tm_mon = gps_handle.data.date.month - 1,
        .tm_mday = gps_handle.data.date.day,
        .tm_hour = gps_handle.data.time.hour,
        .tm_min = gps_handle.data.time.minute,
        .tm_sec = gps_handle.data.time.second
    };
    return mktime(&t);
}
