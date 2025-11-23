#ifndef LOGIC_H
#define LOGIC_H

#include "cubesat_data.h"
#include "stm32h7xx_hal.h"
#include "main.h"

typedef enum {
    LOGIC_OK = 0,
    LOGIC_IN_PROGRESS,
    LOGIC_ERROR,
    LOGIC_BATTERY_CRITICAL,
    LOGIC_BATTERY_SLEEP,
    LOGIC_SENSOR_CRITICAL,
    LOGIC_FLASH_CRITICAL,
    LOGIC_LORA_CRITICAL,
    LOGIC_TEMP_CRITICAL,
    LOGIC_WATCHDOG_TIMEOUT
} logic_error_t;


logic_error_t Logic_Init(void);
logic_error_t Logic_Process(void);
logic_error_t Logic_SetState(cubesat_state_t new_state);

cubesat_state_t Logic_GetState(void);
cubesat_status_t* Logic_GetStatus(void);
system_health_t* Logic_GetHealth(void);

//battery
uint8_t Logic_Battery_Read(void);
bool Logic_Battery_Check_Critical(void);
bool Logic_Battery_Check_Low(void);
bool Logic_Battery_Check_Comm(void);
bool Logic_Battery_Check_Sleep(void);

//sensor
uint8_t Logic_Sensor_Count_Working(void);
bool Logic_Sensor_Check_Critical_Fail(void);
sensor_status_t Logic_Sensor_Get_Status(sensor_id_t id);
void Logic_Sensor_Mark_Dead(sensor_id_t id);

//temperature
float Logic_Temp_Read(void);
bool Logic_Temp_Check_Safe(void);

//flash
uint32_t Logic_Flash_Get_Free_KB(void);
bool Logic_Flash_Check_Min_Free(void);

//lora
bool Logic_LoRa_Is_OK(void);

//command
gcs_command_t Logic_Command_Get_Pending(void);
void Logic_Command_Clear(void);
void Logic_Command_Set(gcs_command_t cmd);

//log
void Logic_Log(const char* format, ...);
void Logic_Log_Error(error_level_t level, uint16_t code, const char* msg);

//watchdog
void Logic_Watchdog_Init(uint32_t timeout_ms);
void Logic_Watchdog_Kick(void);



#endif