#ifndef LOGIC_H
#define LOGIC_H

#include "cubesat_data.h"
#include <stdbool.h>
#include <stdint.h>


system_health_t* Logic_GetHealth(void);
cubesat_status_t* Logic_GetStatus(void);


uint8_t Logic_Battery_Read(void);
bool Logic_Battery_Check_Critical(void);  // <20%
bool Logic_Battery_Check_Low(void);       // <30%
bool Logic_Battery_Check_Comm(void);      // >=25%


float Logic_Temp_Read(void);
bool Logic_Temp_Check_Safe(void);  // -10 to 70°C


uint8_t Logic_Sensor_Count_Working(void);
void Logic_Sensor_Mark_Dead(sensor_id_t sensor);
bool Logic_Sensor_Check_Critical_Fail(void);  // >=4 failed


bool Logic_Flash_Check_Min_Free(void);  // >=5KB
uint32_t Logic_Flash_Get_Free_KB(void);

gcs_command_t Logic_Command_Get_Pending(void);
void Logic_Command_Set(gcs_command_t cmd);
void Logic_Command_Clear(void);

void Logic_Log(const char *format, ...);

uint32_t Logic_GPS_GetUnixTime(void);
#endif 
