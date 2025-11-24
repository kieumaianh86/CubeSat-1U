#ifndef LOGIC_SCIENCE_H
#define LOGIC_SCIENCE_H

#include "stm32h7xx_hal.h"
#include "cubesat_data.h"
#include "axis.h"
#include "hmc5883l.h"
#include "mpu6050.h"
#include "neo8m.h"
//#include "OV2640.h"
#include "stdint.h"

/* //science phase timing
#define SCIENCE_PHASE1_COLLECT_MS 40000 // phase 1: collect data
#define SCIENCE_PHASE2_PROCESS_MS 55000 //phase 2: process & save data
#define SCIENCE_PHASE3_CLEAN_MS   60000 //phase 3: clean data
#define SCIENCE_PHASE4_COMPLETE_MS  61000 //phase 4: chuyen state */

/* #define SCIENCE_PHASE1_PREP_MS         10000
#define SCIENCE_LAST_UPDATE_SENSOR_MS  1000
#define SCIENCE_PHASE1_COLLECT_MS      30000
#define SCIENCE_PHASE2_PROCESS_MS      5000 */

//phase 1: collect(40s) = prep(10s) + loop(30s): check bat(every 10s), read sensors
#define SCIENCE_PHASE1_PREP_MS       10000
#define SCIENCE_PHASE1_COLLECT_MS    40000

//phase 2: process and save (55s - 40s = 15s) = 
#define SCIENCE_PHASE2_PROCESS_MS   55000

//phase 3: clean (60s - 55s = 5s)
#define SCIENCE_PHASE3_CLEAN_MS     60000


typedef enum {
    SCIENCE_OK = 0,
    SCIENCE_IN_PROGRESS,
    SCIENCE_ERR_BATTERY_LOW,
    SCIENCE_ERR_BATTERY_CRITICAL,
    SCIENCE_ERR_SENSOR_CRITICAL,
    SCIENCE_ERR_FLASH_FAIL,
    SCIENCE_ERR_TIMEOUT,
    SCIENCE_ERR_EMERGENCY_CMD
} science_error_t;

typedef enum {
    SCIENCE_PHASE_STANDBY = 0,
    SCIENCE_PHASE_PREP,           // 0-10s
    SCIENCE_PHASE_COLLECT,        // 10-40s
    SCIENCE_PHASE_PROCESS,   // 40-55s
    SCIENCE_PHASE_CLEAN,        // 55-60s
    SCIENCE_PHASE_COMPLETE
} science_phase_t;


/* science_error_t Logic_Science_Init(void);
science_error_t Logic_Science_Process(uint32_t ms);
science_error_t Logic_Science_Phase_Collect(uint32_t ms);
science_error_t Logic_Science_Phase_Process(uint32_t ms);
science_error_t Logic_Science_Phase_Clean(uint32_t ms); */

science_error_t Logic_Science_Init(void);
science_error_t Logic_Science_Process(uint32_t dt_ms);
science_error_t Logic_Science_Abort(void);
science_phase_t Logic_Science_GetPhase(void);
uint32_t Logic_Science_GetPhaseTimer(void);

#endif