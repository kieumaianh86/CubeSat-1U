#ifndef LOGIC_COMM_H
#define LOGIC_COMM_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32h7xx_hal.h"
#include "cubesat_data.h"

typedef enum {
  COMM_PHASE_STANDBY = 0,
  COMM_PHASE_PREP,        //0-5s
  COMM_PHASE_BEACON,      //5-10s
  COMM_PHASE_LISTEN,      //10-25s
  COMM_PHASE_DATA_TX,     //25-37s
  COMM_PHASE_END,         //37-40s
  COMM_PHASE_COMPLETE
} comm_phase_t;

#define COMM_PHASE1_PREP_MS             5000
#define COMM_PHASE2_SEND_BEACON_MS      10000
#define COMM_PHASE3_LISTEN_MS           25000
#define COMM_PHASE4_DATA_TX_MS            37000
#define COMM_PHASE5_END_MS                 40000

//timing constant
#define LORA_POWER_DELAY_MS       100
#define LORA_INIT_START_MS        500
#define LORA_INIT_TIMEOUT_MS      3000
#define LORA_RETRY_DELAY_MS       500
#define LORA_MAX_ATTEMPTS         2
#define CSMA_CHECK_TIME_MS        4000

#define BEACON_TX_TIMEOUT_MS        2000
#define BEACON_RETRY_DELAY_MS       500
#define BEACON_MAX_RETRIES          3

#define LISTEN_POLL_INTERVAL_MS     50
#define LISTEN_RX_TIMEOUT_MS        100

#define DATA_PACKET_TIMEOUT_MS      1000
#define DATA_ACK_TIMEOUT_MS         500
#define DATA_MAX_FAILS              5
#define DATA_PAYLOAD_MAX            180

#define END_GOODBYE_DELAY_MS        100
#define END_POWEROFF_DELAY_MS       1000

#define BATTERY_MIN_COMM            25
#define BATTERY_MIN_SAFE            20
#define LORA_FAIL_MAX               3

typedef enum {
  COMM_OK = 0,
  COMM_IN_PROGRESS,
  COMM_ERR_BATTERY_LOW,
  COMM_ERR_BATTERY_CRITICAL,
  COMM_ERR_LORA_FAIL,
  COMM_ERR_TIMEOUT
} comm_error_t;

//Subsystem states
typedef struct 
{
  bool powered, initialized;
  uint8_t attempts;
  uint32_t last_tick;
} lora_t;

typedef struct 
{
  uint8_t retry;
  bool sent;
  uint32_t last_tick;
} beacon_t;

typedef struct 
{
  bool requested, wait_ack;
  uint8_t *buf, fails;
  uint32_t size, offset, last_tick;
  uint16_t curr, total;
} datatx_t;

typedef struct 
{
  bool sent;
  uint32_t poweroff_tick;
} end_t;

comm_error_t Logic_Comm_Init(void);
comm_error_t Logic_Comm_Process(void);
comm_error_t Logic_Comm_Abort(void);
comm_phase_t Logic_Comm_GetPhase(void);
uint32_t Logic_Comm_GetPhaseTimer(void);

#endif