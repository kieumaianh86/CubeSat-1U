#ifndef CUBESAT_TYPES_H
#define CUBESAT_TYPES_H

#include "stdint.h"
#include "stdbool.h"

//battery thresholds
#define BATTERY_CRITICAL    20 //chuyen SAFE ngay lap tuc
#define BATTERY_LOW         30 //khong cho phep SCIENCE
#define BATTERY_COMM_MIN    25 //khong cho phep COMM
#define BATTERY_SLEEP       15 //bat buoc SLEEP tu SAFE
#define BATTERY_IDLE_SLEEP  40 //sleep neu idle > 5p

//sensor thresholds
#define SENSOR_MIN_WORKING  3
#define SENSOR_CRITICAL_FAIL  4
#define SENSOR_TOTAL  6

//temperature threshold
#define TEMP_MAX_SAFE   70
#define TEMP_MIN_SAFE   -10

//timeout values (ms)
#define TIMEOUT_INIT              15000
#define TIMEOUT_SCIENCE_TOTAL     60000
#define TIMEOUT_COMM_TOTAL        40000
#define TIMEOUT_SENSOR_INIT       2000
#define TIMEOUT_SENSOR_READ_MIN   1000
#define TIMEOUT_SENSOR_READ_MAX   5000
#define TIMEOUT_SAFE_RETRY        10000
#define TIMEOUT_SAFE_AUTO_EXIT    60000
#define TIMEOUT_SLEEP_WAKE_HIGH   600000 //10p
#define TIMEOUT_SLEEP_WAKE_LOW    1800000 //30p
#define TIMEOUT_LORA_INIT         3000
#define TIMEOUT_BEACON_TX         2000
#define TIMEOUT_RX_LISTEN         15000
#define TIMEOUT_PACKET_TX         500

//retry counts
#define RETRY_SENSOR              3
#define RETRY_FLASH               3
#define RETRY_LORA                1
#define RETRY_BEACON              3
#define RETRY_PACKET_FAIL_MAX     5

//timing
#define IDLE_TIME_FOR_SLEEP       300000 //5p
#define IDLE_TIME_LONG            600000 //10p
#define BATTERY_CHECK_INTERVAL    10000 //10s
#define GCS_POLL_INTERVAL         5000 //5s
#define GCS_RX_TIMEOUT            2000 //2s
#define SAFE_CHECK_INTERVAL       10000 //10s

//flash
#define FLASH_MIN_FREE_KB         5

//lora
#define LORA_PACKET_MAX_SIZE      240

typedef enum {
  STATE_INIT = 0,
  STATE_STANDBY,
  STATE_SCIENCE,
  STATE_COMM,
  STATE_SAFE,
  STATE_SLEEP,
  STATE_COUNT
} cubesat_state_t;

typedef enum {
  SENSOR_GPS = 0,
  SENSOR_DS18B20,
  SENSOR_MPU6050,
  SENSOR_HMC5883L,
  SENSOR_CAMERA,
  SENSOR_COUNT
} sensor_id_t;

typedef enum {
  SENSOR_STATUS_OK = 0,
  SENSOR_STATUS_FAIL,
  SENSOR_STATUS_DEAD,
  SENSOR_STATUS_NOT_INIT
} sensor_status_t;

typedef enum {
  CMD_NONE = 0,
  CMD_PING,
  CMD_GET_STATUS,
  CMD_GET_DATA,
  CMD_SET_CONFIG,
  CMD_RESET,
  CMD_ENTER_SAFE,
  CMD_START_SCIENCE,
  CMD_START_COMM
} gcs_command_t;

typedef enum {
  ERROR_NONE = 0,
  ERROR_WARNING,
  ERROR_ERROR,
  ERROR_CRITICAL
} error_level_t;

typedef enum {
  INIT_RESULT_SUCCESS = 0,
  INIT_RESULT_SLEEP,
  INIT_RESULT_SAFE
} init_result_t;

typedef struct {
  uint8_t battery_percent;
  float temperature;
  uint32_t flash_free_kb;
  uint8_t sensor_status[SENSOR_COUNT];
  uint8_t sensor_fail_count[SENSOR_COUNT];
  uint8_t working_sensor_count;
  bool lora_ok;
  uint8_t lora_fail_count;
} system_health_t;

typedef struct 
{
  cubesat_state_t current_state;
  cubesat_state_t previous_state;
  uint32_t state_entry_time;
  uint32_t idle_time;
  uint32_t last_comm_time;
  uint32_t last_battery_check;
  gcs_command_t pending;
  uint16_t error_code;
  bool sensors_recovered;
  bool flash_recovered;
  bool lora_recovered;
  bool temp_recovered;
} cubesat_status_t;


typedef struct 
{
  uint32_t timestamp;
  uint8_t battery;
  uint8_t sensor_status[SENSOR_COUNT];
  uint32_t flash_free;
  char message[64];
} log_entry_t;

#endif