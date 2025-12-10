#ifndef TASKS_CONFIG_H
#define TASKS_CONFIG_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "cubesat_data.h"

//task priority
#define PRIORITY_CMD_RECEIVER   3
#define PRIORITY_STATE_MACHINE  2
#define PRIORITY_OUTPUT_MGR     1

//stack size
#define STACK_CMD_RECEIVER    512
#define STACK_STATE_MACHINE   2048
#define STACK_OUTPUT_MGR      1024

//queue definition
#define CMD_QUEUE_LENGTH      5
#define OUT_QUEUE_LENGTH      10
 
//output request types
typedef enum {
  OUT_LOG,
  OUT_TELEMETRY,
  OUT_FLASH_WRITE,
  OUT_LED
} output_type_t;

//command message structure
typedef struct
{
  gcs_command_t cmd;
  uint32_t timestamp;
  uint8_t payload[32];
  uint16_t payload_len;
} command_msg_t;

typedef struct 
{
  output_type_t type;
  union  
  {
    struct  
    {
      char msg[128];
    }log;

    struct  
    {
      uint8_t data[256];
      uint16_t len;
    }telemetry;

    struct  
    {
      uint32_t addr;
      uint8_t data[512];
      uint16_t len;
    } flash;

    struct  
    {
      uint8_t state; //0=of, 1=on, 2=blimk
    } led;
  } data;
  
}output_request_t;

//shared data structure
typedef struct  
{
  system_health_t health;
  cubesat_status_t status;
  SemaphoreHandle_t mutex;
}shared_data_t;

//event group bits
#define EVENT_LORA_RX           (1 << 0)
#define EVENT_RTC_ALARM         (1 << 1)
#define EVENT_BATTERY_LOW       (1 << 2)
#define EVENT_CRITICAL_ERROR    (1 << 3)

//GLOBAL HANDLE
extern QueueHandle_t cmdQueue;
extern QueueHandle_t outQueue;
extern SemaphoreHandle_t healthMutex;
extern SemaphoreHandle_t i2cMutex;
extern SemaphoreHandle_t sdMutex;
extern EventGroupHandle_t systemEvents;
extern shared_data_t g_shared;

//TASK FUNCTION
void Task_CommandReceiver(void *pvParameters);
void Task_StateMachine(void *pvParameters);
void Task_OutputManager(void *pvParameters);

//helper function
void Task_Init(void);
void SharedData_Init(void);

// Thread-safe accessors
static inline void SharedData_Lock(void) {
    xSemaphoreTake(g_shared.mutex, portMAX_DELAY);
}

static inline void SharedData_Unlock(void) {
    xSemaphoreGive(g_shared.mutex);
}

// Queue helpers
static inline BaseType_t SendCommand(const command_msg_t *cmd, TickType_t timeout) {
    return xQueueSend(cmdQueue, cmd, timeout);
}

static inline BaseType_t ReceiveCommand(command_msg_t *cmd, TickType_t timeout) {
    return xQueueReceive(cmdQueue, cmd, timeout);
}

static inline BaseType_t SendOutput(const output_request_t *req, TickType_t timeout) {
    return xQueueSend(outQueue, req, timeout);
}

static inline BaseType_t ReceiveOutput(output_request_t *req, TickType_t timeout) {
    return xQueueReceive(outQueue, req, timeout);
}

// ISR helpers (FromISR versions)
static inline BaseType_t SendCommandFromISR(const command_msg_t *cmd, 
                                            BaseType_t *pxHigherPriorityTaskWoken) {
    return xQueueSendFromISR(cmdQueue, cmd, pxHigherPriorityTaskWoken);
}

static inline BaseType_t SendOutputFromISR(const output_request_t *req,
                                           BaseType_t *pxHigherPriorityTaskWoken) {
    return xQueueSendFromISR(outQueue, req, pxHigherPriorityTaskWoken);
}

#endif
