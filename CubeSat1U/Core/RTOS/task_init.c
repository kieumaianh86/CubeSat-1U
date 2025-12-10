#include "task_config.h"
#include "hardware.h"

//global bandle
QueueHandle_t cmdQueue = NULL;
QueueHandle_t outQueue = NULL;
SemaphoreHandle_t healthMutex = NULL;
SemaphoreHandle_t i2cMutex = NULL;
SemaphoreHandle_t flashMutex = NULL;
EventGroupHandle_t systemEvents = NULL;
shared_data_t g_shared = {0};

// Task handles (for debugging/monitoring)
static TaskHandle_t hTaskCmdRx = NULL;
static TaskHandle_t hTaskStateMachine = NULL;
static TaskHandle_t hTaskOutput = NULL;

void SharedData_Init(void)
{
    memset(&g_shared, 0, sizeof(g_shared));
    
    // Create mutex for shared data
    g_shared.mutex = xSemaphoreCreateMutex();
    configASSERT(g_shared.mutex != NULL);
    
    // Initialize default values
    g_shared.status.current_state = STATE_INIT;
    g_shared.status.previous_state = STATE_INIT;
    g_shared.status.state_entry_time = HAL_GetTick();
    g_shared.status.pending = CMD_NONE;
    
    g_shared.health.battery_percent = 100;
    g_shared.health.temperature = 25.0f;
    g_shared.health.lora_ok = false;
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        g_shared.health.sensor_status[i] = SENSOR_STATUS_NOT_INIT;
        g_shared.health.sensor_fail_count[i] = 0;
    }
}


void Task_Init (void){
  //initialize shared data first
  SharedData_Init();

  //create queue
  cmdQueue = xQueueCreate(CMD_QUEUE_LENGTH, sizeof(command_msg_t));
  configASSERT(cmdQueue != NULL); //tra ve true -> chay tiep, false -> dung lai

  outQueue = xQueueCreate(OUT_QUEUE_LENGTH, sizeof(output_request_t));
  configASSERT(outQueue != NULL);

  //create mutex
  healthMutex = xSemaphoreCreateMutex();
  configASSERT(healthMutex != NULL);

  i2cMutex = xSemaphoreCreateMutex();
  configASSERT(i2cMutex != NULL);

  flashMutex = xSemaphoreCreateMutex();
  configASSERT(flashMutex != NULL);

  //create event group
  systemEvents = xEventGroupCreate();
  configASSERT(systemEvents != NULL);

  //create task
  BaseType_t result;
  result = xTaskCreate(Task_CommandReceiver, "CmdRx", STACK_CMD_RECEIVER, NULL, PRIORITY_CMD_RECEIVER, &hTaskCmdRx);
  configASSERT(result == pdPASS);

  result = xTaskCreate(Task_StateMachine, "StateMachine", STACK_STATE_MACHINE, NULL, PRIORITY_STATE_MACHINE, &hTaskStateMachine);
  configASSERT(result == pdPASS);

  result = xTaskCreate(Task_OutputManager, "Output", STACK_OUTPUT_MGR, NULL, PRIORITY_OUTPUT_MGR, &hTaskOutput);
  configASSERT(result == pdPASS);

  //log initialization
  output_request_t init_log = {
    .type = OUT_LOG
  };

  snprintf(init_log.data.log.msg, sizeof(init_log.data.log.msg), "RTOS: Tasks initialized!\r\n");

  //task monitoring handle
  TaskHandle_t Tasks_GetHandle_CmdRx(void){ return hTaskCmdRx; }
  TaskHandle_t Tasks_GetHandle_StateMachine(void) { return hTaskStateMachine; }
  TaskHandle_t Tasks_GetHandle_Output(void) { return hTaskOutput; }
}