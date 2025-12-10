#include "task_config.h"
#include "cubesat_data.h"
#include "logic.h"
#include "logic_science.h"
#include "logic_comm.h"
#include "logic_safe.h"
#include "logic_sleep.h"
#include "hardware.h"
#include <stdio.h>
#include <stdint.h>

extern hardware_t hw;

static void ProcessCommand(const command_msg_t *cmd);
static uint32_t GetStatePeriod(cubesat_state_t state);
static void CheckCriticalConditions(void);
static cubesat_state_t ProcessState(cubesat_state_t current);
static void TransitionTo(cubesat_state_t new_state);

void Task_StateMachine(void *pvParameters)
{
  command_msg_t cmd;
  cubesat_state_t next_state;
  TickType_t period_ticks;
  
  //initial state
  g_shared.status.current_state = STATE_INIT;
  g_shared.status.state_entry_time = HAL_GetTick();

  while(1)
  {
    //get period for current state
    period_ticks = pdMS_TO_TICKS(GetStatePeriod(g_shared.status.current_state));

    //check for command (non-blocking)
    if (ReceiveCommand(&cmd, 0) == pdPASS)
    {
      ProcessCommand(&cmd);
    }
    //lock shared data for state processing
    SharedData_Lock();
    //check critical conditions first
    CheckCriticalConditions();
    //process current state and get next state
    next_state = ProcessState(g_shared.status.current_state);
    //handle state transition
    if (next_state != g_shared.status.current_state)
    {
      TransitionTo(next_state);
    }
    SharedData_Unlock();
    //delay according to state period
    vTaskDelay(period_ticks);
    
  }

}

static void ProcessCommand(const command_msg_t *cmd)
{
  SharedData_Lock();
  switch(cmd->cmd)
  {
  case CMD_PING:
  {//handled by COMM STATE
    if (g_shared.status.current_state == STATE_COMM)
    {
      g_shared.status.pending = CMD_PING;
    }
    break;
  }
  case CMD_GET_STATUS:
  {
    if (g_shared.status.current_state == STATE_COMM)
    {
      g_shared.status.pending = CMD_GET_STATUS;
    }
    break;
  }
  case CMD_SET_CONFIG:
  {
  //apply config change
    g_shared.status.pending = CMD_SET_CONFIG;
    break;
  }
  case CMD_RESET:
  {//send output log before reset
    output_request_t out = {.type = OUT_LOG};
    snprintf(out.data.log.msg, sizeof(out.data.log.msg), "RESET requested\r\n");
    SendOutput(&out, 100);

    SharedData_Unlock();
    vTaskDelay(pdMS_TO_TICKS(500));
    HAL_NVIC_SystemReset();
    break;
  }
  case CMD_ENTER_SAFE:
    {g_shared.status.pending = CMD_ENTER_SAFE;
    break;}
  case CMD_START_SCIENCE:
  {
    if (g_shared.status.current_state == STATE_STANDBY)
    {
      g_shared.status.pending = CMD_START_SCIENCE;
    }
    break;
  }
  case CMD_START_COMM:
  {
    if ((g_shared.status.current_state == STATE_STANDBY) || (g_shared.status.current_state == STATE_SCIENCE))
    {
      g_shared.status.pending = CMD_START_COMM;
    }
    break;
  }
  default:
    break;  
  }
  SharedData_Unlock();
}

static void CheckCriticalCondition(void)
{
  //battery critical -> force SAFE
  if (g_shared.health.battery_percent < BATTERY_CRITICAL)
  {
    if (g_shared.status.current_state != STATE_SAFE &&
       g_shared.status.current_state != STATE_SLEEP)
    {
      g_shared.status.error_code = 0x01;
      TransitionTo(STATE_SAFE);
    }
    
  }
  //sensor critical fail -> SAFE
  if (g_shared.health.working_sensor_count < (SENSOR_TOTAL - SENSOR_CRITICAL_FAIL + 1))
  {
    if (g_shared.status.current_state != STATE_SAFE)
    {
      g_shared.status.error_code = 0x02; //sensor critical
      TransitionTo(STATE_SAFE);
    }
    
  }
  // Temperature out of range -> SAFE
  if (g_shared.health.temperature > TEMP_MAX_SAFE || 
      g_shared.health.temperature < TEMP_MIN_SAFE)
    {
    if (g_shared.status.current_state != STATE_SAFE)
      {
        g_shared.status.error_code = 0x03; // Temperature critical
        TransitionTo(STATE_SAFE);
      }
    }
  
  
}

static cubesat_state_t ProcessState(cubesat_state_t current)
{
  switch (current)
  {
  case STATE_INIT:
  {
    hw_status_t hw_status = hardware_init_process(&hw);
    if (hw_status == HW_OK)
    {
      return STATE_STANDBY;
    }
    else if (g_shared.health.battery_percent < BATTERY_SLEEP)
    {
      return STATE_SLEEP;
    }
    else if (hw_status == HW_ERROR)
    {
      g_shared.status.error_code = 0x10;
      return STATE_SAFE;
    }
    //check timeout
    if ((HAL_GetTick() - g_shared.status.state_entry_time) > TIMEOUT_INIT)
    {
      return STATE_STANDBY; 
    }
    break;
  }
  case STATE_STANDBY:
  {
    //check pending command
    if (g_shared.status.pending == CMD_START_SCIENCE)
    {
      if (g_shared.health.battery_percent >= BATTERY_LOW
          && g_shared.health.working_sensor_count >= SENSOR_MIN_WORKING)
      {
        g_shared.status.pending = CMD_NONE;
        Logic_Science_Init();
        return STATE_SCIENCE;
      }      
    }
    if (g_shared.status.pending == CMD_START_COMM)
    {
      if (g_shared.health.battery_percent >= BATTERY_COMM_MIN
      && g_shared.health.lora_ok)
      {
        g_shared.status.pending = CMD_NONE;
        Logic_Comm_Init();
        return STATE_COMM;
      }
    }
    if (g_shared.status.pending == CMD_ENTER_SAFE)
    {
      g_shared.status.pending = CMD_NONE;
      return STATE_SAFE;
    }
    
    //auto sleep conditions
    uint32_t idle = HAL_GetTick() - g_shared.status.state_entry_time;
    if (g_shared.health.battery_percent < BATTERY_LOW ||
        (idle > IDLE_TIME_FOR_SLEEP && g_shared.health.battery_percent < BATTERY_IDLE_SLEEP))
    {
      return STATE_SLEEP;
    }
    break;    
  }
  case STATE_SCIENCE:
  {
    science_error_t result = Logic_Science_Process();

    if (result == SCIENCE_OK)
    {
      Logic_Science_Abort();
      return STATE_STANDBY;
    }
    else if (result == SCIENCE_ERR_BATTERY_CRITICAL || SCIENCE_ERR_FLASH_FAIL)
    {
      Logic_Science_Abort();
      g_shared.status.error_code = 0x20 | result;
      return STATE_SAFE;
    }
    else if (result == SCIENCE_ERR_EMERGENCY_CMD)
    {
      Logic_Science_Abort();
      Logic_Comm_Init();
      return STATE_COMM;
    }
    break;
    
    
  }

  case STATE_COMM:
  {
    comm_error_t result = Logic_Comm_Process();
    if (result == COMM_OK)
    {
      Logic_Comm_Abort();
      g_shared.status.error_code = 0x30 | result;
      return STATE_SAFE;
    }
    break;
  }
  case STATE_SAFE:
  {
    safe_error_t result = Logic_Safe_Process();
    if (result == SAFE_EXIT_TO_STANDBY)
    {
      Logic_Safe_Reset();
      return STATE_STANDBY;
    }
    else if (result == SAFE_EXIT_TO_SLEEP)
    {
      Logic_Safe_Reset();
      return STATE_SLEEP;
    }
    break;
    
  }
  case STATE_SLEEP:
  {
    Logic_Sleep_Enter();
    //after wakeup
    wake_source_t source = WAKE_SOURCE_RTC;
    sleep_error_t result = Logic_Sleep_WakeUp(source);
  }
  default:
    break;
  }
}