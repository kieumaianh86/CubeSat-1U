#include "logic_comm.h"
#include "logic.h"
#include "packet_protocol.h"
#include "lora_e32.h"
#include "hardware.h"
#include "INA219.h"
#include <string.h>
#include <stdlib.h>

//external lora handle
extern lora_e32_handle_t lora_handle;
extern lora_e32_config_t lora_config;

static comm_phase_t current_phase = COMM_PHASE_STANDBY;
static uint32_t comm_start_tick = 0;

static lora_t lora;
static beacon_t beacon;
static datatx_t dtx;
static end_t end_state;
static uint32_t listen_tick;

//privfate function declarations
static void reset(void);
static comm_error_t Logic_Comm_Phase_Prep(uint32_t ms);
static comm_error_t Logic_Comm_Phase_Beacon(uint32_t ms);
static comm_error_t Logic_Comm_Phase_Listen(uint32_t ms);
static comm_error_t Logic_Comm_Phase_DataTx(uint32_t ms);
static comm_error_t Logic_Comm_Phase_End(uint32_t ms);
static void Logic_Comm_Handle_Command(packet_type_t type, const uint8_t* pkt, uint16_t len);


comm_error_t Logic_Comm_Init(void)
{
  reset();
  current_phase = COMM_PHASE_PREP;
  comm_start_tick = HAL_GetTick();
  Logic_Log("COMM: Init\r\n");
  return COMM_OK;
}

comm_error_t Logic_Comm_Process(uint32_t ms)
{
  comm_error_t result = COMM_IN_PROGRESS;
  system_health_t* health = Logic_GetHealth();
  uint32_t total_timer = HAL_GetTick() - comm_start_tick;

  //total timeout check (40s)
  if (total_timer > COMM_PHASE5_END_MS)
  {
    Logic_Log("COMM: Total timeout\r\n");
    reset();
    return COMM_ERR_TIMEOUT;
  }

  //battery critical check
  if (Logic_Battery_Check_Critical())
  {
    Logic_Log("COMM: Battery critical\r\n");
    return COMM_ERR_BATTERY_CRITICAL;
  }

  if (health->lora_fail_count >= LORA_FAIL_MAX)
  {
    Logic_Log("COMM: LoRa failed\r\n");
    reset();
    return COMM_ERR_LORA_FAIL;
  }
  

  switch (current_phase)
  {
  case COMM_PHASE_PREP:
    result = Logic_Comm_Phase_Prep(total_timer);
    if (result == COMM_OK && total_timer >= COMM_PHASE1_PREP_MS)
    {
      current_phase = COMM_PHASE_BEACON;
      Logic_Log("COMM: PREP complete\r\n");
      result = COMM_IN_PROGRESS;
    }
    break;
  case COMM_PHASE_BEACON:
    result = Logic_Comm_Phase_Beacon(total_timer);
    if (result == COMM_OK && total_timer >= COMM_PHASE2_SEND_BEACON_MS)
    {
      current_phase = COMM_PHASE_LISTEN;
      listen_tick = HAL_GetTick();
      Logic_Log("COMM: BEACON complete\r\n");
      result = COMM_IN_PROGRESS;
    }
    break;
  case COMM_PHASE_LISTEN:
    result = Logic_Comm_Phase_Listen(total_timer);
    if (total_timer >= COMM_PHASE3_LISTEN_MS)
    {
      current_phase = dtx.requested ? COMM_PHASE_DATA_TX : COMM_PHASE_END;
      Logic_Log("COMM: LISTEN complete\r\n");
      result = COMM_IN_PROGRESS;
    }
    break;
    
  case COMM_PHASE_DATA_TX:
    result = Logic_Comm_Phase_DataTx(total_timer);
    if (result == COMM_OK || total_timer >= COMM_PHASE4_DATA_TX_MS)
    {
      current_phase = COMM_PHASE_END;
      Logic_Log("COMM: DATA_TX complete\r\n");
      result = COMM_IN_PROGRESS;
    }
    break;
  case COMM_PHASE_END:
    result = Logic_Comm_Phase_End(total_timer);
    if (total_timer >= COMM_PHASE5_END_MS)
    {
      current_phase = COMM_PHASE_COMPLETE;
      Logic_Log("COMM: Complete\r\n");
    }
    break;

  case COMM_PHASE_COMPLETE:
    current_phase = COMM_PHASE_STANDBY;
    Logic_Log("COMM: Complete\r\n");
    return COMM_OK;

  default:
    break;
  }

  if (result != COMM_IN_PROGRESS && result != COMM_OK)
  {
    reset();
  }
  
  
  return result;
}

static void reset(void)
{
  if (lora.powered) HW_Power_LoRa_Off();
  if (dtx.buf) free(dtx.buf);
  memset(&lora, 0, sizeof(lora));
  memset(&beacon, 0, sizeof(beacon));
  memset(&dtx, 0, sizeof(dtx));
  memset(&end_state, 0, sizeof(end_state));
  listen_tick = 0;  
}


static comm_error_t Logic_Comm_Phase_Prep(uint32_t ms)
{
  uint32_t now = HAL_GetTick();
  system_health_t* health = Logic_GetHealth();

  if (!Logic_Battery_Check_Comm())
  {
    Logic_Log("COMM PREP: Battery < 25%%\r\n");
    return COMM_ERR_BATTERY_LOW;
  }
  if (!lora.powered && ms >= LORA_POWER_DELAY_MS)
  {
    HW_Power_LoRa_On();
    lora.powered = true;
    lora.last_tick = now;
  }
  //init with retry
  if (lora.powered && !lora.initialized && ms >= LORA_INIT_START_MS && ms < LORA_INIT_TIMEOUT_MS)
  {
    if (lora.attempts == 0 || (now - lora.last_tick) >= LORA_RETRY_DELAY_MS)
    {
      if (lora.attempts < LORA_MAX_ATTEMPTS)
      {
        if (LoRa_E32_Init(&lora_handle, &lora_config) == LORA_E32_OK)
        {
          lora.initialized = true;
          LoRa_E32_SetMode(&lora_handle, LORA_E32_MODE_NORMAL);
        }
        lora.attempts++;
        lora.last_tick = now;
      }
      
      if (lora.attempts >= LORA_MAX_ATTEMPTS && !lora.initialized)
      {
        health->lora_fail_count++;
        return COMM_ERR_LORA_FAIL;
      }
      
    }
    
  }
  
  if (ms >= LORA_INIT_TIMEOUT_MS && !lora.initialized)
  {
    health->lora_fail_count++;
    return COMM_ERR_LORA_FAIL;
  }
  
  //csma (lang nghe xem kenh co ban khong)
  if (lora.initialized && ms >= CSMA_CHECK_TIME_MS && ms < CSMA_CHECK_TIME_MS+100)
  {
    if (LoRa_E32_DataAvailable(&lora_handle)) {
      Logic_Log("COMM: Channel busy\r\n");
      }
  }
  return (ms >= COMM_PHASE1_PREP_MS && lora.initialized) ? COMM_OK : COMM_IN_PROGRESS;
}


static comm_error_t Logic_Comm_Phase_Beacon(uint32_t ms)
{
  //phase 2: send beacon (5-10s)
  uint32_t now = HAL_GetTick();
  system_health_t* health = Logic_GetHealth();
  cubesat_status_t* status = Logic_GetStatus();

  if (ms < COMM_PHASE1_PREP_MS || ms >= COMM_PHASE2_SEND_BEACON_MS) {
        return COMM_IN_PROGRESS;
    }
    
    if (!beacon.sent && beacon.retry < BEACON_MAX_RETRIES) {
      if (beacon.retry == 0 || (now - beacon.last_tick) >= BEACON_RETRY_DELAY_MS) {
          uint8_t buf[32];
          uint16_t len = Packet_Build_Beacon(buf, 0x01, ms/1000, health->battery_percent,
                                                (uint8_t)status->current_state, 
                                                health->working_sensor_count, health->flash_free_kb);
            
          beacon.sent = (LoRa_E32_Send(&lora_handle, buf, len, BEACON_TX_TIMEOUT_MS) == LORA_E32_OK);
          beacon.retry++;
          beacon.last_tick = now;
            
          if (!beacon.sent && beacon.retry >= BEACON_MAX_RETRIES) {
            health->lora_fail_count++;
            return COMM_ERR_LORA_FAIL;
          }
      }
    }
    
    if (ms >= COMM_PHASE2_SEND_BEACON_MS && !beacon.sent) {
        health->lora_fail_count++;
        return COMM_ERR_LORA_FAIL;
    }
    
    return beacon.sent ? COMM_OK : COMM_IN_PROGRESS;
  
}

static comm_error_t Logic_Comm_Phase_Listen(uint32_t ms)
{
    uint32_t now = HAL_GetTick();
    
    if (ms < COMM_PHASE2_SEND_BEACON_MS || (now - listen_tick) < LISTEN_POLL_INTERVAL_MS) {
        return COMM_IN_PROGRESS;
    }
    
    listen_tick = now;
    
    if (LoRa_E32_DataAvailable(&lora_handle)) {
        uint8_t buf[256];
        uint16_t len = sizeof(buf);
        
        if (LoRa_E32_Receive(&lora_handle, buf, &len, LISTEN_RX_TIMEOUT_MS) == LORA_E32_OK &&
            len > 0 && buf[0] == PACKET_HEADER) {
            
            if (Packet_Verify_CRC(buf, len)) {
                Logic_Comm_Handle_Command(Packet_Parse_Type(buf), buf, len);
            } else {
                uint8_t nack[8] = {PACKET_HEADER, PKT_TYPE_NACK};
                uint16_t crc = Packet_Calculate_CRC(nack, 2);
                nack[2] = crc >> 8;
                nack[3] = crc & 0xFF;
                LoRa_E32_Send(&lora_handle, nack, 4, 500);
            }
        }
    }
    
    return COMM_IN_PROGRESS;
}

static void Logic_Comm_Handle_Command(packet_type_t type, const uint8_t* pkt, uint16_t len)
{
  uint8_t resp[256];
    uint16_t rlen = 0;
    system_health_t* h = Logic_GetHealth();
    cubesat_status_t* s = Logic_GetStatus();
    
    switch (type) {
    case PKT_TYPE_PING:
        rlen = Packet_Build_Pong(resp, ((ping_packet_t*)pkt)->timestamp);
        break;
        
    case PKT_TYPE_GET_STATUS: {
        status_packet_t st = {
            .header = PACKET_HEADER, .type = PKT_TYPE_STATUS,
            .current_state = (uint8_t)s->current_state,
            .battery_percent = h->battery_percent,
            .sensor_mpu6050 = h->sensor_status[SENSOR_MPU6050],
            .sensor_hmc5883l = h->sensor_status[SENSOR_HMC5883L],
            .sensor_tmp117 = h->sensor_status[SENSOR_DS18B20],
            .sensor_ov2640 = h->sensor_status[SENSOR_CAMERA],
            .sensor_gps = h->sensor_status[SENSOR_GPS],
            .flash_free_kb = h->flash_free_kb,
            .flash_used_kb = 4096 - h->flash_free_kb
        };
        rlen = Packet_Build_Status(resp, &st);
        break;
    }
    
    case PKT_TYPE_GET_DATA:
        dtx.requested = true;
        rlen = Packet_Build_ACK(resp, 0);
        break;
        
    case PKT_TYPE_SET_CONFIG:
        rlen = Packet_Build_ACK(resp, 0);
        break;
        
    case PKT_TYPE_RESET:
        rlen = Packet_Build_ACK(resp, 0);
        LoRa_E32_Send(&lora_handle, resp, rlen, 500);
        NVIC_SystemReset();
        return;
        
    case PKT_TYPE_ENTER_SAFE:
        Logic_Command_Set(CMD_ENTER_SAFE);
        rlen = Packet_Build_ACK(resp, 0);
        break;
        
    default: {
        resp[0] = PACKET_HEADER;
        resp[1] = PKT_TYPE_NACK;
        uint16_t crc = Packet_Calculate_CRC(resp, 2);
        resp[2] = crc >> 8;
        resp[3] = crc & 0xFF;
        rlen = 4;
        break;
    }
    }
    
    if (rlen > 0) {
        LoRa_E32_Send(&lora_handle, resp, rlen, 500);
    }
}

static comm_error_t Logic_Comm_Phase_DataTx(uint32_t ms)
{
  uint32_t now = HAL_GetTick();
  if (ms < COMM_PHASE3_LISTEN_MS) return COMM_IN_PROGRESS;
    
    // Init
    if (dtx.curr == 0 && !dtx.buf) {
        dtx.size = 50000;
        dtx.buf = malloc(dtx.size);
        if (!dtx.buf) return COMM_OK;
        
        // TODO: Flash_Read_Science_Data(dtx.buf, &dtx.size);
        
        dtx.total = (dtx.size + DATA_PAYLOAD_MAX - 1) / DATA_PAYLOAD_MAX;
        dtx.last_tick = now;
    }
    
    // TX loop
    if (dtx.curr < dtx.total) {
        if (dtx.wait_ack) {
            uint8_t ack[32];
            uint16_t alen = sizeof(ack), seq = 0;
            
            if (LoRa_E32_DataAvailable(&lora_handle) &&
                LoRa_E32_Receive(&lora_handle, ack, &alen, LISTEN_RX_TIMEOUT_MS) == LORA_E32_OK &&
                Packet_Parse_ACK(ack, alen, &seq) && seq == dtx.curr + 1) {
                
                dtx.fails = 0;
                dtx.curr++;
                dtx.wait_ack = false;
            } else if ((now - dtx.last_tick) >= DATA_ACK_TIMEOUT_MS) {
                dtx.fails++;
                dtx.wait_ack = false;
                if (dtx.fails >= DATA_MAX_FAILS) return COMM_OK;
            }
        } else {
            uint8_t tx[256], pay[DATA_PAYLOAD_MAX];
            uint32_t rem = dtx.size - dtx.offset;
            uint8_t plen = (rem > DATA_PAYLOAD_MAX) ? DATA_PAYLOAD_MAX : rem;
            
            if (dtx.curr == 0) {
                science_data_first_t* f = (science_data_first_t*)pay;
                memset(f, 0, sizeof(*f));
                f->has_image = 1;
                f->image_size = dtx.size;
                uint8_t hs = sizeof(*f);
                memcpy(&pay[hs], &dtx.buf[dtx.offset], plen - hs);
                dtx.offset += plen - hs;
            } else {
                memcpy(pay, &dtx.buf[dtx.offset], plen);
                dtx.offset += plen;
            }
            
            uint16_t tlen = Packet_Build_DataPacket(tx, dtx.curr + 1, dtx.total, pay, plen);
            
            if (LoRa_E32_Send(&lora_handle, tx, tlen, DATA_PACKET_TIMEOUT_MS) == LORA_E32_OK) {
                dtx.wait_ack = true;
                dtx.last_tick = now;
            } else {
                dtx.fails++;
                if (dtx.fails >= DATA_MAX_FAILS) return COMM_OK;
            }
        }
    }
    
    // End
    if (dtx.curr >= dtx.total || ms >= COMM_PHASE4_DATA_TX_MS) {
        uint8_t end[32];
        uint16_t elen = Packet_Build_End(end, dtx.curr);
        LoRa_E32_Send(&lora_handle, end, elen, DATA_PACKET_TIMEOUT_MS);
        return COMM_OK;
    }
    
    return COMM_IN_PROGRESS;
  
}

static comm_error_t Logic_Comm_Phase_End(uint32_t ms)
{
  uint32_t now = HAL_GetTick();
  cubesat_status_t* s = Logic_GetStatus();

  if (ms < COMM_PHASE4_DATA_TX_MS) 
  {
    return COMM_IN_PROGRESS;
  }
  if (!end_state.sent && ms >= (COMM_PHASE4_DATA_TX_MS + END_GOODBYE_DELAY_MS))
  {
    uint8_t gb[8] = {PACKET_HEADER, PKT_TYPE_GOODBYE};
    uint16_t crc = Packet_Calculate_CRC(gb,2);
    gb[2] = crc >> 8;
    gb[3] = crc & 0xFF;
    LoRa_E32_Send(&lora_handle, gb, 4, 500);
    end_state.sent = true;
    end_state.poweroff_tick = now;
  }
  if (end_state.sent && (now - end_state.poweroff_tick) >= END_POWEROFF_DELAY_MS && lora.powered) {
    HW_Power_LoRa_Off();
    lora.powered = false;
    s->last_comm_time = now;
  }
    
    return COMM_IN_PROGRESS;
  
}

comm_error_t Logic_Comm_Abort(void)
{
  reset();
  current_phase = COMM_PHASE_STANDBY;
  return COMM_OK;
}

comm_phase_t Logic_Comm_GetPhase(void) { return current_phase; }
uint32_t Logic_Comm_GetPhaseTimer(void) { return HAL_GetTick() - comm_start_tick; }