#include "logic_comm.h"
#include "logic.h"
#include "packet_protocol.h"
#include "lora_e32.h"
#include "hardware.h"
#include "sdcard.h"
#include "OV2640.h"
#include "INA219.h"
#include <string.h>
#include <stdlib.h>

extern lora_e32_handle_t lora_handle;
extern lora_e32_config_t lora_config;

static comm_phase_t current_phase = COMM_PHASE_STANDBY;
static uint32_t comm_start_tick = 0;

static lora_t lora;
static beacon_t beacon;
static datatx_t dtx;
static end_t end_state;
static uint32_t listen_tick;

// Static buffer for data transmission (can store image data)
static uint8_t dtx_buffer[51200];  // 50KB buffer

static void reset(void);
static comm_error_t Logic_Comm_Phase_Prep(void);
static comm_error_t Logic_Comm_Phase_Beacon(void);
static comm_error_t Logic_Comm_Phase_Listen(void);
static comm_error_t Logic_Comm_Phase_DataTx(void);
static comm_error_t Logic_Comm_Phase_End(void);
static void Logic_Comm_Handle_Command(packet_type_t type, const uint8_t* pkt, uint16_t len);

comm_error_t Logic_Comm_Init(void)
{
  reset();
  current_phase = COMM_PHASE_PREP;
  comm_start_tick = HAL_GetTick();
  Logic_Log("COMM: Init\r\n");
  return COMM_OK;
}

comm_error_t Logic_Comm_Process(void)
{
  uint32_t total_timer = HAL_GetTick() - comm_start_tick;
  system_health_t* health = Logic_GetHealth();

  // Check total timeout only, NO fixed phase timeouts
  // Allow flexible timing for camera image transmission
  if (total_timer > TIMEOUT_COMM_TOTAL && current_phase != COMM_PHASE_DATA_TX)
  {
    Logic_Log("COMM: Total timeout\r\n");
    reset();
    return COMM_ERR_TIMEOUT;
  }

  // Battery critical check
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
    if (Logic_Comm_Phase_Prep() == COMM_OK) {
      current_phase = COMM_PHASE_BEACON;
      Logic_Log("COMM: PREP done\r\n");
    }
    break;
    
  case COMM_PHASE_BEACON:
    if (Logic_Comm_Phase_Beacon() == COMM_OK) {
      current_phase = COMM_PHASE_LISTEN;
      listen_tick = HAL_GetTick();
      Logic_Log("COMM: BEACON done\r\n");
    }
    break;
    
  case COMM_PHASE_LISTEN:
    Logic_Comm_Phase_Listen();
    // Timeout or received GET_DATA
    if (HAL_GetTick() - listen_tick > TIMEOUT_RX_LISTEN || dtx.requested) {
      current_phase = dtx.requested ? COMM_PHASE_DATA_TX : COMM_PHASE_END;
      Logic_Log("COMM: LISTEN done\r\n");
    }
    break;
    
  case COMM_PHASE_DATA_TX:
    // NO TIMEOUT - continues until transmission complete or error
    if (Logic_Comm_Phase_DataTx() == COMM_OK) {  
      current_phase = COMM_PHASE_END;
      Logic_Log("COMM: DATA_TX done\r\n");
    }
    break;
    
  case COMM_PHASE_END:
    if (Logic_Comm_Phase_End() == COMM_OK) {
      current_phase = COMM_PHASE_COMPLETE;
    }
    break;

  case COMM_PHASE_COMPLETE:
    current_phase = COMM_PHASE_STANDBY;
    return COMM_OK;
  }
  
  return COMM_IN_PROGRESS;
}

static void reset(void)
{
  if (lora.powered) HW_Power_LoRa_Off();
  memset(&lora, 0, sizeof(lora));
  memset(&beacon, 0, sizeof(beacon));
  memset(&dtx, 0, sizeof(dtx));
  memset(&end_state, 0, sizeof(end_state));
  listen_tick = 0;  
}

static comm_error_t Logic_Comm_Phase_Prep(void) 
{
  static uint8_t step = 0;
    
  switch (step) {
  case 0:
    if (!Logic_Battery_Check_Comm()) {
      step = 0;
      return COMM_ERR_BATTERY_LOW;
    }
    step = 1;
    break;
        
  case 1:
    if (!lora.powered) {
      HW_Power_LoRa_On();
      lora.powered = true;
      lora.last_tick = HAL_GetTick();
    }
    if (HAL_GetTick() - lora.last_tick > 50) {
      step = 2;
    }
    break;
        
  case 2:
    if (!lora.initialized) {
      if (LoRa_E32_Init(&lora_handle, &lora_config) == LORA_E32_OK) {
        lora.initialized = true;
        step = 0;
        return COMM_OK;
      }
      lora.attempts++;
      if (lora.attempts >= RETRY_LORA) {
        step = 0;
        return COMM_ERR_LORA_FAIL;
      }
    }
    break;
  }
    
  return COMM_IN_PROGRESS;
}

static comm_error_t Logic_Comm_Phase_Beacon(void) 
{
    static uint8_t retry = 0;
    system_health_t* health = Logic_GetHealth();
    cubesat_status_t* status = Logic_GetStatus();
    
    if (!beacon.sent) {
        uint8_t buf[32];
        uint16_t len = Packet_Build_Beacon(buf, 0x01, HAL_GetTick()/1000,
                                           health->battery_percent,
                                           (uint8_t)status->current_state,
                                           health->working_sensor_count,
                                           health->flash_free_kb);
        
        if (LoRa_E32_Send(&lora_handle, buf, len, BEACON_TX_TIMEOUT_MS) == LORA_E32_OK) {
            beacon.sent = true;
            retry = 0;
            return COMM_OK;
        }
        
        retry++;
        if (retry >= BEACON_MAX_RETRIES) {
            retry = 0;
            return COMM_ERR_LORA_FAIL;
        }
    }
    
    return beacon.sent ? COMM_OK : COMM_IN_PROGRESS;
}

static comm_error_t Logic_Comm_Phase_Listen(void) 
{
    if (LoRa_E32_DataAvailable(&lora_handle)) {
        uint8_t buf[256];
        uint16_t len = sizeof(buf);
        
        if (LoRa_E32_Receive(&lora_handle, buf, &len, LISTEN_RX_TIMEOUT_MS) == LORA_E32_OK &&
            len > 0 && buf[0] == PACKET_HEADER) {
            
            if (Packet_Verify_CRC(buf, len)) {
                Logic_Comm_Handle_Command(Packet_Parse_Type(buf), buf, len);
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

// NO FIXED TIMEOUT - transmits until complete or max fails reached
static comm_error_t Logic_Comm_Phase_DataTx(void) 
{
    // Initialize buffer on first call
    if (dtx.curr == 0 && dtx.size == 0) {
        dtx.buf = dtx_buffer;
        
        // Read data from SD card (could be sensor data or camera image)
        dtx.size = sizeof(dtx_buffer);
        if (SD_ReadScience(dtx.buf, &dtx.size) != SD_OK) {
            Logic_Log("COMM: SD read failed\r\n");
            dtx.size = 0;
            return COMM_OK;  // Skip transmission
        }
        
        dtx.total = (dtx.size + DATA_PAYLOAD_MAX - 1) / DATA_PAYLOAD_MAX;
        dtx.last_tick = HAL_GetTick();
        Logic_Log("COMM: Data ready, %lu bytes, %u packets\r\n", dtx.size, dtx.total);
    }
    
    // Transmit packets sequentially with ACK
    if (dtx.curr < dtx.total) {
        if (dtx.wait_ack) {
            // Wait for ACK
            uint8_t ack[32];
            uint16_t alen = sizeof(ack), seq;
            
            if (LoRa_E32_DataAvailable(&lora_handle) &&
                LoRa_E32_Receive(&lora_handle, ack, &alen, 100) == LORA_E32_OK &&
                Packet_Parse_ACK(ack, alen, &seq) && seq == dtx.curr + 1) {
                // ACK received
                dtx.fails = 0;
                dtx.curr++;
                dtx.wait_ack = false;
                Logic_Log("COMM: Packet %u/%u ACK\r\n", dtx.curr, dtx.total);
            } else if (HAL_GetTick() - dtx.last_tick > TIMEOUT_ACK_TX) {
                // ACK timeout - retry
                dtx.fails++;
                dtx.wait_ack = false;
                Logic_Log("COMM: Packet %u timeout, retry\r\n", dtx.curr + 1);
                
                if (dtx.fails >= DATA_MAX_FAILS) {
                    Logic_Log("COMM: Max fails reached\r\n");
                    return COMM_OK;  // Complete with partial transmission
                }
            }
        } else {
            // Send next packet
            uint8_t tx[256], pay[DATA_PAYLOAD_MAX];
            uint32_t rem = dtx.size - dtx.offset;
            uint8_t plen = (rem > DATA_PAYLOAD_MAX) ? DATA_PAYLOAD_MAX : rem;
            
            memcpy(pay, &dtx.buf[dtx.offset], plen);
            dtx.offset += plen;
            
            uint16_t tlen = Packet_Build_DataPacket(tx, dtx.curr + 1, dtx.total, pay, plen);
            
            if (LoRa_E32_Send(&lora_handle, tx, tlen, TIMEOUT_PACKET_TX) == LORA_E32_OK) {
                dtx.wait_ack = true;
                dtx.last_tick = HAL_GetTick();
                Logic_Log("COMM: Sent packet %u/%u (%u bytes)\r\n", dtx.curr + 1, dtx.total, plen);
            } else {
                dtx.fails++;
                if (dtx.fails >= DATA_MAX_FAILS) {
                    return COMM_OK;
                }
            }
        }
        
        return COMM_IN_PROGRESS;  // Continue transmission
    } else {
        // All packets sent successfully
        uint8_t end[32];
        uint16_t elen = Packet_Build_End(end, dtx.curr);
        LoRa_E32_Send(&lora_handle, end, elen, 500);
        Logic_Log("COMM: Transmission complete (%lu bytes)\r\n", dtx.size);
        return COMM_OK;  // Done
    }
}

static comm_error_t Logic_Comm_Phase_End(void) 
{
    static uint8_t step = 0;
    cubesat_status_t* status = Logic_GetStatus();
    
    switch (step) {
    case 0:
        {
            uint8_t gb[8] = {PACKET_HEADER, PKT_TYPE_GOODBYE};
            uint16_t crc = Packet_Calculate_CRC(gb, 2);
            gb[2] = crc >> 8;
            gb[3] = crc & 0xFF;
            LoRa_E32_Send(&lora_handle, gb, 4, 500);
            step = 1;
        }
        break;
        
    case 1:
        HW_Power_LoRa_Off();
        lora.powered = false;
        status->last_comm_time = HAL_GetTick();
        step = 0;
        return COMM_OK;   
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
