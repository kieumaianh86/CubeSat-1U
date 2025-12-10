#include "task_config.h"
#include "lora_e32.h"
#include "packet_protocol.h"
#include "hardware.h"
#include <string.h>

extern lora_e32_handle_t lora_handle;

//private function
static bool ProcessLoRaPacket(const uint8_t *packet, uint16_t len, command_msg_t *cmd);
static void SendNACK(void);

void Task_CommandReceiver(void *pvParameters)
{
  uint8_t rx_buffer[256];
  command_msg_t cmd_msg;
  //wait a bit for hardware init
  vTaskDelay(pdMS_TO_TICKS(1000));
  while(1)
  {
    //wait for LoRa RX event (blocking, max 200ms)
    EventBits_t events = xEventGroupWaitBits(
      systemEvents,
      EVENT_LORA_RX,
      pdTRUE,
      pdFALSE,
      pdMS_TO_TICKS(200)
    );

    //check if lora data available
    if ((events & EVENT_LORA_RX) || LoRa_E32_DataAvailable(&lora_handle))
    {
      uint16_t rx_len = sizeof(rx_buffer);
      //receive packet (non-blocking with short timeout)
      if (LoRa_E32_Receive(&lora_handle, rx_buffer, &rx_len, 100) == LORA_E32_OK)
      {
        if (rx_len > 0)
        {
          //process packet and build command msg
          if (ProcessLoRaPacket(rx_buffer, rx_len, &cmd_msg))
          {
            //send to command queue (non-blocking)
            if (SendCommand(&cmd_msg, 0) != pdPASS)
            {
              //queue full - log drop
              output_request_t out = {
                .type = OUT_LOG
              };
              snprintf(out.data.log.msg, sizeof(out.data.log.msg), "CMD_RX: Queue full, dropped cmd %d\r\n", cmd_msg.cmd);
              SendOutput(&out, 0);
            }
            
          }
          else
          {
            //invalid packet
            SendNACK();
          }
          
        }
        
      }
      
    }
    taskYIELD();
  }
}

static bool ProcessLoRaPacket(const uint8_t *packet, uint16_t len, command_msg_t *cmd)
{
  //check minimum length
  if (len < 4 || packet[0] != PACKET_HEADER)
  {
    return false;
  }

  //verify crc
  if (!Packet_Verify_CRC(packet, len)) 
  {
    return false;
  }
  //parse packet type and map to command
  packet_type_t pkt_type = Packet_Parse_Type(packet);
  cmd->timestamp = HAL_GetTick();
  cmd->payload_len = 0;

  switch (pkt_type)
  {
  case PKT_TYPE_PING:
    cmd->cmd = CMD_PING;
    if (len >= sizeof(ping_packet_t))
    {
      ping_packet_t *ping = (ping_packet_t*)packet;
      memcpy(cmd->payload, &ping->timestamp, sizeof(ping->timestamp));
      cmd->payload_len = sizeof(ping->timestamp);
    }
    
    break;
  case PKT_TYPE_GET_STATUS:
    cmd->cmd = CMD_GET_STATUS;
    break;
  case PKT_TYPE_GET_DATA:
    cmd->cmd = CMD_GET_DATA;
    break;
  case PKT_TYPE_SET_CONFIG:
    cmd->cmd = CMD_SET_CONFIG;
    if (len > 4)
    {
      uint16_t payload_len = len - 4; //trừ đi 4 byte/header + type + crc
      if (payload_len > sizeof(cmd->payload))
      {
        payload_len = sizeof(cmd->payload);
      }
      memcpy(cmd->payload, &packet[2], payload_len);
      cmd->payload_len = payload_len;
      
    }
    break;
  case PKT_TYPE_RESET:
    cmd->cmd = CMD_RESET;
    break;
  case PKT_TYPE_ENTER_SAFE:
    cmd->cmd = CMD_ENTER_SAFE;
    break;    
  default:
    return false;
    
  }
  return true;
  
}

static void SendNACK(void) 
{
  uint8_t nack[8] = {PACKET_HEADER, PKT_TYPE_NACK};
  uint16_t crc = Packet_Calculate_CRC(nack,2);
  nack[2] = crc >> 8;
  nack[3] = crc & 0xFF;
  //Send NACK
  LoRa_E32_Send(&lora_handle, nack, 4, 500);
}