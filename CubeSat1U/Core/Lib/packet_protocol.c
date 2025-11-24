#include "packet_protocol.h"
#include "string.h"

// CRC-16-CCITT lookup table (simplified)
static uint16_t crc16_ccitt(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
            {
                crc = (crc << 1) ^ 0x1021;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}

uint16_t Packet_Calculate_CRC(const uint8_t* data, uint16_t length)
{
    return crc16_ccitt(data, length);
}

bool Packet_Verify_CRC(const uint8_t* packet, uint16_t length)
{
    if (length < 4) // Minimum: header + type + crc
    {
        return false;
    }
    
    // CRC is last 2 bytes
    uint16_t received_crc = (uint16_t)(packet[length - 2] << 8) | packet[length - 1];
    uint16_t calculated_crc = Packet_Calculate_CRC(packet, length - 2);
    
    return (received_crc == calculated_crc);
}

uint16_t Packet_Build_Beacon(uint8_t* buffer, uint8_t sat_id, uint32_t uptime, 
                              uint8_t battery, uint8_t state, uint8_t sensors_ok, 
                              uint32_t flash_free)
{
    beacon_packet_t* pkt = (beacon_packet_t*)buffer;
    
    pkt->header = PACKET_HEADER;
    pkt->type = PKT_TYPE_BEACON;
    pkt->satellite_id = sat_id;
    pkt->uptime_sec = uptime;
    pkt->battery_percent = battery;
    pkt->current_state = state;
    pkt->sensors_ok = sensors_ok;
    pkt->flash_free_kb = flash_free;
    
    // Calculate CRC (excluding CRC field itself)
    uint16_t crc = Packet_Calculate_CRC(buffer, sizeof(beacon_packet_t) - 2);
    pkt->crc = crc;
    
    return sizeof(beacon_packet_t);
}

uint16_t Packet_Build_Status(uint8_t* buffer, const status_packet_t* status)
{
    memcpy(buffer, status, sizeof(status_packet_t) - 2);
    
    status_packet_t* pkt = (status_packet_t*)buffer;
    pkt->header = PACKET_HEADER;
    pkt->type = PKT_TYPE_STATUS;
    
    uint16_t crc = Packet_Calculate_CRC(buffer, sizeof(status_packet_t) - 2);
    pkt->crc = crc;
    
    return sizeof(status_packet_t);
}

uint16_t Packet_Build_Pong(uint8_t* buffer, uint32_t timestamp)
{
    ping_packet_t* pkt = (ping_packet_t*)buffer;
    
    pkt->header = PACKET_HEADER;
    pkt->type = PKT_TYPE_PONG;
    pkt->timestamp = timestamp;
    
    uint16_t crc = Packet_Calculate_CRC(buffer, sizeof(ping_packet_t) - 2);
    pkt->crc = crc;
    
    return sizeof(ping_packet_t);
}

uint16_t Packet_Build_ACK(uint8_t* buffer, uint16_t seq_num)
{
    ack_packet_t* pkt = (ack_packet_t*)buffer;
    
    pkt->header = PACKET_HEADER;
    pkt->type = PKT_TYPE_ACK;
    pkt->acked_seq = seq_num;
    
    uint16_t crc = Packet_Calculate_CRC(buffer, sizeof(ack_packet_t) - 2);
    pkt->crc = crc;
    
    return sizeof(ack_packet_t);
}

uint16_t Packet_Build_End(uint8_t* buffer, uint16_t total_sent)
{
    end_packet_t* pkt = (end_packet_t*)buffer;
    
    pkt->header = PACKET_HEADER;
    pkt->type = PKT_TYPE_END;
    pkt->total_sent = total_sent;
    
    uint16_t crc = Packet_Calculate_CRC(buffer, sizeof(end_packet_t) - 2);
    pkt->crc = crc;
    
    return sizeof(end_packet_t);
}

uint16_t Packet_Build_DataPacket(uint8_t* buffer, uint16_t seq, uint16_t total, 
                                  const uint8_t* payload, uint8_t payload_size)
{
    if (payload_size > PACKET_DATA_PAYLOAD_MAX)
    {
        payload_size = PACKET_DATA_PAYLOAD_MAX;
    }
    
    // Build header
    buffer[0] = PACKET_HEADER;
    buffer[1] = PKT_TYPE_DATA;
    buffer[2] = (seq >> 8) & 0xFF;
    buffer[3] = seq & 0xFF;
    buffer[4] = (total >> 8) & 0xFF;
    buffer[5] = total & 0xFF;
    buffer[6] = payload_size;
    
    // Copy payload
    memcpy(&buffer[7], payload, payload_size);
    
    // Calculate CRC
    uint16_t packet_len = 7 + payload_size;
    uint16_t crc = Packet_Calculate_CRC(buffer, packet_len);
    buffer[packet_len] = (crc >> 8) & 0xFF;
    buffer[packet_len + 1] = crc & 0xFF;
    
    return packet_len + 2; // Total packet size including CRC
}

packet_type_t Packet_Parse_Type(const uint8_t* packet)
{
    if (packet[0] != PACKET_HEADER)
    {
        return 0; // Invalid packet
    }
    
    return (packet_type_t)packet[1];
}

bool Packet_Parse_Command(const uint8_t* packet, uint16_t length, command_packet_t* cmd)
{
    if (length < 4 || packet[0] != PACKET_HEADER)
    {
        return false;
    }
    
    if (!Packet_Verify_CRC(packet, length))
    {
        return false;
    }
    
    cmd->header = packet[0];
    cmd->type = packet[1];
    
    // Copy payload (if any)
    uint16_t payload_len = length - 4; // header + type + crc(2)
    if (payload_len > sizeof(cmd->payload))
    {
        payload_len = sizeof(cmd->payload);
    }
    memcpy(cmd->payload, &packet[2], payload_len);
    
    return true;
}

bool Packet_Parse_ACK(const uint8_t* packet, uint16_t length, uint16_t* acked_seq)
{
    if (length < sizeof(ack_packet_t) || packet[0] != PACKET_HEADER)
    {
        return false;
    }
    
    if (packet[1] != PKT_TYPE_ACK)
    {
        return false;
    }
    
    if (!Packet_Verify_CRC(packet, length))
    {
        return false;
    }
    
    ack_packet_t* ack = (ack_packet_t*)packet;
    *acked_seq = ack->acked_seq;
    
    return true;
}
