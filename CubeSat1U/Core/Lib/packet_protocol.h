#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include "stdint.h"
#include "stdbool.h"

//packet constant
#define PACKET_HEADER               0xAA
#define PACKET_MAX_SIZE             240
#define PACKET_PAYLOAD_MAX          230
#define PACKET_DATA_PAYLOAD_MAX     180

//packet types
typedef enum {
  PKT_TYPE_BEACON   = 0x01,
  PKT_TYPE_STATUS   = 0x02,
  PKT_TYPE_DATA     = 0x03,
  PKT_TYPE_ACK      = 0x04,
  PKT_TYPE_NACK     = 0x05,
  PKT_TYPE_END      = 0x06,
  PKT_TYPE_ACK_END   = 0x07,
  PKT_TYPE_PING   = 0x10,
  PKT_TYPE_PONG   = 0x11,
  PKT_TYPE_GET_STATUS   = 0x20,
  PKT_TYPE_GET_DATA   = 0x21,
  PKT_TYPE_SET_CONFIG   = 0x22,
  PKT_TYPE_RESET   = 0x23,
  PKT_TYPE_ENTER_SAFE   = 0x24,
  PKT_TYPE_GOODBYE   = 0xFF,
} packet_type_t;

//generic packet header
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t type;
} packet_header_t;

//beacon packet
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t type;
  uint8_t satellite_id;
  uint32_t uptime_sec;
  uint8_t battery_percent;
  uint8_t current_state;
  uint8_t sensors_ok;
  uint32_t flash_free_kb;
  uint16_t crc;
} beacon_packet_t;

//status pakcet
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t type;
  uint8_t current_state;
  uint8_t battery_percent;
  //sensor status
  uint8_t sensor_mpu6050;
  uint8_t sensor_hmc5883l;
  uint8_t sensor_ds18b20;
  uint8_t sensor_ov2640;
  uint8_t sensor_gps;
  //flash info
  uint32_t flash_free_kb;
  uint32_t flash_used_kb;
  //packet counters
  uint32_t packets_sent;
  uint32_t packets_failed;
  uint16_t crc;
} status_packet_t;

//science data packet header
typedef struct __attribute__((packed))
{
  uint8_t header;
  uint8_t type;
  uint16_t sequence_num;
  uint16_t total_packets;
  uint8_t paylaod_size;
} data_packet_header_t;

typedef struct __attribute__((packed))
{
  //timestap
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  //imu data
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  //magnetometer
  int16_t mag_raw_x;
  int16_t mag_raw_y;
  int16_t mag_raw_z;
  float heading;
  //temperature
  float temperature;
  float humidity;
  //camera info
  uint8_t has_image; //0 = NO, 1 = YES
  uint32_t image_size;
} science_data_first_t;

//ack packet
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t type;
  uint16_t acked_seq;
  uint16_t crc;
} ack_packet_t;

//end packet
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t type;
  uint16_t total_sent;
  uint16_t crc;
} ping_packet_t;

//command packet (from gcs)
typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_T type;
  uint8_t payload[16];
  uint16_t crc;
} command_packet_t;

//crc function
uint16_t packet_calculate_crc(const uint8_t* data, uint16_t length);
bool packet_verify_crc(const uint8_t* packet, uint16_t length);

//packet build functions
uint16_t packet_build_beacon(uint8_t* buffer, uint8_t sat_id, uint32_t uptime,
                              uint8_t battery, uint8_t state, uint8_t sensors_ok,
                              uint8_t flash_free);
uint16_t Packet_Build_Status(uint8_t* buffer, const status_packet_t* status);
uint16_t Packet_Build_Pong(uint8_t* buffer, uint32_t timestamp);
uint16_t Packet_Build_ACK(uint8_t* buffer, uint16_t seq_num);
uint16_t Packet_Build_End(uint8_t* buffer, uint16_t total_sent);
uint16_t Packet_Build_DataPacket(uint8_t* buffer, uint16_t seq, uint16_t total, 
                                  const uint8_t* payload, uint8_t payload_size);

//packet parse function
packet_type_t Packet_Parse_Type(const uint8_t* packet);
bool Packet_Parse_Command(const uint8_t* packet, uint16_t length, command_packet_t* cmd);
bool Packet_Parse_ACK(const uint8_t* packet, uint16_t length, uint16_t* acked_seq);


#endif