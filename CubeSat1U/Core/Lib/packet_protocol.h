#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include "stdint.h"
#include "stdbool.h"

// ========== PACKET CONSTANTS ==========
#define PACKET_HEADER           0xAA
#define PACKET_MAX_SIZE         240     // LoRa E32 max
#define PACKET_PAYLOAD_MAX      230     // Header(1) + Type(1) + Payload + CRC(2) = 240
#define PACKET_DATA_PAYLOAD_MAX 180     // Actual usable payload for data

// ========== PACKET TYPES ==========
typedef enum {
    PKT_TYPE_BEACON     = 0x01,
    PKT_TYPE_STATUS     = 0x02,
    PKT_TYPE_DATA       = 0x03,
    PKT_TYPE_ACK        = 0x04,
    PKT_TYPE_NACK       = 0x05,
    PKT_TYPE_END        = 0x06,
    PKT_TYPE_ACK_END    = 0x07,
    PKT_TYPE_PING       = 0x10,
    PKT_TYPE_PONG       = 0x11,
    PKT_TYPE_GET_STATUS = 0x20,
    PKT_TYPE_GET_DATA   = 0x21,
    PKT_TYPE_SET_CONFIG = 0x22,
    PKT_TYPE_RESET      = 0x23,
    PKT_TYPE_ENTER_SAFE = 0x24,
    PKT_TYPE_GOODBYE    = 0xFF
} packet_type_t;

// ========== PACKET STRUCTURES ==========

// Generic packet header
typedef struct __attribute__((packed)) {
    uint8_t header;     // 0xAA
    uint8_t type;       // packet_type_t
} packet_header_t;

// Beacon Packet (20 bytes total)
typedef struct __attribute__((packed)) {
    uint8_t header;         // 0xAA
    uint8_t type;           // PKT_TYPE_BEACON
    uint8_t satellite_id;   // e.g., 0x01
    uint32_t uptime_sec;    // seconds since boot
    uint8_t battery_percent;
    uint8_t current_state;  // cubesat_state_t
    uint8_t sensors_ok;     // number of working sensors
    uint32_t flash_free_kb;
    uint16_t crc;
} beacon_packet_t;

// Status Packet (detailed status response)
typedef struct __attribute__((packed)) {
    uint8_t header;             // 0xAA
    uint8_t type;               // PKT_TYPE_STATUS
    uint8_t current_state;
    uint8_t battery_percent;
    uint8_t charging;           // 0 = NO, 1 = YES
    // Sensor status (6 sensors)
    uint8_t sensor_mpu6050;     // 0 = OK, 1 = FAIL, 2 = DEAD
    uint8_t sensor_hmc5883l;
    uint8_t sensor_tmp117;
    uint8_t sensor_ov2640;
    uint8_t sensor_light;
    uint8_t sensor_gps;
    // Flash info
    uint32_t flash_free_kb;
    uint32_t flash_used_kb;
    // Packet counters
    uint32_t packets_sent;
    uint32_t packets_failed;
    uint16_t crc;
} status_packet_t;

// Science Data Packet Header
typedef struct __attribute__((packed)) {
    uint8_t header;             // 0xAA
    uint8_t type;               // PKT_TYPE_DATA
    uint16_t sequence_num;      // Current packet number
    uint16_t total_packets;     // Total packets in transfer
    uint8_t payload_size;       // Size of payload in this packet
    // Payload follows (up to 180 bytes)
    // CRC at the end (2 bytes)
} data_packet_header_t;

// Science Data Payload - First packet contains sensor data
typedef struct __attribute__((packed)) {
    // Timestamp
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    // IMU Data
    float accel_x;      // g
    float accel_y;
    float accel_z;
    float gyro_x;       // deg/s
    float gyro_y;
    float gyro_z;
    // Magnetometer
    int16_t mag_raw_x;
    int16_t mag_raw_y;
    int16_t mag_raw_z;
    float heading;      // degrees
    // Temperature/Humidity
    float temperature;  // Celsius
    float humidity;     // %
    // Camera info
    uint8_t has_image;  // 0 = NO, 1 = YES
    uint32_t image_size;// Total image size in bytes
    // Image data block follows (remaining bytes)
} science_data_first_t;

// ACK Packet
typedef struct __attribute__((packed)) {
    uint8_t header;         // 0xAA
    uint8_t type;           // PKT_TYPE_ACK
    uint16_t acked_seq;     // Sequence number being acknowledged
    uint16_t crc;
} ack_packet_t;

// END Packet
typedef struct __attribute__((packed)) {
    uint8_t header;         // 0xAA
    uint8_t type;           // PKT_TYPE_END
    uint16_t total_sent;    // Total packets sent
    uint16_t crc;
} end_packet_t;

// PING/PONG Packet
typedef struct __attribute__((packed)) {
    uint8_t header;         // 0xAA
    uint8_t type;           // PKT_TYPE_PING or PKT_TYPE_PONG
    uint32_t timestamp;     // For round-trip time calculation
    uint16_t crc;
} ping_packet_t;

// Command Packet (from GCS)
typedef struct __attribute__((packed)) {
    uint8_t header;         // 0xAA
    uint8_t type;           // Command type
    uint8_t payload[16];    // Optional command parameters
    uint16_t crc;
} command_packet_t;

// ========== CRC FUNCTIONS ==========
uint16_t Packet_Calculate_CRC(const uint8_t* data, uint16_t length);
bool Packet_Verify_CRC(const uint8_t* packet, uint16_t length);

// ========== PACKET BUILD FUNCTIONS ==========
uint16_t Packet_Build_Beacon(uint8_t* buffer, uint8_t sat_id, uint32_t uptime, 
                              uint8_t battery, uint8_t state, uint8_t sensors_ok, 
                              uint32_t flash_free);
uint16_t Packet_Build_Status(uint8_t* buffer, const status_packet_t* status);
uint16_t Packet_Build_Pong(uint8_t* buffer, uint32_t timestamp);
uint16_t Packet_Build_ACK(uint8_t* buffer, uint16_t seq_num);
uint16_t Packet_Build_End(uint8_t* buffer, uint16_t total_sent);
uint16_t Packet_Build_DataPacket(uint8_t* buffer, uint16_t seq, uint16_t total, 
                                  const uint8_t* payload, uint8_t payload_size);

// ========== PACKET PARSE FUNCTIONS ==========
packet_type_t Packet_Parse_Type(const uint8_t* packet);
bool Packet_Parse_Command(const uint8_t* packet, uint16_t length, command_packet_t* cmd);
bool Packet_Parse_ACK(const uint8_t* packet, uint16_t length, uint16_t* acked_seq);

#endif // PACKET_PROTOCOL_H
