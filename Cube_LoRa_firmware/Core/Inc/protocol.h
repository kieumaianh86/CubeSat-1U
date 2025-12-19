#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/* Frame structure: AA 55 | TYPE | SRC | DST | LEN | PAYLOAD | CRC16 */
#define FRAME_HEADER_1      0xAA
#define FRAME_HEADER_2      0x55

#define MIN_FRAME_LEN       8
#define MAX_PAYLOAD_LEN     64
#define TX_BUF_SIZE         80
#define RX_BUF_SIZE         80

// IDs (tùy bạn sửa)
#define SAT_SRC_ID          0x10
#define GND_DST_ID          0x01

// Packet types
#define PKT_TELEMETRY       0x01
#define PKT_COMMAND         0x02
#define PKT_ACK             0x03
#define PKT_DATA            0x04
#define PKT_SCIENCE         0x05

// timeout frame
#define FRAME_TIMEOUT_MS    1500

/* Function prototypes */
void protocol_send_frame(uint8_t type, const uint8_t *payload, uint8_t len);
void protocol_process_frame(uint8_t *frame, uint16_t size);
void handle_command(const uint8_t *cmd, uint8_t len);
void protocol_periodic_tasks(void);

#endif
