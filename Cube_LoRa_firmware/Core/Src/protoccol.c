/*
 * protocol.c
 * Fixed version – fully consistent with protocol.h
 */

#include "protocol.h"
#include "crc16.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

/* UART TX wrapper */
static HAL_StatusTypeDef uart_tx(const uint8_t *pData, uint16_t size)
{
    return HAL_UART_Transmit(&huart1, (uint8_t*)pData, size, 200);
}

/* ============================================================================
 *  BUILD & SEND FRAME
 * ============================================================================ */
void protocol_send_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (len > MAX_PAYLOAD_LEN) len = MAX_PAYLOAD_LEN;

    uint8_t tx[TX_BUF_SIZE];
    uint16_t idx = 0;

    tx[idx++] = FRAME_HEADER_1;
    tx[idx++] = FRAME_HEADER_2;

    tx[idx++] = type;
    tx[idx++] = SAT_SRC_ID;
    tx[idx++] = GND_DST_ID;
    tx[idx++] = len;

    if (payload != NULL && len > 0)
    {
        memcpy(&tx[idx], payload, len);
        idx += len;
    }

    uint16_t crc = crc16_compute(tx, idx);
    tx[idx++] = (uint8_t)(crc & 0xFF);
    tx[idx++] = (uint8_t)(crc >> 8);

    uart_tx(tx, idx);
}

/* ============================================================================
 *  COMMAND HANDLER
 * ============================================================================ */
void handle_command(const uint8_t *payload, uint8_t len)
{
    char cmd[64] = {0};
    uint8_t cpy = (len < 63 ? len : 63);
    memcpy(cmd, payload, cpy);

    const char ack_ping[]    = "ACK_PING";
    const char ack_sci[]     = "ACK_SCIENCE";
    const char ack_cfg[]     = "ACK_CONFIG";
    const char ack_reset[]   = "ACK_RESET";
    const char ack_unknown[] = "ACK_UNKNOWN";

    if (strcmp(cmd, "PING") == 0) {
        protocol_send_frame(PKT_ACK, (uint8_t*)ack_ping, sizeof(ack_ping)-1);

    } else if (strcmp(cmd, "SCIENCE") == 0) {
        protocol_send_frame(PKT_ACK, (uint8_t*)ack_sci, sizeof(ack_sci)-1);

    } else if (strncmp(cmd, "CONFIG", 6) == 0) {
        protocol_send_frame(PKT_ACK, (uint8_t*)ack_cfg, sizeof(ack_cfg)-1);

    } else if (strcmp(cmd, "RESET") == 0) {
        protocol_send_frame(PKT_ACK, (uint8_t*)ack_reset, sizeof(ack_reset)-1);
        HAL_Delay(20);
        NVIC_SystemReset();

    } else {
        protocol_send_frame(PKT_ACK, (uint8_t*)ack_unknown, sizeof(ack_unknown)-1);
    }
}

/* ============================================================================
 *  FRAME PARSER
 * ============================================================================ */
void protocol_process_frame(uint8_t *frame, uint16_t size)
{
    if (!frame || size < MIN_FRAME_LEN) return;

    if (frame[0] != FRAME_HEADER_1 || frame[1] != FRAME_HEADER_2)
        return;

    uint8_t type = frame[2];
    uint8_t len  = frame[5];

    if (size < (6 + len + 2)) return;

    uint8_t *payload = &frame[6];

    uint16_t recv_crc = (uint16_t)frame[6 + len] |
                        ((uint16_t)frame[6 + len + 1] << 8);

    uint16_t calc_crc = crc16_compute(frame, 6 + len);

    if (recv_crc != calc_crc)
    {
        const char nack[] = "NACK_CRC";
        protocol_send_frame(PKT_ACK, (uint8_t*)nack, sizeof(nack)-1);
        return;
    }

    switch (type)
    {
        case PKT_COMMAND:
            handle_command(payload, len);
            break;

        case PKT_TELEMETRY:
            break;

        case PKT_ACK:
            break;

        case PKT_DATA:
            break;

        case PKT_SCIENCE:
            break;

        default:
            break;
    }
}

/* ============================================================================
 *  PERIODIC TELEMETRY
 * ============================================================================ */
void protocol_periodic_tasks(void)
{
    static uint32_t t = 0;
    t++;

    if (t % 2000 == 0)
    {
        char tlm[64];
        int temp = 25;
        int batt = 88;

        int n = snprintf(tlm, sizeof(tlm),
                         "TEMP:%d,BATT:%d,MODE:OP",
                         temp, batt);

        protocol_send_frame(PKT_TELEMETRY, (uint8_t*)tlm, n);
    }
}
