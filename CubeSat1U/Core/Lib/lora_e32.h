#ifndef LORA_E32_H
#define LORA_E32_H

#include "stdint.h"
#include "stdbool.h"
#include "stm32h7xx_hal.h"

typedef enum {
    LORA_E32_OK = 0,
    LORA_E32_ERROR,
    LORA_E32_BUSY,
    LORA_E32_TIMEOUT,
    LORA_E32_INVALID_PARAM
} lora_e32_status_t;

typedef enum {
    LORA_E32_MODE_NORMAL = 0,
    LORA_E32_MODE_WAKEUP,
    LORA_E32_MODE_POWER_SAVE,
    LORA_E32_MODE_SLEEP
} lora_e32_mode_t;

typedef struct {
    UART_HandleTypeDef* huart;
    GPIO_TypeDef* m0_port;
    uint16_t m0_pin;
    GPIO_TypeDef* m1_port;
    uint16_t m1_pin;
    GPIO_TypeDef* aux_port;
    uint16_t aux_pin;
    uint32_t uart_timeout_ms;
} lora_e32_config_t;

typedef struct {
    UART_HandleTypeDef* uart;
    GPIO_TypeDef* m0_port;
    uint16_t m0_pin;
    GPIO_TypeDef* m1_port;
    uint16_t m1_pin;
    GPIO_TypeDef* aux_port;
    uint16_t aux_pin;
    uint8_t rx_buffer[256];
    uint16_t rx_index;
    uint16_t rx_write_index;
    uint16_t rx_read_index;
    uint8_t rx_temp;
    bool initialized;
    lora_e32_mode_t current_mode;
} lora_e32_handle_t;

// Init & Mode
lora_e32_status_t LoRa_E32_Init(lora_e32_handle_t* lora, const lora_e32_config_t* config);
lora_e32_status_t LoRa_E32_SetMode(lora_e32_handle_t* lora, lora_e32_mode_t mode);
bool LoRa_E32_IsReady(const lora_e32_handle_t* lora);

// TX/RX
lora_e32_status_t LoRa_E32_Send(lora_e32_handle_t* lora, const uint8_t* data, uint16_t len, uint32_t timeout_ms);
lora_e32_status_t LoRa_E32_Receive(lora_e32_handle_t* lora, uint8_t* data, uint16_t* len, uint32_t timeout_ms);
bool LoRa_E32_DataAvailable(const lora_e32_handle_t* lora);
void LoRa_E32_FlushRxBuffer(lora_e32_handle_t* lora);

// Power
lora_e32_status_t LoRa_E32_PowerOff(lora_e32_handle_t* lora);
lora_e32_status_t LoRa_E32_PowerOn(lora_e32_handle_t* lora);

// UART Interrupt
void LoRa_E32_UART_IRQHandler(lora_e32_handle_t* lora);

#endif // LORA_E32_H
