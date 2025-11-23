#include "lora_e32.h"
#include "string.h"

#define AUX_TIMEOUT_MS 1000
#define RX_BUFFER_SIZE 256

static void lora_set_mode_pins(lora_e32_handle_t* lora, uint8_t m0, uint8_t m1);
static bool lora_wait_aux_high(lora_e32_handle_t* lora, uint32_t timeout_ms);

lora_e32_status_t LoRa_E32_Init(lora_e32_handle_t* lora, const lora_e32_config_t* config)
{
    if (lora == NULL || config == NULL || config->huart == NULL)
    {
        return LORA_E32_INVALID_PARAM;
    }
    
    lora->uart = config->huart;
    lora->m0_port = config->m0_port;
    lora->m0_pin = config->m0_pin;
    lora->m1_port = config->m1_port;
    lora->m1_pin = config->m1_pin;
    lora->aux_port = config->aux_port;
    lora->aux_pin = config->aux_pin;
    lora->rx_index = 0;
    lora->rx_write_index = 0;
    lora->rx_read_index = 0;
    lora->initialized = false;
    lora->current_mode = LORA_E32_MODE_NORMAL;
    
    memset(lora->rx_buffer, 0, sizeof(lora->rx_buffer));
    
    // Set to normal mode (M0=0, M1=0)
    lora_set_mode_pins(lora, 0, 0);
    
    // Wait for AUX high (module ready)
    if (!lora_wait_aux_high(lora, AUX_TIMEOUT_MS))
    {
        return LORA_E32_TIMEOUT;
    }
    
    // Start UART interrupt receive
    HAL_UART_Receive_IT(lora->uart, &lora->rx_temp, 1);
    
    lora->initialized = true;
    return LORA_E32_OK;
}

lora_e32_status_t LoRa_E32_SetMode(lora_e32_handle_t* lora, lora_e32_mode_t mode)
{
    if (lora == NULL || !lora->initialized)
    {
        return LORA_E32_ERROR;
    }
    
    switch (mode)
    {
    case LORA_E32_MODE_NORMAL:      // M0=0, M1=0
        lora_set_mode_pins(lora, 0, 0);
        break;
    case LORA_E32_MODE_WAKEUP:      // M0=1, M1=0
        lora_set_mode_pins(lora, 1, 0);
        break;
    case LORA_E32_MODE_POWER_SAVE:  // M0=0, M1=1
        lora_set_mode_pins(lora, 0, 1);
        break;
    case LORA_E32_MODE_SLEEP:       // M0=1, M1=1
        lora_set_mode_pins(lora, 1, 1);
        break;
    default:
        return LORA_E32_INVALID_PARAM;
    }
    
    if (!lora_wait_aux_high(lora, AUX_TIMEOUT_MS))
    {
        return LORA_E32_TIMEOUT;
    }
    
    lora->current_mode = mode;
    return LORA_E32_OK;
}

bool LoRa_E32_IsReady(const lora_e32_handle_t* lora)
{
    if (lora == NULL || !lora->initialized)
    {
        return false;
    }
    
    return (HAL_GPIO_ReadPin(lora->aux_port, lora->aux_pin) == GPIO_PIN_SET);
}

lora_e32_status_t LoRa_E32_Send(lora_e32_handle_t* lora, const uint8_t* data, uint16_t len, uint32_t timeout_ms)
{
    if (lora == NULL || !lora->initialized || data == NULL || len == 0)
    {
        return LORA_E32_INVALID_PARAM;
    }
    
    if (!LoRa_E32_IsReady(lora))
    {
        return LORA_E32_BUSY;
    }
    
    if (HAL_UART_Transmit(lora->uart, (uint8_t*)data, len, timeout_ms) != HAL_OK)
    {
        return LORA_E32_ERROR;
    }
    
    // Wait for transmission complete
    if (!lora_wait_aux_high(lora, timeout_ms))
    {
        return LORA_E32_TIMEOUT;
    }
    
    return LORA_E32_OK;
}

lora_e32_status_t LoRa_E32_Receive(lora_e32_handle_t* lora, uint8_t* data, uint16_t* len, uint32_t timeout_ms)
{
    if (lora == NULL || !lora->initialized || data == NULL || len == NULL)
    {
        return LORA_E32_INVALID_PARAM;
    }
    
    uint32_t start = HAL_GetTick();
    uint16_t count = 0;
    uint16_t max_len = *len;
    
    // Read from circular buffer
    while ((HAL_GetTick() - start) < timeout_ms && count < max_len)
    {
        if (lora->rx_read_index != lora->rx_write_index)
        {
            data[count++] = lora->rx_buffer[lora->rx_read_index];
            lora->rx_read_index = (lora->rx_read_index + 1) % RX_BUFFER_SIZE;
            start = HAL_GetTick(); // Reset timeout on data received
        }
    }
    
    *len = count;
    
    if (count == 0)
    {
        return LORA_E32_TIMEOUT;
    }
    
    return LORA_E32_OK;
}

bool LoRa_E32_DataAvailable(const lora_e32_handle_t* lora)
{
    if (lora == NULL || !lora->initialized)
    {
        return false;
    }
    
    // Check if there's data in circular buffer
    return (lora->rx_read_index != lora->rx_write_index);
}

void LoRa_E32_FlushRxBuffer(lora_e32_handle_t* lora)
{
    if (lora == NULL)
    {
        return;
    }
    
    lora->rx_read_index = lora->rx_write_index;
}

lora_e32_status_t LoRa_E32_PowerOff(lora_e32_handle_t* lora)
{
    if (lora == NULL)
    {
        return LORA_E32_INVALID_PARAM;
    }
    
    // Set to sleep mode
    return LoRa_E32_SetMode(lora, LORA_E32_MODE_SLEEP);
}

lora_e32_status_t LoRa_E32_PowerOn(lora_e32_handle_t* lora)
{
    if (lora == NULL)
    {
        return LORA_E32_INVALID_PARAM;
    }
    
    // Set to normal mode
    return LoRa_E32_SetMode(lora, LORA_E32_MODE_NORMAL);
}

void LoRa_E32_UART_IRQHandler(lora_e32_handle_t* lora)
{
    if (lora == NULL || !lora->initialized)
    {
        return;
    }
    
    // Store received byte in circular buffer
    uint16_t next_index = (lora->rx_write_index + 1) % RX_BUFFER_SIZE;
    
    // Check for buffer overflow
    if (next_index != lora->rx_read_index)
    {
        lora->rx_buffer[lora->rx_write_index] = lora->rx_temp;
        lora->rx_write_index = next_index;
    }
    
    // Continue receiving
    HAL_UART_Receive_IT(lora->uart, &lora->rx_temp, 1);
}

static void lora_set_mode_pins(lora_e32_handle_t* lora, uint8_t m0, uint8_t m1)
{
    HAL_GPIO_WritePin(lora->m0_port, lora->m0_pin, m0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(lora->m1_port, lora->m1_pin, m1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool lora_wait_aux_high(lora_e32_handle_t* lora, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    
    while ((HAL_GetTick() - start) < timeout_ms)
    {
        if (HAL_GPIO_ReadPin(lora->aux_port, lora->aux_pin) == GPIO_PIN_SET)
        {
            return true;
        }
        HAL_Delay(1);
    }
    
    return false;
}
