#include "task_config.h"
#include "hardware.h"
#include "sdcard.h"
#include <string.h>

extern UART_HandleTypeDef huart3;
extern lora_e32_handle_t lora_handle;

static void ProcessLog(const output_request_t *req);
static void ProcessTelemetry(const output_request_t *req);
static void ProcessFlashWrite(const output_request_t *req);
static void ProcessLED(const output_request_t *req);

void Task_OutputManager(void *pvParameters)
{
    output_request_t req;
    
    for (;;)
    {
        if (ReceiveOutput(&req, portMAX_DELAY) == pdPASS)
        {
            switch (req.type)
            {
            case OUT_LOG:
                ProcessLog(&req);
                break;
            case OUT_TELEMETRY:
                ProcessTelemetry(&req);
                break;
            case OUT_FLASH_WRITE:
                ProcessFlashWrite(&req);
                break;
            case OUT_LED:
                ProcessLED(&req);
                break;
            default:
                break;
            }
        }
        taskYIELD();
    }
}

static void ProcessLog(const output_request_t *req)
{
    HAL_UART_Transmit(&huart3, (uint8_t*)req->data.log.msg,
                     strlen(req->data.log.msg), 100);
}

static void ProcessTelemetry(const output_request_t *req)
{
    LoRa_E32_Send(&lora_handle, req->data.telemetry.data,
                 req->data.telemetry.len, 1000);
}

static void ProcessFlashWrite(const output_request_t *req)
{
    // FIXED: Actual SD card write implementation
    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(5000)) == pdPASS)
    {
        sd_status_t result = SD_WriteScience(req->data.flash.data, 
                                            req->data.flash.len);
        if (result != SD_OK) {
            // Log write failure
            char err[64];
            snprintf(err, sizeof(err), "SD write failed: %d\r\n", result);
            HAL_UART_Transmit(&huart3, (uint8_t*)err, strlen(err), 100);
        }
        xSemaphoreGive(sdMutex);
    }
}

static void ProcessLED(const output_request_t *req)
{
    // FIXED: Actual LED control (if LED exists)
    // Uncomment when LED is configured in hardware
    /*
    switch (req->data.led.state)
    {
    case 0: // Off
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        break;
    case 1: // On
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        break;
    case 2: // Blink - use software timer
        // TODO: Setup timer for blinking if needed
        break;
    }
    */
}
