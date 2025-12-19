/* main.c – Firmware LoRa CubeSat (Clean Version) */

#include "main.h"
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "protocol.h"
#include "crc16.h"

#include <string.h>
#include <stdio.h>


/* RX buffer */
static uint8_t rx_buffer[RX_BUF_SIZE];
static uint16_t rx_index = 0;

uint32_t last_rx_tick = 0;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* Interrupt-driven receive */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        rx_index++;
        last_rx_tick = HAL_GetTick();

        /* prepare next byte */
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    /* Start UART reception (1 byte interrupt mode) */
    rx_index = 0;
    HAL_UART_Receive_IT(&huart1, &rx_buffer[0], 1);

    uint32_t last_periodic = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* ----------------------------------------------------
         * CHECK FRAME TIMEOUT
         * ---------------------------------------------------- */
        if (rx_index > 0 && (now - last_rx_tick) > FRAME_TIMEOUT_MS)
        {
            /* A complete frame may be available */
            uint16_t frame_len = rx_index;

            if (frame_len >= MIN_FRAME_LEN)
            {
                protocol_process_frame(rx_buffer, frame_len);
            }

            /* Reset buffer */
            rx_index = 0;
        }

        /* ----------------------------------------------------
         * PERIODIC TELEMETRY
         * ---------------------------------------------------- */
        if (now - last_periodic >= 1000)    // mỗi 1 giây
        {
            last_periodic = now;
            protocol_periodic_tasks();
        }
    }
}

/* ============================================================
 * CLOCK CONFIG (CubeMX default for STM32F103C8)
 * ============================================================ */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1 |
                                       RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}


static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Configure LoRa pins:
       M0 = PB0
       M1 = PB1
       AUX = PA8
       Set M0/M1 = LOW (Normal mode)
    */

    /* M0 - PB0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

    /* M1 - PB1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

    /* AUX (input) - PA8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* ============================================================
 * ERROR HANDLER
 * ============================================================ */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        // Optionally blink LED or debug
    }
}

