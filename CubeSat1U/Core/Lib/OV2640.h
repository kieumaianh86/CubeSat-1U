#ifndef __OV2640_H__
#define __OV2640_H__

#include <stm32h7xx_hal.h>
#include <stm32h7xx_hal_dcmi.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * OV2640 I2C Address
 * ========================= */
#define OV2640_I2C_ADDR         0x60  // 7-bit address shifted (0x30 << 1)

/* =========================
 * OV2640 Registers
 * ========================= */
#define OV2640_REG_PID          0x0A  // Product ID High Byte (0x26)
#define OV2640_REG_VER          0x0B  // Product ID Low Byte (0x42)
#define OV2640_REG_MIDH         0x1C  // Manufacturer ID High Byte
#define OV2640_REG_MIDL         0x1D  // Manufacturer ID Low Byte

/* Expected chip ID values */
#define OV2640_PID_VALUE        0x26
#define OV2640_VER_VALUE        0x42

/* Réolution */
#define CAM_WIDTH   160
#define CAM_HEIGHT  120
#define BYTES_PER_PIXEL  2
//uint8_t framecam_buffer[CAM_WIDTH * CAM_HEIGHT * BYTES_PER_PIXEL];
  
/* =========================
 * Status codes
 * ========================= */
typedef enum {
    OV2640_OK = 0,
    OV2640_ERROR,
    OV2640_INVALID_PARAM,
    OV2640_TIMEOUT,
    OV2640_DEVICE_NOT_FOUND,
    OV2640_INIT_FAILED,
    OV2640_CONFIG_ERROR,
    OV2640_I2C_OK,
    OV2640_I2C_TIMEOUT,
    OV2640_I2C_BUSY,
    OV2640_I2C_ERROR,
		OV2640_CAPTURE_OK
} ov2640_status_t;

/* =========================
 * Image Format
 * ========================= */
typedef enum {
    FORMAT_YUV422 = 0,
    FORMAT_JPEG
} image_format_t;

/* =========================
 * Image Resolution (only 160x120 supported)
 * ========================= */
typedef enum {
    RES_160x120 = 0,
		RES_320x240,
    RES_800x600
} image_resolution_t;

/* =========================
 * SCCB (I2C) Functions
 * ========================= */
ov2640_status_t SCCB_Write(uint8_t reg_addr, uint8_t data);
ov2640_status_t SCCB_Read(uint8_t reg_addr, uint8_t *pdata);

/* =========================
 * Main OV2640 Functions
 * ========================= */
ov2640_status_t OV2640_Init(I2C_HandleTypeDef *p_hi2c, DCMI_HandleTypeDef *p_hdcmi);
ov2640_status_t OV2640_StartSnapshot(uint32_t frameBuffer, uint32_t length);
ov2640_status_t OV2640_TakeSnapshot(uint8_t *frameBuffer, uint32_t length);
ov2640_status_t OV2640_StopDCMI(void);

/* =========================
 * Configuration Functions
 * ========================= */
ov2640_status_t OV2640_Configuration(const uint8_t arr[][2]);
ov2640_status_t OV2640_SetFormat(image_format_t format);
ov2640_status_t OV2640_SetResolution(image_resolution_t resolution);

/* =========================
 * Image Adjustment Functions
 * ========================= */
ov2640_status_t OV2640_Contrast(int8_t contrast);        // Range: -2 to +2
ov2640_status_t OV2640_Saturation(int8_t saturation);    // Range: -2 to +2
ov2640_status_t OV2640_Brightness(int8_t brightness);    // Range: -2 to +2
ov2640_status_t OV2640_LightMode(uint8_t lightMode);

/* =========================
 * Debug/Status Functions
 * ========================= */
const char* OV2640_GetStatusString(ov2640_status_t status);
ov2640_status_t OV2640_CheckDevice(I2C_HandleTypeDef *p_hi2c);
bool OV2640_ReadChipID(uint8_t *pid, uint8_t *ver);



#ifdef __cplusplus
}
#endif

#endif /* __OV2640_H__ */
