#include "OV2640.h"
#include "string.h"

/* =========================
 * Configuration Tables
 * ========================= */

// JPEG Initialization Configuration
const uint8_t OV2640_JPEG_INIT[][2] = {
    { 0xff, 0x00 }, { 0x2c, 0xff }, { 0x2e, 0xdf }, { 0xff, 0x01 },
    { 0x3c, 0x32 }, { 0x11, 0x00 }, { 0x09, 0x02 }, { 0x04, 0x28 },
    { 0x13, 0xe5 }, { 0x14, 0x48 }, { 0x2c, 0x0c }, { 0x33, 0x78 },
    { 0x3a, 0x33 }, { 0x3b, 0xfB }, { 0x3e, 0x00 }, { 0x43, 0x11 },
    { 0x16, 0x10 }, { 0x39, 0x92 }, { 0x35, 0xda }, { 0x22, 0x1a },
    { 0x37, 0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 }, { 0x36, 0x1a },
    { 0x06, 0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 }, { 0x0e, 0x41 },
    { 0x4c, 0x00 }, { 0x48, 0x00 }, { 0x5B, 0x00 }, { 0x42, 0x03 },
    { 0x4a, 0x81 }, { 0x21, 0x99 }, { 0x24, 0x40 }, { 0x25, 0x38 },
    { 0x26, 0x82 }, { 0x5c, 0x00 }, { 0x63, 0x00 }, { 0x61, 0x70 },
    { 0x62, 0x80 }, { 0x7c, 0x05 }, { 0x20, 0x80 }, { 0x28, 0x30 },
    { 0x6c, 0x00 }, { 0x6d, 0x80 }, { 0x6e, 0x00 }, { 0x70, 0x02 },
    { 0x71, 0x94 }, { 0x73, 0xc1 }, { 0x12, 0x40 }, { 0x17, 0x11 },
    { 0x18, 0x43 }, { 0x19, 0x00 }, { 0x1a, 0x4b }, { 0x32, 0x09 },
    { 0x37, 0xc0 }, { 0x4f, 0x60 }, { 0x50, 0xa8 }, { 0x6d, 0x00 },
    { 0x3d, 0x38 }, { 0x46, 0x3f }, { 0x4f, 0x60 }, { 0x0c, 0x3c },
    { 0xff, 0x00 }, { 0xe5, 0x7f }, { 0xf9, 0xc0 }, { 0x41, 0x24 },
    { 0xe0, 0x14 }, { 0x76, 0xff }, { 0x33, 0xa0 }, { 0x42, 0x20 },
    { 0x43, 0x18 }, { 0x4c, 0x00 }, { 0x87, 0xd5 }, { 0x88, 0x3f },
    { 0xd7, 0x03 }, { 0xd9, 0x10 }, { 0xd3, 0x82 }, { 0xc8, 0x08 },
    { 0xc9, 0x80 }, { 0x7c, 0x00 }, { 0x7d, 0x00 }, { 0x7c, 0x03 },
    { 0x7d, 0x48 }, { 0x7d, 0x48 }, { 0x7c, 0x08 }, { 0x7d, 0x20 },
    { 0x7d, 0x10 }, { 0x7d, 0x0e }, { 0x90, 0x00 }, { 0x91, 0x0e },
    { 0x91, 0x1a }, { 0x91, 0x31 }, { 0x91, 0x5a }, { 0x91, 0x69 },
    { 0x91, 0x75 }, { 0x91, 0x7e }, { 0x91, 0x88 }, { 0x91, 0x8f },
    { 0x91, 0x96 }, { 0x91, 0xa3 }, { 0x91, 0xaf }, { 0x91, 0xc4 },
    { 0x91, 0xd7 }, { 0x91, 0xe8 }, { 0x91, 0x20 }, { 0x92, 0x00 },
    { 0x93, 0x06 }, { 0x93, 0xe3 }, { 0x93, 0x05 }, { 0x93, 0x05 },
    { 0x93, 0x00 }, { 0x93, 0x04 }, { 0x93, 0x00 }, { 0x93, 0x00 },
    { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 }, { 0x93, 0x00 },
    { 0x93, 0x00 }, { 0x96, 0x00 }, { 0x97, 0x08 }, { 0x97, 0x19 },
    { 0x97, 0x02 }, { 0x97, 0x0c }, { 0x97, 0x24 }, { 0x97, 0x30 },
    { 0x97, 0x28 }, { 0x97, 0x26 }, { 0x97, 0x02 }, { 0x97, 0x98 },
    { 0x97, 0x80 }, { 0x97, 0x00 }, { 0x97, 0x00 }, { 0xc3, 0xed },
    { 0xa4, 0x00 }, { 0xa8, 0x00 }, { 0xc5, 0x11 }, { 0xc6, 0x51 },
    { 0xbf, 0x80 }, { 0xc7, 0x10 }, { 0xb6, 0x66 }, { 0xb8, 0xA5 },
    { 0xb7, 0x64 }, { 0xb9, 0x7C }, { 0xb3, 0xaf }, { 0xb4, 0x97 },
    { 0xb5, 0xFF }, { 0xb0, 0xC5 }, { 0xb1, 0x94 }, { 0xb2, 0x0f },
    { 0xc4, 0x5c }, { 0xc0, 0x64 }, { 0xc1, 0x4B }, { 0x8c, 0x00 },
    { 0x86, 0x3D }, { 0x50, 0x00 }, { 0x51, 0xC8 }, { 0x52, 0x96 },
    { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x5a, 0xC8 },
    { 0x5b, 0x96 }, { 0x5c, 0x00 }, { 0xd3, 0x00 }, { 0xc3, 0xed },
    { 0x7f, 0x00 }, { 0xda, 0x00 }, { 0xe5, 0x1f }, { 0xe1, 0x67 },
    { 0xe0, 0x00 }, { 0xdd, 0x7f }, { 0x05, 0x00 }, { 0x12, 0x40 },
    { 0xd3, 0x04 }, { 0xc0, 0x16 }, { 0xC1, 0x12 }, { 0x8c, 0x00 },
    { 0x86, 0x3d }, { 0x50, 0x00 }, { 0x51, 0x2C }, { 0x52, 0x24 },
    { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x5A, 0x2c },
    { 0x5b, 0x24 }, { 0x5c, 0x00 },
    { 0xff, 0xff }  // End marker
};

// YUV422 Format Configuration
const uint8_t OV2640_YUV422[][2] = {
    { 0xFF, 0x00 }, { 0x05, 0x00 }, { 0xDA, 0x10 }, { 0xD7, 0x03 },
    { 0xDF, 0x00 }, { 0x33, 0x80 }, { 0x3C, 0x40 }, { 0xe1, 0x77 },
    { 0x00, 0x00 },
    { 0xff, 0xff }  // End marker
};

// JPEG Format Configuration
const uint8_t OV2640_JPEG[][2] = {
    { 0xe0, 0x14 }, { 0xe1, 0x77 }, { 0xe5, 0x1f }, { 0xd7, 0x03 },
    { 0xda, 0x10 }, { 0xe0, 0x00 }, { 0xFF, 0x01 }, { 0x04, 0x08 },
    { 0xff, 0xff }  // End marker
};

// 160x120 JPEG Resolution Configuration
const uint8_t OV2640_160x120_JPEG[][2] = {
    { 0xFF, 0x01 }, { 0x12, 0x40 }, { 0x17, 0x11 }, { 0x18, 0x43 },
    { 0x19, 0x00 }, { 0x1a, 0x4b }, { 0x32, 0x09 }, { 0x4f, 0xca },
    { 0x50, 0xa8 }, { 0x5a, 0x23 }, { 0x6d, 0x00 }, { 0x39, 0x12 },
    { 0x35, 0xda }, { 0x22, 0x1a }, { 0x37, 0xc3 }, { 0x23, 0x00 },
    { 0x34, 0xc0 }, { 0x36, 0x1a }, { 0x06, 0x88 }, { 0x07, 0xc0 },
    { 0x0d, 0x87 }, { 0x0e, 0x41 }, { 0x4c, 0x00 }, { 0xFF, 0x00 },
    { 0xe0, 0x04 }, { 0xc0, 0x64 }, { 0xc1, 0x4b }, { 0x86, 0x35 },
    { 0x50, 0x92 }, { 0x51, 0xc8 }, { 0x52, 0x96 }, { 0x53, 0x00 },
    { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x57, 0x00 }, { 0x5a, 0x2c },
    { 0x5b, 0x24 }, { 0x5c, 0x00 }, { 0xe0, 0x00 },
    { 0xff, 0xff }  // End marker
};
const uint8_t OV2640_320x240_JPEG[][2] = { { 0xff, 0x01 }, { 0x12, 0x40 },
		{ 0x17, 0x11 }, { 0x18, 0x43 }, { 0x19, 0x00 }, { 0x1a, 0x4b }, { 0x32,
				0x09 }, { 0x4f, 0xca }, { 0x50, 0xa8 }, { 0x5a, 0x23 }, { 0x6d,
				0x00 }, { 0x39, 0x12 }, { 0x35, 0xda }, { 0x22, 0x1a }, { 0x37,
				0xc3 }, { 0x23, 0x00 }, { 0x34, 0xc0 }, { 0x36, 0x1a }, { 0x06,
				0x88 }, { 0x07, 0xc0 }, { 0x0d, 0x87 }, { 0x0e, 0x41 }, { 0x4c,
				0x00 }, { 0xff, 0x00 }, { 0xe0, 0x04 }, { 0xc0, 0x64 }, { 0xc1,
				0x4b }, { 0x86, 0x35 }, { 0x50, 0x89 }, { 0x51, 0xc8 }, { 0x52,
				0x96 }, { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x00 }, { 0x57,
				0x00 }, { 0x5a, 0x50 }, { 0x5b, 0x3c }, { 0x5c, 0x00 }, { 0xe0,
				0x00 }, { 0xff, 0xff }, };
const uint8_t OV2640_800x600_JPEG[][2] = { { 0xFF, 0x01 }, { 0x11, 0x01 },
		{ 0x12, 0x00 }, { 0x17, 0x11 }, { 0x18, 0x75 }, { 0x32, 0x36 }, { 0x19,
				0x01 }, { 0x1a, 0x97 }, { 0x03, 0x0f }, { 0x37, 0x40 }, { 0x4f,
				0xbb }, { 0x50, 0x9c }, { 0x5a, 0x57 }, { 0x6d, 0x80 }, { 0x3d,
				0x34 }, { 0x39, 0x02 }, { 0x35, 0x88 }, { 0x22, 0x0a }, { 0x37,
				0x40 }, { 0x34, 0xa0 }, { 0x06, 0x02 }, { 0x0d, 0xb7 }, { 0x0e,
				0x01 }, { 0xFF, 0x00 }, { 0xe0, 0x04 }, { 0xc0, 0xc8 }, { 0xc1,
				0x96 }, { 0x86, 0x35 }, { 0x50, 0x89 }, { 0x51, 0x90 }, { 0x52,
				0x2c }, { 0x53, 0x00 }, { 0x54, 0x00 }, { 0x55, 0x88 }, { 0x57,
				0x00 }, { 0x5a, 0xc8 }, { 0x5b, 0x96 }, { 0x5c, 0x00 }, { 0xd3,
				0x02 }, { 0xe0, 0x00 }, { 0xff, 0xff } };
/* =========================
 * Private Variables
 * ========================= */
I2C_HandleTypeDef *phi2c ;
DCMI_HandleTypeDef *phdcmi; 
/* =========================
 * SCCB (I2C) Functions Implementation
 * ========================= */
uint32_t jpeg_size = 0;
/**
 * @brief Write data to OV2640 register via SCCB (I2C)
 * @param reg_addr: Register address to write to
 * @param data: Data byte to write
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t SCCB_Write(uint8_t reg_addr, uint8_t data) {
    if (phi2c == NULL) {
        return OV2640_ERROR;
    }

    uint8_t buffer[2] = { reg_addr, data };
    HAL_StatusTypeDef status;

    __disable_irq();
    status = HAL_I2C_Master_Transmit(phi2c, OV2640_I2C_ADDR, buffer, 2, 100);
    __enable_irq();

    if (status != HAL_OK) {
        return OV2640_I2C_ERROR;
    }

    return OV2640_OK;
}

/**
 * @brief Read data from OV2640 register via SCCB (I2C)
 * @param reg_addr: Register address to read from
 * @param pdata: Pointer to store read data
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t SCCB_Read(uint8_t reg_addr, uint8_t *pdata) {
    if (phi2c == NULL || pdata == NULL) {
        return OV2640_INVALID_PARAM;
    }

    HAL_StatusTypeDef status;

    __disable_irq();
    
    // Send register address
    status = HAL_I2C_Master_Transmit(phi2c, OV2640_I2C_ADDR, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        __enable_irq();
        return OV2640_I2C_ERROR;
    }

    // Read data from register
    status = HAL_I2C_Master_Receive(phi2c, (uint16_t) 0x61, pdata, 1, 100);
    __enable_irq();

    if (status != HAL_OK) {
        return OV2640_I2C_ERROR;
    }

    return OV2640_OK;
}

/* =========================
 * Configuration Functions Implementation
 * ========================= */

/**
 * @brief Write configuration array to OV2640
 * @param arr: Configuration array (register-value pairs)
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_Configuration(const uint8_t arr[][2]) {
    if (arr == NULL) {
        return OV2640_INVALID_PARAM;
    }

    uint16_t i = 0;
    uint8_t reg_addr, data;
    ov2640_status_t status;

    while (1) {
        reg_addr = arr[i][0];
        data = arr[i][1];

        // Check for end marker
        if (reg_addr == 0xff && data == 0xff) {
            break;
        }

        // Write register
        status = SCCB_Write(reg_addr, data);
        if (status != OV2640_OK) {
            return OV2640_CONFIG_ERROR;
        }

        // Small delay between writes
        HAL_Delay(1);
        i++;
    }

    return OV2640_OK;
}

/**
 * @brief Set image format (JPEG or YUV422)
 * @param format: Image format to set
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_SetFormat(image_format_t format) {
    ov2640_status_t status;

    switch (format) {
        case FORMAT_JPEG:
            status = OV2640_Configuration(OV2640_JPEG);
            break;
        case FORMAT_YUV422:
            status = OV2640_Configuration(OV2640_YUV422);
            break;
        default:
            return OV2640_INVALID_PARAM;
    }

    return status;
}

/**
 * @brief Set image resolution (only 160x120 supported)
 * @param resolution: Resolution to set
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_SetResolution(image_resolution_t resolution) {
    //if (resolution != RES_160x120) {
    //    return OV2640_INVALID_PARAM;
    //}
	switch(resolution) {
		case RES_160x120:
			return OV2640_Configuration(OV2640_160x120_JPEG);
		case RES_320x240:
			return OV2640_Configuration(OV2640_320x240_JPEG);
		case RES_800x600:
			return OV2640_Configuration(OV2640_800x600_JPEG);
	}
		
    return OV2640_Configuration(OV2640_160x120_JPEG);
}

/* =========================
 * Main Functions Implementation
 * ========================= */

/**
 * @brief Initialize OV2640 camera
 * @param p_hi2c: Pointer to I2C handle
 * @param p_hdcmi: Pointer to DCMI handle
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_Init(I2C_HandleTypeDef *p_hi2c, DCMI_HandleTypeDef *p_hdcmi) {
    if (p_hi2c == NULL || p_hdcmi == NULL) {
        return OV2640_INVALID_PARAM;
    }

    phi2c = p_hi2c;
    phdcmi = p_hdcmi;

    // Hardware reset (assumes CAMERA_RESET_GPIO_Port and CAMERA_RESET_Pin are defined)

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(50);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_13, GPIO_PIN_RESET);


    // Software reset: reset all registers to default values
    ov2640_status_t status;
    status = SCCB_Write(0xff, 0x01);
    if (status != OV2640_OK) {
        return OV2640_ERROR;
        //return status;
    }

    status = SCCB_Write(0x12, 0x80);
    if (status != OV2640_OK) {
        return status;
    }

    HAL_Delay(10);

    // Check device ID
    uint8_t pid, ver;
    if (!OV2640_ReadChipID(&pid, &ver)) {
        return OV2640_DEVICE_NOT_FOUND;
    }

    // Verify chip ID
    if (pid != OV2640_PID_VALUE || ver != OV2640_VER_VALUE) {
        return OV2640_DEVICE_NOT_FOUND;
    }

    // Stop DCMI and clear buffer
    status = OV2640_StopDCMI();
    if (status != OV2640_OK) {
        return status;
    }

    // Load JPEG initialization configuration
    status = OV2640_Configuration(OV2640_JPEG_INIT);
    if (status != OV2640_OK) {
        return OV2640_INIT_FAILED;
    }

    // Set JPEG format
    status = OV2640_SetFormat(FORMAT_JPEG);
    if (status != OV2640_OK) {
        return OV2640_INIT_FAILED;
    }

    // Set 160x120 resolution
    status = OV2640_SetResolution(RES_320x240);
    if (status != OV2640_OK) {
        return OV2640_INIT_FAILED;
    }

    return OV2640_OK;
}

/**
 * @brief Stop DCMI interface
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_StopDCMI() {
    if (phdcmi == NULL) {
        return OV2640_ERROR;
    }

    HAL_DCMI_Stop(phdcmi);
    return OV2640_OK;
}

/**
 * @brief Start snapshot capture
 * @param frameBuffer: Address of frame buffer
 * @param length: Length of data to capture
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_StartSnapshot(uint32_t frameBuffer, uint32_t length) {
    if (phdcmi == NULL || frameBuffer == 0 || length == 0) {
        return OV2640_INVALID_PARAM;
    }

    HAL_StatusTypeDef status;
    status = HAL_DCMI_Start_DMA(phdcmi, DCMI_MODE_SNAPSHOT, 
                                frameBuffer, length);

    if (status != HAL_OK) {
        return OV2640_ERROR;
    }

    return OV2640_OK;
}
ov2640_status_t OV2640_TakeSnapshot(uint8_t *frameBuffer, uint32_t length)
{
		if(phdcmi == NULL) {
			return OV2640_ERROR;
		}
    OV2640_SetFormat(FORMAT_JPEG);  
    OV2640_SetResolution(RES_160x120); 
    uint32_t words = length / 4;
    if (HAL_DCMI_Start_DMA(phdcmi,
                           DCMI_MODE_SNAPSHOT,
                           (uint32_t)frameBuffer,
                           words) != HAL_OK)
    {
        return OV2640_I2C_ERROR;
    }
		HAL_Delay(1000);
    HAL_DCMI_Stop(phdcmi);

    for (uint32_t i = 0; i < length - 1; i++) {
        if (frameBuffer[i] == 0xFF && frameBuffer[i+1] == 0xD9) {
            jpeg_size = i + 2;
            break;
        }
    }
    return OV2640_CAPTURE_OK;
}

/* =========================
 * Image Adjustment Functions (Stub implementations)
 * ========================= */

ov2640_status_t OV2640_Contrast(int8_t contrast) {
    // TODO: Implement contrast adjustment
    // Range: -2 to +2
    if (contrast < -2 || contrast > 2) {
        return OV2640_INVALID_PARAM;
    }
    return OV2640_OK;
}

ov2640_status_t OV2640_Saturation(int8_t saturation) {
    // TODO: Implement saturation adjustment
    // Range: -2 to +2
    if (saturation < -2 || saturation > 2) {
        return OV2640_INVALID_PARAM;
    }
    return OV2640_OK;
}

ov2640_status_t OV2640_Brightness(int8_t brightness) {
    // TODO: Implement brightness adjustment
    // Range: -2 to +2
    if (brightness < -2 || brightness > 2) {
        return OV2640_INVALID_PARAM;
    }
    return OV2640_OK;
}

ov2640_status_t OV2640_LightMode(uint8_t lightMode) {
    // TODO: Implement light mode adjustment
    return OV2640_OK;
}

/* =========================
 * Debug/Status Functions Implementation
 * ========================= */

/**
 * @brief Get status string from status code
 * @param status: Status code
 * @return const char*: Status string
 */
const char* OV2640_GetStatusString(ov2640_status_t status) {
    switch (status) {
        case OV2640_OK:
            return "OV2640_OK";
        case OV2640_ERROR:
            return "OV2640_ERROR";
        case OV2640_INVALID_PARAM:
            return "OV2640_INVALID_PARAM";
        case OV2640_TIMEOUT:
            return "OV2640_TIMEOUT";
        case OV2640_DEVICE_NOT_FOUND:
            return "OV2640_DEVICE_NOT_FOUND";
        case OV2640_INIT_FAILED:
            return "OV2640_INIT_FAILED";
        case OV2640_CONFIG_ERROR:
            return "OV2640_CONFIG_ERROR";
        case OV2640_I2C_OK:
            return "OV2640_I2C_OK";           
        case OV2640_I2C_TIMEOUT:
            return "OV2640_I2C_TIMEOUT";  // KhÃ´ng tÃ¬m tháº¥y
        case OV2640_I2C_BUSY:
            return "OV2640_I2C_BUSY";    // Bus báº­n
        case OV2640_I2C_ERROR:
            return "OV2640_I2C_ERROR";   // Lá»—i chung
        case OV2640_CAPTURE_OK:
            return "OV2640_CAPTURE_OK";
    }
    return "OV2640_OK";
}

/**
 * @brief Check if OV2640 device is present and responding
 * @return ov2640_status_t: Operation status
 */
ov2640_status_t OV2640_CheckDevice(I2C_HandleTypeDef *p_hi2c) {
    phi2c = p_hi2c;

    if (phi2c == NULL) {
        return OV2640_ERROR;
    }

    HAL_StatusTypeDef status;
    status = HAL_I2C_IsDeviceReady(phi2c, OV2640_I2C_ADDR, 3, 100);

    switch (status) {
        case HAL_OK:
            return OV2640_I2C_OK;           // Device sáºµn sÃ ng
        case HAL_TIMEOUT:
            return OV2640_I2C_TIMEOUT;  // KhÃ´ng tÃ¬m tháº¥y
        case HAL_BUSY:
            return OV2640_I2C_BUSY;    // Bus báº­n
        case HAL_ERROR:
            return OV2640_I2C_ERROR;    // Lá»—i khÃ¡c
    }
    return OV2640_OK;
}

/**
 * @brief Read chip ID (PID and VER)
 * @param pid: Pointer to store Product ID
 * @param ver: Pointer to store Version
 * @return bool: true if successful, false otherwise
 */
bool OV2640_ReadChipID(uint8_t *pid, uint8_t *ver) {
    if (pid == NULL || ver == NULL) {
        return false;
    }

    ov2640_status_t status;

    // Select sensor register bank
    status = SCCB_Write(0xff, 0x01);
    if (status != OV2640_OK) {
        return false;
    }

    // Read PID (Product ID High Byte) - should be 0x26
    status = SCCB_Read(OV2640_REG_PID, pid);
    if (status != OV2640_OK) {
        return false;
    }

    // Read VER (Product ID Low Byte) - should be 0x42
    status = SCCB_Read(OV2640_REG_VER, ver);
    if (status != OV2640_OK) {
        return false;
    }

    return true;
}