#ifndef __OV2640_H__
#define __OV2640_H__

#include <stm32h7xx_hal.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================
 * Status codes
 * ========================= */
typedef enum {
    OV2640_OK = 0,
    OV2640_ERROR,
    OV2640_I2C_ERROR,
    OV2640_DEVICE_NOT_FOUND,
    OV2640_TIMEOUT,
    OV2640_INVALID_PARAM,
    OV2640_NOT_READY
} ov2640_status_t;

/* =========================
 * I2C address (7-bit)
 * =========================
 * Mặc định OV2640 dùng SCCB 7-bit 0x30 (8-bit write=0x60/read=0x61).
 * Cho phép override qua macro compiler nếu cần.
 */
#ifndef OV2640_I2C_ADDR_7BIT
#define OV2640_I2C_ADDR_7BIT   0x30
#endif

/* =========================
 * Register pair for config tables
 * ========================= */
typedef struct {
    uint8_t reg;
    uint8_t val;
} ov2640_regval_t;

/* Ký hiệu kết thúc bảng thanh ghi */
#define OV2640_TBL_END   {0xFF, 0xFF}

/* =========================
 * Camera handle/context
 * ========================= */
typedef struct {
    I2C_HandleTypeDef  *hi2c;    /* I2C/SCCB handle */
    DCMI_HandleTypeDef *hdcmi;   /* DCMI handle */

    uint32_t width;              /* ảnh đầu ra (pixel) */
    uint32_t height;
    uint8_t  bpp;                /* bytes/pixel (RGB565 = 2, YUV422 = 2) */

    volatile uint8_t frame_ready;/* set trong frame callback */
} OV2640_Handle;

/* =========================
 * Public API
 * ========================= */

/* Khởi tạo handle (gán HAL handle, set mặc định W/H/BPP, ping thiết bị) */
ov2640_status_t OV2640_Init(OV2640_Handle *cam,
                            I2C_HandleTypeDef *hi2c,
                            DCMI_HandleTypeDef *hdcmi);

/* Kiểm tra thiết bị sẵn sàng trên bus */
ov2640_status_t OV2640_Ping(OV2640_Handle *cam);

/* Reset mềm cảm biến (0xFF=0x01; 0x12=0x80) */
ov2640_status_t OV2640_Reset(OV2640_Handle *cam);

/* Ghi/đọc 1 thanh ghi */
ov2640_status_t OV2640_WriteReg(OV2640_Handle *cam, uint8_t reg, uint8_t val);
ov2640_status_t OV2640_ReadReg (OV2640_Handle *cam, uint8_t reg, uint8_t *val);

/* Áp một bảng cấu hình (kết thúc bằng OV2640_TBL_END) */
ov2640_status_t OV2640_ApplyTable(OV2640_Handle *cam, const ov2640_regval_t *tbl);

/* Bắt một khung hình (snapshot) vào buffer:
   - buf_len_bytes phải >= width*height*bpp
   - Hàm sẽ gọi HAL_DCMI_Start_DMA với length tính theo WORD 32-bit */
ov2640_status_t OV2640_StartSnapshot(OV2640_Handle *cam, void *buf, uint32_t buf_len_bytes);

/* Đợi xong khung (frame_ready==1). timeout_ms=0 nghĩa là chờ vô hạn. */
ov2640_status_t OV2640_WaitFrame(OV2640_Handle *cam, uint32_t timeout_ms);

/* Gọi hàm này trong HAL_DCMI_FrameEventCallback() để set cờ frame_ready */
void OV2640_OnFrameEvent(OV2640_Handle *cam);

/* =========================
 * Preset tables (khai báo; định nghĩa ở .c)
 * =========================
 * Bạn có thể cung cấp sẵn vài bảng như QVGA RGB565, QQVGA, JPEG...
 * Dưới đây là ví dụ khai báo một bảng QVGA/RGB565.
 */
extern const ov2640_regval_t OV2640_QVGA_RGB565[];

#ifdef __cplusplus
}
#endif

#endif /* __OV2640_H__ */

