#include "OV2640.h"
#include "main.h"
#include <string.h>

/* =============================
 *  Cấu hình thời gian & cache
 * ============================= */
#ifndef OV2640_I2C_TIMEOUT_MS
#define OV2640_I2C_TIMEOUT_MS   100U
#endif

#ifndef OV2640_RESET_DELAY_MS
#define OV2640_RESET_DELAY_MS   100U
#endif

/* Bật 1 nếu muốn Invalidate D-Cache sau khi DMA nhận xong (H7, CM7) */
#ifndef OV2640_DCACHE_MAINTENANCE
#define OV2640_DCACHE_MAINTENANCE  1
#endif

/* =============================
 *  Static helpers (riêng .c)
 * ============================= */
static ov2640_status_t _write_reg(OV2640_Handle *cam, uint8_t reg, uint8_t val);
static ov2640_status_t _read_reg (OV2640_Handle *cam, uint8_t reg, uint8_t *val);
static ov2640_status_t _apply_table(OV2640_Handle *cam, const ov2640_regval_t *tbl);

/* =============================
 *  Public API
 * ============================= */

ov2640_status_t OV2640_Init(OV2640_Handle *cam,
                            I2C_HandleTypeDef *hi2c,
                            DCMI_HandleTypeDef *hdcmi)
{
    if (!cam || !hi2c || !hdcmi) return OV2640_INVALID_PARAM;

    memset(cam, 0, sizeof(*cam));
    cam->hi2c  = hi2c;
    cam->hdcmi = hdcmi;

    /* Mặc định (QVGA RGB565)  */
    cam->width  = 320;
    cam->height = 240;
    cam->bpp    = 2;

    /* Kiểm tra thiết bị trên bus */
    return OV2640_Ping(cam);
}

ov2640_status_t OV2640_Ping(OV2640_Handle *cam)
{
    if (!cam || !cam->hi2c) return OV2640_INVALID_PARAM;

    if (HAL_I2C_IsDeviceReady(cam->hi2c, (OV2640_I2C_ADDR_7BIT << 1),
                              2, OV2640_I2C_TIMEOUT_MS) == HAL_OK) {
        return OV2640_OK;
    }
    return OV2640_DEVICE_NOT_FOUND;
}

ov2640_status_t OV2640_Reset(OV2640_Handle *cam)
{
    if (!cam) return OV2640_INVALID_PARAM;

    /* Chọn bank 1, ghi reset mềm */
    if (_write_reg(cam, 0xFF, 0x01) != OV2640_OK) return OV2640_I2C_ERROR;
    if (_write_reg(cam, 0x12, 0x80) != OV2640_OK) return OV2640_I2C_ERROR;

    HAL_Delay(OV2640_RESET_DELAY_MS);
    return OV2640_OK;
}

ov2640_status_t OV2640_WriteReg(OV2640_Handle *cam, uint8_t reg, uint8_t val)
{
    return _write_reg(cam, reg, val);
}

ov2640_status_t OV2640_ReadReg(OV2640_Handle *cam, uint8_t reg, uint8_t *val)
{
    return _read_reg(cam, reg, val);
}

ov2640_status_t OV2640_ApplyTable(OV2640_Handle *cam, const ov2640_regval_t *tbl)
{
    return _apply_table(cam, tbl);
}

ov2640_status_t OV2640_StartSnapshot(OV2640_Handle *cam, void *buf, uint32_t buf_len_bytes)
{
    if (!cam || !cam->hdcmi || !buf) return OV2640_INVALID_PARAM;
    if (cam->width == 0 || cam->height == 0 || cam->bpp == 0) return OV2640_INVALID_PARAM;

    uint32_t total_bytes = cam->width * cam->height * cam->bpp;
    if (buf_len_bytes < total_bytes) return OV2640_ERROR;

    /* DCMI DMA: length = số WORD 32-bit cần nhận */
    uint32_t len_words = (total_bytes + 3U) / 4U;

    cam->frame_ready = 0;

    /* Nếu cần, làm sạch cache trước khi nhận (tuỳ cấu hình hệ thống) */
#if (OV2640_DCACHE_MAINTENANCE)
  #if defined (SCB_CleanInvalidateDCache) || defined (SCB_CleanDCache)
    /* Với H7, nếu buffer ở DCache-able region và dùng DMA, cân nhắc Clean trước khi nhận.
       Ở đây ta thường chỉ cần Invalidate SAU khi nhận xong. */
  #endif
#endif

    if (HAL_DCMI_Start_DMA(cam->hdcmi, DCMI_MODE_SNAPSHOT,
                           (uint32_t)buf, len_words) != HAL_OK) {
        return OV2640_ERROR;
    }
    return OV2640_OK;
}

ov2640_status_t OV2640_WaitFrame(OV2640_Handle *cam, uint32_t timeout_ms)
{
    if (!cam || !cam->hdcmi) return OV2640_INVALID_PARAM;

    uint32_t t0 = HAL_GetTick();
    while (!cam->frame_ready) {
        if (timeout_ms && (HAL_GetTick() - t0 >= timeout_ms)) {
            HAL_DCMI_Stop(cam->hdcmi);
            return OV2640_TIMEOUT;
        }
    }

    /* Dừng DCMI sau khi xong khung */
    HAL_DCMI_Stop(cam->hdcmi);

    /* Invalidate D-Cache để CPU thấy dữ liệu DMA mới */
#if (OV2640_DCACHE_MAINTENANCE)
  #if defined (SCB_InvalidateDCache_by_Addr)
    /* CHÚ Ý: địa chỉ phải 32-byte aligned khi gọi Invalidate theo vùng */
    /* Không có con trỏ buffer ở đây; người gọi biết địa chỉ buffer -> nên Invalidate bên ngoài
       nơi gọi OV2640_WaitFrame, khi có sẵn pointer buffer. */
  #endif
#endif

    return OV2640_OK;
}

void OV2640_OnFrameEvent(OV2640_Handle *cam)
{
    if (cam) cam->frame_ready = 1;
}

/* =============================
 *  Static helpers
 * ============================= */
static ov2640_status_t _write_reg(OV2640_Handle *cam, uint8_t reg, uint8_t val)
{
    if (!cam || !cam->hi2c) return OV2640_INVALID_PARAM;

    if (HAL_I2C_Mem_Write(cam->hi2c, (OV2640_I2C_ADDR_7BIT << 1),
                          reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, OV2640_I2C_TIMEOUT_MS) != HAL_OK) {
        return OV2640_I2C_ERROR;
    }
    return OV2640_OK;
}

static ov2640_status_t _read_reg(OV2640_Handle *cam, uint8_t reg, uint8_t *val)
{
    if (!cam || !cam->hi2c || !val) return OV2640_INVALID_PARAM;

    if (HAL_I2C_Mem_Read(cam->hi2c, (OV2640_I2C_ADDR_7BIT << 1),
                         reg, I2C_MEMADD_SIZE_8BIT,
                         val, 1, OV2640_I2C_TIMEOUT_MS) != HAL_OK) {
        return OV2640_I2C_ERROR;
    }
    return OV2640_OK;
}

static ov2640_status_t _apply_table(OV2640_Handle *cam, const ov2640_regval_t *tbl)
{
    if (!cam || !tbl) return OV2640_INVALID_PARAM;

    for (const ov2640_regval_t *p = tbl; !(p->reg == 0xFF && p->val == 0xFF); ++p) {
        ov2640_status_t st = _write_reg(cam, p->reg, p->val);
        if (st != OV2640_OK) return st;
        HAL_Delay(1); /* nhỏ để sensor theo kịp */
    }
    return OV2640_OK;
}

/* =============================
 *  Preset tables (ví dụ)
 * ============================= */

/* BẢNG MẪU: chỉ reset để bạn compile được.
 * Hãy dán bảng QVGA + RGB565 thực tế của bạn vào giữa 2 dòng TODO bên dưới,
 * rồi giữ lại OV2640_TBL_END ở cuối.
 */
const ov2640_regval_t OV2640_QVGA_RGB565[] = {
    /* Reset mềm */
    {0xFF, 0x01},
    {0x12, 0x80},
    /* chờ ở ngoài bằng HAL_Delay(OV2640_RESET_DELAY_MS) */

    /* ========== TODO: DÁN BẢNG CẤU HÌNH QVGA RGB565 CỦA BẠN TẠI ĐÂY ========== */
    /* Ví dụ: chọn bank DSP (0xFF=0x00), set format RGB565, scale/crop xuống QVGA, vv. */
    /* Tham khảo bảng đã test từ project của bạn để đảm bảo màu sắc & timing đúng.     */
    /* ============================================================================ */

    OV2640_TBL_END
};

/* =============================
 *  Gợi ý tích hợp callback
 * =============================
 *  Trong user code:
 *
 *  extern OV2640_Handle g_cam;
 *  void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
 *  {
 *      OV2640_OnFrameEvent(&g_cam);
 *  }
 *
 *  Sau khi WaitFrame() xong, nếu hệ thống bật D-Cache và buffer ở vùng cache,
 *  bạn nên gọi:
 *
 *  #if defined (SCB_InvalidateDCache_by_Addr)
 *      SCB_InvalidateDCache_by_Addr((void*)frame_buf, cam.width*cam.height*cam.bpp);
 *  #endif
 *
 *  để CPU thấy dữ liệu mới do DMA ghi.
 */