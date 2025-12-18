#ifndef SD_CARD_H
#define SD_CARD_H

#include "main.h"
#include "fatfs.h"

typedef enum {
    SD_OK = 0,
    SD_ERROR,
    SD_NOT_READY,
    SD_WRITE_ERROR,
    SD_READ_ERROR,
    SD_MOUNT_ERROR
} sd_status_t;

// Init & mount
sd_status_t SD_Init(void);
sd_status_t SD_Mount(void);
sd_status_t SD_Unmount(void);
uint8_t SD_IsReady(void);

// File operations
sd_status_t SD_WriteScience(const uint8_t *data, uint32_t len);
sd_status_t SD_ReadScience(uint8_t *data, uint32_t *len);
sd_status_t SD_DeleteOldest(void);

// Info
uint32_t SD_GetFreeKB(void);
uint32_t SD_GetUsedKB(void);

sd_status_t SD_WriteFile(const char* filename, const uint8_t *data, uint32_t len);
sd_status_t SD_ReadFile(const char* filename, uint8_t *data, uint32_t *len);
sd_status_t SD_DeleteFile(const char* filename);

sd_status_t SaveBufferToSD(uint8_t *buffer, uint32_t size);
const char* SD_GetStatusString(sd_status_t status);
#endif
