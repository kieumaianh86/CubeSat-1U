#include "sdcard.h"
#include "string.h"
#include "stdio.h"

static FATFS fs;
static uint8_t sd_ready = 0;
static char latest_filename[40] = {0};  // Tăng size cho drive prefix

sd_status_t SD_Init(void) {
    return SD_Mount();
}

sd_status_t SD_Mount(void) {
    // FIX: Dùng "0:" cho STM32H7
    FRESULT res = f_mount(&fs, "0:", 1);
    if (res == FR_OK) {
        sd_ready = 1;
        return SD_OK;
    }
    sd_ready = 0;
    return SD_MOUNT_ERROR;
}

sd_status_t SD_Unmount(void) {
    f_mount(NULL, "0:", 0);
    sd_ready = 0;
    return SD_OK;
}

uint8_t SD_IsReady(void) {
    return sd_ready;
}

// Hàm lưu buffer ảnh vào SD card
sd_status_t SaveBufferToSD(uint8_t *buffer, uint32_t size) {
    static unsigned long photo_number = 1;
    char filename[30];
    FIL file;
    UINT bytes_written;
    
    // Tạo tên file
    sprintf(filename, "0:/IMG_%04lu.jpg", photo_number);
    
    // Mở file để ghi
    if (f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
        return SD_WRITE_ERROR;
    }
    
    // Ghi dữ liệu
    FRESULT res = f_write(&file, buffer, size, &bytes_written);
    
    // Đóng file
    f_close(&file);
    
    if (res == FR_OK && bytes_written == size) {
        photo_number++;  // Tăng số cho ảnh tiếp theo
        return SD_OK;
    }
    
    return SD_WRITE_ERROR;
}

const char* SD_GetStatusString(sd_status_t status) {
    switch (status) {
        case SD_OK:
            return "SD_OK";
        case SD_ERROR:
            return "SD_ERROR";
        case SD_MOUNT_ERROR:
            return "SD_MOUNT_ERROR";
        case SD_NOT_READY:
            return "SD_NOT_READY";
        case SD_WRITE_ERROR:
            return "SD_WRITE_ERROR";
        case SD_READ_ERROR:
            return "SD_READ_ERROR";
    }
    return "SD_UNKNOWN_STATUS";
}

sd_status_t SD_WriteScience(const uint8_t *data, uint32_t len) {
    if (!sd_ready) return SD_NOT_READY;
    
    FIL file;
    UINT bw;
    
    // FIX: Thêm drive prefix "0:"
    snprintf(latest_filename, sizeof(latest_filename), "0:sci_%lu.dat", HAL_GetTick());
    
    FRESULT res = f_open(&file, latest_filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return SD_WRITE_ERROR;
    
    res = f_write(&file, data, len, &bw);
    
    // FIX: Thêm f_sync() để đảm bảo data được ghi xuống SD
    if (res == FR_OK) {
        f_sync(&file);
    }
    
    f_close(&file);
    
    return (res == FR_OK && bw == len) ? SD_OK : SD_WRITE_ERROR;
}

sd_status_t SD_ReadScience(uint8_t *data, uint32_t *len) {
    if (!sd_ready) return SD_NOT_READY;
    if (latest_filename[0] == 0) return SD_READ_ERROR;  // No file written yet
    
    FIL file;
    UINT br;
    
    FRESULT res = f_open(&file, latest_filename, FA_READ);
    if (res != FR_OK) return SD_READ_ERROR;
    
    res = f_read(&file, data, *len, &br);
    f_close(&file);
    
    if (res == FR_OK) {
        *len = br;
        return SD_OK;
    }
    
    return SD_READ_ERROR;
}

sd_status_t SD_DeleteOldest(void) {
    if (!sd_ready) return SD_NOT_READY;
    
    DIR dir;
    FILINFO fno;
    char oldest_name[40] = {0};  // Tăng size cho drive prefix
    DWORD oldest_time = 0xFFFFFFFF;
    
    // FIX: Dùng "0:/" cho STM32H7
    if (f_opendir(&dir, "0:/") != FR_OK) return SD_ERROR;
    
    // Find oldest sci_*.dat file
    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
        if (strncmp(fno.fname, "sci_", 4) == 0) {
            // Extract timestamp from filename
            DWORD file_time = 0;
            sscanf(fno.fname, "sci_%lu.dat", &file_time);
            
            if (file_time < oldest_time) {
                oldest_time = file_time;
                // FIX: Thêm drive prefix
                snprintf(oldest_name, sizeof(oldest_name), "0:%s", fno.fname);
            }
        }
    }
    
    f_closedir(&dir);
    
    // Delete oldest file if found
    if (oldest_name[0]) {
        if (f_unlink(oldest_name) == FR_OK) {
            return SD_OK;
        }
    }
    
    return SD_ERROR;
}

uint32_t SD_GetFreeKB(void) {
    if (!sd_ready) return 0;
    
    FATFS *fs_ptr;
    DWORD fre_clust;
    
    // FIX: Dùng "0:" và tính toán chính xác
    if (f_getfree("0:", &fre_clust, &fs_ptr) == FR_OK) {
        // Tính chính xác: free_clusters * sectors_per_cluster * bytes_per_sector / 1024
        uint32_t free_sectors = fre_clust * fs_ptr->csize;
        
        #if _MAX_SS != _MIN_SS
        uint32_t sector_size = fs_ptr->ssize;
        #else
        uint32_t sector_size = _MIN_SS;  // Thường là 512
        #endif
        
        return (free_sectors * sector_size) / 1024;
    }
    return 0;
}

uint32_t SD_GetUsedKB(void) {
    if (!sd_ready) return 0;
    
    FATFS *fs_ptr;
    DWORD fre_clust, tot_clust;
    
    // FIX: Tính used = total - free (chính xác hơn)
    if (f_getfree("0:", &fre_clust, &fs_ptr) == FR_OK) {
        tot_clust = fs_ptr->n_fatent - 2;  // Total clusters
        DWORD used_clust = tot_clust - fre_clust;
        
        uint32_t used_sectors = used_clust * fs_ptr->csize;
        
        #if _MAX_SS != _MIN_SS
        uint32_t sector_size = fs_ptr->ssize;
        #else
        uint32_t sector_size = _MIN_SS;
        #endif
        
        return (used_sectors * sector_size) / 1024;
    }
    return 0;
}

sd_status_t SD_WriteFile(const char* filename, const uint8_t *data, uint32_t len) {
    if (!sd_ready) return SD_NOT_READY;
    
    FIL file;
    UINT bw;
    char fullpath[50];
    
    // Thêm drive prefix
    snprintf(fullpath, sizeof(fullpath), "0:%s", filename);
    
    FRESULT res = f_open(&file, fullpath, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) return SD_WRITE_ERROR;
    
    res = f_write(&file, data, len, &bw);
    if (res == FR_OK) {
        f_sync(&file);
    }
    f_close(&file);
    
    return (res == FR_OK && bw == len) ? SD_OK : SD_WRITE_ERROR;
}

sd_status_t SD_ReadFile(const char* filename, uint8_t *data, uint32_t *len) {
    if (!sd_ready) return SD_NOT_READY;
    
    FIL file;
    UINT br;
    char fullpath[50];
    
    snprintf(fullpath, sizeof(fullpath), "0:%s", filename);
    
    FRESULT res = f_open(&file, fullpath, FA_READ);
    if (res != FR_OK) return SD_READ_ERROR;
    
    res = f_read(&file, data, *len, &br);
    f_close(&file);
    
    if (res == FR_OK) {
        *len = br;
        return SD_OK;
    }
    return SD_READ_ERROR;
}

sd_status_t SD_DeleteFile(const char* filename) {
    if (!sd_ready) return SD_NOT_READY;
    
    char fullpath[50];
    snprintf(fullpath, sizeof(fullpath), "0:%s", filename);
    
    FRESULT res = f_unlink(fullpath);
    return (res == FR_OK) ? SD_OK : SD_ERROR;
}