#include "calib_flash.h"
#include "BNO055_STM32.h"
#include <string.h>
#include <stdio.h>

/*
 * Flash memory layout at CALIB_FLASH_ADDR:
 * STM32L4 requires 64-bit (double word) writes
 *
 *  Offset  Size   Content
 *  0       4      Magic number (0xBEEFCAFE) — proves data is valid
 *  4       4      Reserved (0xFFFFFFFF)
 *  8       22     Calibration offset bytes from BNO055
 *  30      2      Padding to complete 64-bit boundary
 *  Total:  32 bytes = 4 double words
 */

#define CALIB_TOTAL_DWORDS  4   // 4 x 8 bytes = 32 bytes total

/* ── Erase the calibration flash page ───────────────────────────────── */
void Calib_Flash_Erase(void) {

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = CALIB_FLASH_BANK,
        .Page      = CALIB_FLASH_PAGE,
        .NbPages   = 1
    };
    uint32_t pageError = 0;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &pageError);
    HAL_FLASH_Lock();

    printf("Calibration flash page erased.\r\n");
}

/* ── Save calibration offsets to flash ──────────────────────────────── */
bool Calib_Flash_Save(const uint8_t *data) {

    // Build 32-byte buffer (padded with 0xFF)
    uint8_t buf[32];
    memset(buf, 0xFF, sizeof(buf));

    // Write magic number at bytes 0-3
    uint32_t magic = CALIB_MAGIC;
    memcpy(&buf[0], &magic, 4);

    // Write 22 calibration bytes starting at byte 8
    memcpy(&buf[8], data, CALIB_DATA_SIZE);

    // Erase the page before writing
    Calib_Flash_Erase();

    HAL_FLASH_Unlock();

    //wipe out any ghost errors before touching the memory!
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    bool ok = true;
    for (int i = 0; i < CALIB_TOTAL_DWORDS; i++) {
        uint64_t dword;
        memcpy(&dword, &buf[i * 8], 8);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                              CALIB_FLASH_ADDR + (i * 8),
                              dword) != HAL_OK) {
            printf("Flash write error at dword %d!\r\n", i);
            ok = false;
            break;
        }
    }

    HAL_FLASH_Lock();

    if (ok) printf("Calibration saved to flash successfully.\r\n");
    return ok;
}

/* ── Load calibration offsets from flash ────────────────────────────── */
bool Calib_Flash_Load(uint8_t *data) {

    // Check magic number (flash is memory-mapped on STM32L4, can read directly)
    uint32_t magic = *(volatile uint32_t *)(CALIB_FLASH_ADDR);

    if (magic != CALIB_MAGIC) {
        printf("No valid calibration found in flash.\r\n");
        return false;
    }

    // Copy 22 calibration bytes (starting at offset 8)
    memcpy(data, (const uint8_t *)(CALIB_FLASH_ADDR + 8), CALIB_DATA_SIZE);
    printf("Calibration loaded from flash successfully.\r\n");
    return true;
}
