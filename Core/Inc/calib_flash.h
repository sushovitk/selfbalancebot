#ifndef CALIB_FLASH_H
#define CALIB_FLASH_H

#include "stm32l4xx_hal.h"
#include <stdbool.h>

/* ── Flash config for STM32L4R5ZI ─────────────────────────────────────
 * Bank 2, last page (page 255): 0x081FF000, size 4KB
 * Make sure this page is NOT used by your firmware in your .ld file
 * ──────────────────────────────────────────────────────────────────── */
#define CALIB_FLASH_ADDR    0x081FF000UL
#define CALIB_FLASH_BANK    FLASH_BANK_2
#define CALIB_FLASH_PAGE    255
#define CALIB_MAGIC         0xBEEFCAFEUL
#define CALIB_DATA_SIZE     22            // BNO055 offset registers are 22 bytes

bool Calib_Flash_Save(const uint8_t *data);
bool Calib_Flash_Load(uint8_t *data);
void Calib_Flash_Erase(void);

#endif
