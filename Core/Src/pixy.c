/* pixy.c — Pixy2 camera driver module
 *
 * This file contains only the Pixy2 chip-select helpers and any
 * other Pixy2-specific utility functions.
 * Peripheral initialisation (SPI1, GPIO) and all HAL boilerplate
 * (main, SystemClock_Config, Error_Handler) live exclusively in main.c.
 */

#include "main.h"
#include <string.h>
#include "pixy2.h"

/* USER CODE BEGIN PD */
#define PIXY2_SEND_SYNC_BYTE  0xae
#define PIXY2_SEND_SYNC_BYTE2 0xc1
#define PIXY2_RECV_SYNC_BYTE  0xaf
#define PIXY2_RECV_SYNC_BYTE2 0xc1
/* USER CODE END PD */

/*
 * hspi1 and pixy are defined in main.c; reference them here with extern.
 * Do NOT re-declare them as plain variables — that is what caused the
 * "multiple definition" linker errors.
 */
extern SPI_HandleTypeDef hspi1;
extern Pixy2 pixy;

/* USER CODE BEGIN 0 */

static inline void Pixy2_CS_Select(void) {
    HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_RESET);
}

static inline void Pixy2_CS_Deselect(void) {
    HAL_GPIO_WritePin(PIXY_CS_GPIO_Port, PIXY_CS_Pin, GPIO_PIN_SET);
}

/* USER CODE END 0 */
