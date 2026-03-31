//
// pixy2.h — C-compatible public header for the STM32 Pixy2 driver.
//
// This header exposes the same API that main.c has always used:
//   Pixy2_Init(), Pixy2_GetVersion(), Pixy2_GetBlocks(), Pixy2_SetLED()
// and the same Pixy2Block / Pixy2 structs (field names unchanged).
//
// The implementation is in pixy2.cpp (C++) which wraps TPixy2<Link2SPI_SS>.
// The extern "C" guards here ensure the function symbols have C linkage so
// the C linker in main.c can resolve them without name-mangling.
//

#ifndef PIXY2_H
#define PIXY2_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
//  Constants
// ---------------------------------------------------------------------------
#define PIXY2_MAX_BLOCKS    10
#define PIXY2_TIMEOUT_MS   500

// ---------------------------------------------------------------------------
//  Pixy2Block — one detected colour-connected-component object.
//
//  Field names use the plain (non-m_prefixed) form so that main.c code such
//  as  pixy.blocks[i].signature  continues to compile without changes.
//  Internally, pixy2.cpp copies the library's  m_signature / m_x / ...
//  fields into these fields after each GetBlocks call.
// ---------------------------------------------------------------------------
typedef struct {
    uint16_t signature;
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
    int16_t  angle;    // colour-code blocks only; 0 for normal blocks
    uint8_t  index;
    uint8_t  age;
} Pixy2Block;

// ---------------------------------------------------------------------------
//  Pixy2Version — firmware / hardware version information
// ---------------------------------------------------------------------------
typedef struct {
    uint8_t hardware[2];
    uint8_t firmware[3];
    char    firmwareType[10];
} Pixy2Version;

// ---------------------------------------------------------------------------
//  Pixy2 — top-level handle passed to every API function.
//  The fields below are updated by pixy2.cpp after each call so that
//  application code can read them directly (e.g. pixy.blocks[i].signature).
// ---------------------------------------------------------------------------
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *csPort;
    uint16_t           csPin;
    Pixy2Block         blocks[PIXY2_MAX_BLOCKS];
    uint8_t            numBlocks;
    Pixy2Version       version;
} Pixy2;

// ---------------------------------------------------------------------------
//  Public API
// ---------------------------------------------------------------------------

// Pixy2_Init — configure the link and wait up to 5 s for the Pixy2 to boot.
//   hspi   : pointer to the HAL SPI handle (e.g. &hspi1)
//   csPort : GPIO port for the manual chip-select (e.g. GPIOD)
//   csPin  : GPIO pin  for the manual chip-select (e.g. GPIO_PIN_14)
// Returns HAL_OK on success, HAL_ERROR or HAL_TIMEOUT on failure.
HAL_StatusTypeDef Pixy2_Init(Pixy2 *pixy,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef      *csPort,
                              uint16_t           csPin);

// Pixy2_GetVersion — re-query firmware/hardware version and populate pixy->version.
HAL_StatusTypeDef Pixy2_GetVersion(Pixy2 *pixy);

// Pixy2_GetBlocks — request colour-connected-component data.
//   sigmap    : bitmask of signatures to request (0xff = all)
//   maxBlocks : upper limit on returned blocks
// Returns the number of blocks detected (0..PIXY2_MAX_BLOCKS),
// or -1 on communication error.
// Detected blocks are stored in pixy->blocks[] and pixy->numBlocks.
int Pixy2_GetBlocks(Pixy2 *pixy, uint8_t sigmap, uint8_t maxBlocks);

// Pixy2_SetLED — set the RGB LED on the Pixy2 camera itself.
HAL_StatusTypeDef Pixy2_SetLED(Pixy2 *pixy, uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif

#endif // PIXY2_H
