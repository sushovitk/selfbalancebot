//
// pixy2.cpp — STM32 implementation of the pixy2.h C API.
//
// This file is compiled as C++ (STM32CubeIDE does this automatically from
// the .cpp extension).  It holds a single static TPixy2<Link2SPI_SS> instance
// and exposes extern "C" wrapper functions that match the signatures declared
// in pixy2.h, which is what main.c calls.
//
// Architecture
// ────────────
//  main.c  (C)
//    │  calls Pixy2_Init / Pixy2_GetBlocks / …  via pixy2.h
//    ▼
//  pixy2.cpp  (C++)
//    │  holds static TPixy2<Link2SPI_SS> pixyLib
//    │  configures Link2SPI_SS with HAL handles
//    │  forwards calls to pixyLib.init() / pixyLib.ccc.getBlocks() / …
//    │  copies Block::m_* fields → Pixy2Block::* fields for main.c to read
//    ▼
//  TPixy2.h / Pixy2SPI_SS.h  (Pixy2 library, STM32-ported)
//    │  implements the Pixy2 serial protocol over SPI
//    ▼
//  HAL_SPI_TransmitReceive / HAL_GPIO_WritePin  (STM32 HAL)
//

#include "pixy2.h"         // C API we are implementing
#include "Pixy2SPI_SS.h"   // TPixy2<Link2SPI_SS> — pulls in TPixy2.h and sub-modules
#include "Pixy2Line.h"

#include <string.h>        // memcpy

// ---------------------------------------------------------------------------
//  Single global library instance.
//  There is only one Pixy2 connected to the board, so a global is appropriate.
//  Constructor runs before main() via the C++ runtime init table.
// ---------------------------------------------------------------------------
static Pixy2SPI_SS pixyLib;   // Pixy2SPI_SS == TPixy2<Link2SPI_SS>

// ---------------------------------------------------------------------------
//  Helper — copy library Block (m_* fields) into our C Pixy2Block (* fields).
//  Field layout is identical; only names differ, so this is a safe field-copy.
// ---------------------------------------------------------------------------
static void copyBlock(Pixy2Block *dst, const Block *src)
{
    dst->signature = src->m_signature;
    dst->x         = src->m_x;
    dst->y         = src->m_y;
    dst->width     = src->m_width;
    dst->height    = src->m_height;
    dst->angle     = src->m_angle;
    dst->index     = src->m_index;
    dst->age       = src->m_age;
}

// ---------------------------------------------------------------------------
//  extern "C" — all functions below have C linkage so main.c can call them
//  without C++ name-mangling.
// ---------------------------------------------------------------------------
extern "C" {

// ---------------------------------------------------------------------------
//  Pixy2_Init
//
//  1. Stores the HAL SPI handle and CS GPIO in pixy (for the caller's records)
//     and in the Link2SPI_SS instance (so it can drive the SPI peripheral).
//  2. Calls pixyLib.init() which:
//       a. Calls Link2SPI_SS::open() — parks CS high, returns 0.
//       b. Polls getVersion() for up to 5 s — the Pixy2 takes ~1–2 s to boot.
//       c. On success calls getResolution() to cache frame dimensions.
//  3. Copies the version info returned by the library into pixy->version.
// ---------------------------------------------------------------------------
HAL_StatusTypeDef Pixy2_Init(Pixy2 *pixy,
                              SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef      *csPort,
                              uint16_t           csPin)
{
    // Store handles in the C struct (informational; used if caller reads them)
    pixy->hspi      = hspi;
    pixy->csPort    = csPort;
    pixy->csPin     = csPin;
    pixy->numBlocks = 0;

    // Give the HAL handles to the SPI link class before init() calls open().
    pixyLib.m_link.configure(hspi, csPort, csPin);

    // init() blocks until Pixy2 responds or 5 s elapse.
    int8_t res = pixyLib.init();
    if (res < 0)
        return (res == PIXY_RESULT_TIMEOUT) ? HAL_TIMEOUT : HAL_ERROR;

    // Copy version info into the C struct so main.c can inspect it if wanted.
    if (pixyLib.version != NULL)
    {
        pixy->version.hardware[0] = (uint8_t)(pixyLib.version->hardware & 0xFF);
        pixy->version.hardware[1] = (uint8_t)(pixyLib.version->hardware >> 8);
        pixy->version.firmware[0] = pixyLib.version->firmwareMajor;
        pixy->version.firmware[1] = pixyLib.version->firmwareMinor;
        pixy->version.firmware[2] = (uint8_t)(pixyLib.version->firmwareBuild & 0xFF);
        memcpy(pixy->version.firmwareType,
               pixyLib.version->firmwareType,
               sizeof(pixy->version.firmwareType));
    }

    return HAL_OK;
}

// ---------------------------------------------------------------------------
//  Pixy2_GetVersion
//
//  Re-issues the version request and repopulates pixy->version.
//  Useful if the caller wants to re-confirm the camera is still alive.
// ---------------------------------------------------------------------------
HAL_StatusTypeDef Pixy2_GetVersion(Pixy2 *pixy)
{
    int8_t res = pixyLib.getVersion();
    if (res < 0)
        return HAL_ERROR;
    if (pixyLib.version == NULL)
        return HAL_ERROR;

    pixy->version.hardware[0] = (uint8_t)(pixyLib.version->hardware & 0xFF);
    pixy->version.hardware[1] = (uint8_t)(pixyLib.version->hardware >> 8);
    pixy->version.firmware[0] = pixyLib.version->firmwareMajor;
    pixy->version.firmware[1] = pixyLib.version->firmwareMinor;
    pixy->version.firmware[2] = (uint8_t)(pixyLib.version->firmwareBuild & 0xFF);
    memcpy(pixy->version.firmwareType,
           pixyLib.version->firmwareType,
           sizeof(pixy->version.firmwareType));

    return HAL_OK;
}

// ---------------------------------------------------------------------------
//  Pixy2_GetBlocks
//
//  Requests colour-connected-component data from the Pixy2.
//
//    wait=false  — non-blocking: returns immediately if Pixy2 is busy
//                  (returns -1 from this function, main loop treats it as 0)
//    sigmap      — bitmask of signatures to include (0xff = all seven)
//    maxBlocks   — caller's upper limit (also clamped to PIXY2_MAX_BLOCKS)
//
//  On success:
//    * pixy->numBlocks is updated.
//    * pixy->blocks[0..numBlocks-1] are populated with copied block data.
//    * The function return value equals pixy->numBlocks.
//
//  Returns -1 on any error or BUSY (caller checks count > 0).
// ---------------------------------------------------------------------------
int Pixy2_GetBlocks(Pixy2 *pixy, uint8_t sigmap, uint8_t maxBlocks)
{
    // Clamp maxBlocks to our static array size.
    if (maxBlocks > PIXY2_MAX_BLOCKS)
        maxBlocks = PIXY2_MAX_BLOCKS;

    // Non-blocking request — returns immediately if Pixy2 is still processing
    // the previous frame.  PIXY_RESULT_BUSY (-2) is treated the same as 0.
    int8_t n = pixyLib.ccc.getBlocks(/*wait=*/false, sigmap, maxBlocks);
    if (n <= 0)
    {
        pixy->numBlocks = 0;
        return (n == 0) ? 0 : -1;
    }

    // Clamp to our array and copy fields from Block (m_*) → Pixy2Block (plain).
    pixy->numBlocks = (uint8_t)((n > (int8_t)PIXY2_MAX_BLOCKS)
                                    ? PIXY2_MAX_BLOCKS
                                    : (uint8_t)n);

    for (uint8_t i = 0; i < pixy->numBlocks; i++)
        copyBlock(&pixy->blocks[i], &pixyLib.ccc.blocks[i]);

    return (int)pixy->numBlocks;
}

// ---------------------------------------------------------------------------
//  Pixy2_SetLED
//
//  Sets the RGB LED on the Pixy2 camera board itself (not the Nucleo LEDs).
// ---------------------------------------------------------------------------
HAL_StatusTypeDef Pixy2_SetLED(Pixy2 *pixy, uint8_t r, uint8_t g, uint8_t b)
{
    (void)pixy;  // pixy handle unused — library uses its own internal state
    int8_t res = pixyLib.setLED(r, g, b);
    return (res >= 0) ? HAL_OK : HAL_ERROR;
}

} // extern "C"
