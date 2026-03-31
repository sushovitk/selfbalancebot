//
// TPixy2.h — main Pixy2 template class (STM32 HAL port)
//
// Original: Charmed Labs LLC / CMU — GNU GPL v2
// STM32 port: Arduino-specific API calls replaced with STM32 HAL equivalents.
//
// Changes from original:
//   * #include "stm32l4xx_hal.h" replaces Arduino.h / SPI.h
//   * millis()            → HAL_GetTick()        (both return uint32_t ms)
//   * delayMicroseconds() → HAL_Delay() rounded to nearest ms
//   * Serial.println()    → removed (no UART console wired in this project)
//   * #define PIXY_DEBUG  → commented out (would need Serial to be useful)
//   * malloc/free remain  → provided by newlib via sysmem.c / syscalls.c
//

#ifndef _TPIXY2_H
#define _TPIXY2_H

#include "stm32l4xx_hal.h"   // HAL_GetTick, HAL_Delay, uint8_t, etc.
#include <stdlib.h>           // malloc, free
#include <string.h>           // strncpy, memcpy

// ---------------------------------------------------------------------------
//  Arduino timing / delay replacements
//  Defined as static inline functions (not macros) to avoid accidental
//  double-evaluation and to survive inclusion from multiple translation units.
//  They are defined HERE, before the sub-header includes below, so that
//  Pixy2CCC.h / Pixy2Line.h / Pixy2Video.h can use them in their template
//  method bodies.
// ---------------------------------------------------------------------------
static inline uint32_t millis(void)
{
    return HAL_GetTick();          // SysTick-driven, 1 ms resolution
}

static inline void delayMicroseconds(uint32_t us)
{
    // HAL_Delay is 1 ms resolution; round up so we always wait at least us.
    // The delays in the Pixy2 library (25 µs, 500 µs, 5000 µs) are purely
    // "don't thrash the bus" guards — 1–5 ms is safe in every case.
    HAL_Delay((us + 999U) / 1000U);
}

// ---------------------------------------------------------------------------
//  Pixy2 packet-protocol constants (unchanged from original)
// ---------------------------------------------------------------------------
// #define PIXY_DEBUG              // Uncomment to enable debug output (needs UART)

#define PIXY_DEFAULT_ARGVAL          0x80000000UL
#define PIXY_BUFFERSIZE              0x104
#define PIXY_CHECKSUM_SYNC           0xc1af
#define PIXY_NO_CHECKSUM_SYNC        0xc1ae
#define PIXY_SEND_HEADER_SIZE        4
#define PIXY_MAX_PROGNAME            33

#define PIXY_TYPE_REQUEST_CHANGE_PROG   0x02
#define PIXY_TYPE_REQUEST_RESOLUTION    0x0c
#define PIXY_TYPE_RESPONSE_RESOLUTION   0x0d
#define PIXY_TYPE_REQUEST_VERSION       0x0e
#define PIXY_TYPE_RESPONSE_VERSION      0x0f
#define PIXY_TYPE_RESPONSE_RESULT       0x01
#define PIXY_TYPE_RESPONSE_ERROR        0x03
#define PIXY_TYPE_REQUEST_BRIGHTNESS    0x10
#define PIXY_TYPE_REQUEST_SERVO         0x12
#define PIXY_TYPE_REQUEST_LED           0x14
#define PIXY_TYPE_REQUEST_LAMP          0x16
#define PIXY_TYPE_REQUEST_FPS           0x18

#define PIXY_RESULT_OK                  0
#define PIXY_RESULT_ERROR              -1
#define PIXY_RESULT_BUSY               -2
#define PIXY_RESULT_CHECKSUM_ERROR     -3
#define PIXY_RESULT_TIMEOUT            -4
#define PIXY_RESULT_BUTTON_OVERRIDE    -5
#define PIXY_RESULT_PROG_CHANGING      -6

#define PIXY_RCS_MIN_POS               0
#define PIXY_RCS_MAX_POS               1000L
#define PIXY_RCS_CENTER_POS            ((PIXY_RCS_MAX_POS - PIXY_RCS_MIN_POS) / 2)

// Sub-module headers — must come AFTER millis/delayMicroseconds are defined
// so the preprocessor can resolve those identifiers inside the template bodies.
#include "Pixy2CCC.h"
#include "Pixy2Line.h"
#include "Pixy2Video.h"

// ---------------------------------------------------------------------------
//  Version struct
// ---------------------------------------------------------------------------
struct Version
{
    // print() removed: no Serial on STM32 (would need ITM/UART redirect).
    void print() {}

    uint16_t hardware;
    uint8_t  firmwareMajor;
    uint8_t  firmwareMinor;
    uint16_t firmwareBuild;
    char     firmwareType[10];
};

// ---------------------------------------------------------------------------
//  TPixy2 — main template class
//  LinkType must implement: open(uint32_t), close(), send(buf,len),
//                           recv(buf,len,cs*), setArg(uint16_t)
// ---------------------------------------------------------------------------
template <class LinkType> class TPixy2
{
public:
    TPixy2();
    ~TPixy2();

    int8_t init(uint32_t arg = PIXY_DEFAULT_ARGVAL);

    int8_t getVersion();
    int8_t changeProg(const char *prog);
    int8_t setServos(uint16_t s0, uint16_t s1);
    int8_t setCameraBrightness(uint8_t brightness);
    int8_t setLED(uint8_t r, uint8_t g, uint8_t b);
    int8_t setLamp(uint8_t upper, uint8_t lower);
    int8_t getResolution();
    int8_t getFPS();

    Version  *version;
    uint16_t  frameWidth;
    uint16_t  frameHeight;

    // Sub-modules (public members — same as original)
    Pixy2CCC<LinkType>   ccc;
    friend class Pixy2CCC<LinkType>;

    Pixy2Line<LinkType>  line;
    friend class Pixy2Line<LinkType>;

    Pixy2Video<LinkType> video;
    friend class Pixy2Video<LinkType>;

    LinkType m_link;

    // Internal packet buffers — exposed so sub-module templates can access them.
    uint8_t *m_buf;
    uint8_t *m_bufPayload;
    uint8_t  m_type;
    uint8_t  m_length;
    bool     m_cs;

private:
    int16_t getSync();
    int16_t recvPacket();
    int16_t sendPacket();
};

// ---------------------------------------------------------------------------
//  Constructor / destructor
// ---------------------------------------------------------------------------
template <class LinkType> TPixy2<LinkType>::TPixy2()
    : ccc(this), line(this), video(this)
{
    m_buf        = (uint8_t *)malloc(PIXY_BUFFERSIZE);
    m_bufPayload = m_buf + PIXY_SEND_HEADER_SIZE;
    frameWidth   = 0;
    frameHeight  = 0;
    version      = NULL;
}

template <class LinkType> TPixy2<LinkType>::~TPixy2()
{
    m_link.close();
    free(m_buf);
}

// ---------------------------------------------------------------------------
//  init() — open the link then poll getVersion() for up to 5 s.
//  Returns PIXY_RESULT_OK on success, PIXY_RESULT_TIMEOUT if Pixy2 silent.
// ---------------------------------------------------------------------------
template <class LinkType> int8_t TPixy2<LinkType>::init(uint32_t arg)
{
    int8_t res;

    res = m_link.open(arg);
    if (res < 0)
        return res;

    // Wait for Pixy2 to boot (can take ~1–2 s after power-on).
    // getVersion() is an effective "ping"; retry for up to 5 s.
    uint32_t t0 = millis();
    while (millis() - t0 < 5000U)
    {
        if (getVersion() >= 0)
        {
            getResolution();
            return PIXY_RESULT_OK;
        }
        delayMicroseconds(5000); // ~5 ms back-off between attempts
    }
    return PIXY_RESULT_TIMEOUT;
}

// ---------------------------------------------------------------------------
//  getSync() — scan incoming bytes until 0xC1AE or 0xC1AF sync word found.
// ---------------------------------------------------------------------------
template <class LinkType> int16_t TPixy2<LinkType>::getSync()
{
    uint8_t  i, j, c, cprev;
    int16_t  res;
    uint16_t start;

    for (i = 0, j = 0, cprev = 0; ; i++)
    {
        res = m_link.recv(&c, 1);
        if (res >= PIXY_RESULT_OK)
        {
            start  = cprev;
            start |= (uint16_t)c << 8;
            cprev  = c;

            if (start == PIXY_CHECKSUM_SYNC)
            {
                m_cs = true;
                return PIXY_RESULT_OK;
            }
            if (start == PIXY_NO_CHECKSUM_SYNC)
            {
                m_cs = false;
                return PIXY_RESULT_OK;
            }
        }

        // After 4 bytes with no sync, back off briefly and retry.
        // Pixy2 guarantees a response within 100 µs; we allow 4 × 25 µs retries.
        if (i >= 4)
        {
            if (j >= 4)
            {
                // No response — return error (Serial.println removed)
                return PIXY_RESULT_ERROR;
            }
            delayMicroseconds(25);
            j++;
            i = 0;
        }
    }
}

// ---------------------------------------------------------------------------
//  recvPacket() — receive a complete Pixy2 response packet.
// ---------------------------------------------------------------------------
template <class LinkType> int16_t TPixy2<LinkType>::recvPacket()
{
    uint16_t csCalc, csSerial;
    int16_t  res;

    res = getSync();
    if (res < 0)
        return res;

    if (m_cs) // checksum packet
    {
        res = m_link.recv(m_buf, 4);
        if (res < 0) return res;

        m_type   = m_buf[0];
        m_length = m_buf[1];
        csSerial = *(uint16_t *)&m_buf[2];

        res = m_link.recv(m_buf, m_length, &csCalc);
        if (res < 0) return res;

        if (csSerial != csCalc)
            return PIXY_RESULT_CHECKSUM_ERROR;
    }
    else // no-checksum packet
    {
        res = m_link.recv(m_buf, 2);
        if (res < 0) return res;

        m_type   = m_buf[0];
        m_length = m_buf[1];

        res = m_link.recv(m_buf, m_length);
        if (res < 0) return res;
    }
    return PIXY_RESULT_OK;
}

// ---------------------------------------------------------------------------
//  sendPacket() — transmit a request packet.
// ---------------------------------------------------------------------------
template <class LinkType> int16_t TPixy2<LinkType>::sendPacket()
{
    m_buf[0] = PIXY_NO_CHECKSUM_SYNC & 0xff;
    m_buf[1] = PIXY_NO_CHECKSUM_SYNC >> 8;
    m_buf[2] = m_type;
    m_buf[3] = m_length;
    return m_link.send(m_buf, m_length + PIXY_SEND_HEADER_SIZE);
}

// ---------------------------------------------------------------------------
//  Public API — unchanged from original
// ---------------------------------------------------------------------------
template <class LinkType> int8_t TPixy2<LinkType>::changeProg(const char *prog)
{
    int32_t res;
    while (1)
    {
        strncpy((char *)m_bufPayload, prog, PIXY_MAX_PROGNAME);
        m_length = PIXY_MAX_PROGNAME;
        m_type   = PIXY_TYPE_REQUEST_CHANGE_PROG;
        sendPacket();
        if (recvPacket() == 0)
        {
            res = *(int32_t *)m_buf;
            if (res > 0)
            {
                getResolution();
                return PIXY_RESULT_OK;
            }
        }
        else
            return PIXY_RESULT_ERROR;
        delayMicroseconds(1000);
    }
}

template <class LinkType> int8_t TPixy2<LinkType>::getVersion()
{
    m_length = 0;
    m_type   = PIXY_TYPE_REQUEST_VERSION;
    sendPacket();
    if (recvPacket() == 0)
    {
        if (m_type == PIXY_TYPE_RESPONSE_VERSION)
        {
            version = (Version *)m_buf;
            return m_length;
        }
        else if (m_type == PIXY_TYPE_RESPONSE_ERROR)
            return PIXY_RESULT_BUSY;
    }
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::getResolution()
{
    m_length       = 1;
    m_bufPayload[0] = 0;
    m_type         = PIXY_TYPE_REQUEST_RESOLUTION;
    sendPacket();
    if (recvPacket() == 0)
    {
        if (m_type == PIXY_TYPE_RESPONSE_RESOLUTION)
        {
            frameWidth  = *(uint16_t *)m_buf;
            frameHeight = *(uint16_t *)(m_buf + sizeof(uint16_t));
            return PIXY_RESULT_OK;
        }
        else
            return PIXY_RESULT_ERROR;
    }
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::setCameraBrightness(uint8_t brightness)
{
    m_bufPayload[0] = brightness;
    m_length = 1;
    m_type   = PIXY_TYPE_REQUEST_BRIGHTNESS;
    sendPacket();
    if (recvPacket() == 0)
        return (int8_t)(*(uint32_t *)m_buf);
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::setServos(uint16_t s0, uint16_t s1)
{
    *(uint16_t *)(m_bufPayload + 0) = s0;
    *(uint16_t *)(m_bufPayload + 2) = s1;
    m_length = 4;
    m_type   = PIXY_TYPE_REQUEST_SERVO;
    sendPacket();
    if (recvPacket() == 0 &&
        m_type   == PIXY_TYPE_RESPONSE_RESULT &&
        m_length == 4)
        return (int8_t)(*(uint32_t *)m_buf);
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::setLED(uint8_t r, uint8_t g, uint8_t b)
{
    m_bufPayload[0] = r;
    m_bufPayload[1] = g;
    m_bufPayload[2] = b;
    m_length = 3;
    m_type   = PIXY_TYPE_REQUEST_LED;
    sendPacket();
    if (recvPacket() == 0 &&
        m_type   == PIXY_TYPE_RESPONSE_RESULT &&
        m_length == 4)
        return (int8_t)(*(uint32_t *)m_buf);
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::setLamp(uint8_t upper, uint8_t lower)
{
    m_bufPayload[0] = upper;
    m_bufPayload[1] = lower;
    m_length = 2;
    m_type   = PIXY_TYPE_REQUEST_LAMP;
    sendPacket();
    if (recvPacket() == 0 &&
        m_type   == PIXY_TYPE_RESPONSE_RESULT &&
        m_length == 4)
        return (int8_t)(*(uint32_t *)m_buf);
    return PIXY_RESULT_ERROR;
}

template <class LinkType> int8_t TPixy2<LinkType>::getFPS()
{
    m_length = 0;
    m_type   = PIXY_TYPE_REQUEST_FPS;
    sendPacket();
    if (recvPacket() == 0 &&
        m_type   == PIXY_TYPE_RESPONSE_RESULT &&
        m_length == 4)
        return (int8_t)(*(uint32_t *)m_buf);
    return PIXY_RESULT_ERROR;
}

#endif // _TPIXY2_H
