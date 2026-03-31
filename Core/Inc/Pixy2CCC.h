//
// Pixy2CCC.h — Color Connected Components (block detection) module
//
// Original: Charmed Labs LLC / CMU — GNU GPL v2
// STM32 port: Serial.println() removed from Block::print(); no other changes.
//

#ifndef _PIXY2CCC_H
#define _PIXY2CCC_H

#include <stdint.h>

#define CCC_MAX_SIGNATURE     7

#define CCC_RESPONSE_BLOCKS   0x21
#define CCC_REQUEST_BLOCKS    0x20

// sigmap bit-field constants — OR together to select multiple signatures.
#define CCC_SIG1              1
#define CCC_SIG2              2
#define CCC_SIG3              4
#define CCC_SIG4              8
#define CCC_SIG5              16
#define CCC_SIG6              32
#define CCC_SIG7              64
#define CCC_COLOR_CODES       128
#define CCC_SIG_ALL           0xff

// ---------------------------------------------------------------------------
//  Block — one detected colour-connected-component object.
//  Field layout must match the Pixy2 wire format (14 bytes, little-endian).
// ---------------------------------------------------------------------------
struct Block
{
    // print() body removed: Serial is not available on STM32.
    // Implement a UART/ITM redirect and add your own output here if needed.
    void print() {}

    uint16_t m_signature;
    uint16_t m_x;
    uint16_t m_y;
    uint16_t m_width;
    uint16_t m_height;
    int16_t  m_angle;   // colour-code blocks only; 0 for normal blocks
    uint8_t  m_index;
    uint8_t  m_age;
};

template <class LinkType> class TPixy2;

// ---------------------------------------------------------------------------
//  Pixy2CCC — sub-module for colour connected component queries
// ---------------------------------------------------------------------------
template <class LinkType> class Pixy2CCC
{
public:
    Pixy2CCC(TPixy2<LinkType> *pixy) : m_pixy(pixy) {}

    // getBlocks() — request block data from the Pixy2.
    //   wait     : if true, keep retrying on BUSY; if false, return immediately.
    //   sigmap   : bitmask of signatures to request (CCC_SIG_ALL = 0xff).
    //   maxBlocks: maximum number of blocks to return.
    // Returns number of blocks on success, or a negative PIXY_RESULT_* code.
    int8_t getBlocks(bool wait = true, uint8_t sigmap = CCC_SIG_ALL, uint8_t maxBlocks = 0xff);

    uint8_t  numBlocks;
    Block   *blocks;

private:
    TPixy2<LinkType> *m_pixy;
};

template <class LinkType>
int8_t Pixy2CCC<LinkType>::getBlocks(bool wait, uint8_t sigmap, uint8_t maxBlocks)
{
    blocks    = NULL;
    numBlocks = 0;

    while (1)
    {
        m_pixy->m_bufPayload[0] = sigmap;
        m_pixy->m_bufPayload[1] = maxBlocks;
        m_pixy->m_length = 2;
        m_pixy->m_type   = CCC_REQUEST_BLOCKS;

        m_pixy->sendPacket();
        if (m_pixy->recvPacket() == 0)
        {
            if (m_pixy->m_type == CCC_RESPONSE_BLOCKS)
            {
                blocks    = (Block *)m_pixy->m_buf;
                numBlocks = m_pixy->m_length / sizeof(Block);
                return (int8_t)numBlocks;
            }
            else if (m_pixy->m_type == PIXY_TYPE_RESPONSE_ERROR)
            {
                if ((int8_t)m_pixy->m_buf[0] == PIXY_RESULT_BUSY)
                {
                    if (!wait)
                        return PIXY_RESULT_BUSY;
                }
                else if ((int8_t)m_pixy->m_buf[0] != PIXY_RESULT_PROG_CHANGING)
                    return m_pixy->m_buf[0];
            }
        }
        else
            return PIXY_RESULT_ERROR;

        // Back off to avoid thrashing the bus while Pixy2 is busy.
        delayMicroseconds(500);
    }
}

#endif // _PIXY2CCC_H
