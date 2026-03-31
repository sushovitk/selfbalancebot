//
// Pixy2Video.h — video/RGB pixel query module
//
// Original: Charmed Labs LLC / CMU — GNU GPL v2
// STM32 port: no changes needed beyond those in TPixy2.h (delayMicroseconds
// is defined as a static inline in TPixy2.h before this file is included).
//

#ifndef _PIXY2VIDEO_H
#define _PIXY2VIDEO_H

#include <stdint.h>

#define VIDEO_REQUEST_GET_RGB   0x70

template <class LinkType> class TPixy2;

template <class LinkType> class Pixy2Video
{
public:
    Pixy2Video(TPixy2<LinkType> *pixy) : m_pixy(pixy) {}

    int8_t getRGB(uint16_t x, uint16_t y,
                  uint8_t *r, uint8_t *g, uint8_t *b,
                  bool saturate = true);

private:
    TPixy2<LinkType> *m_pixy;
};

template <class LinkType>
int8_t Pixy2Video<LinkType>::getRGB(uint16_t x, uint16_t y,
                                    uint8_t *r, uint8_t *g, uint8_t *b,
                                    bool saturate)
{
    while (1)
    {
        *(int16_t *)(m_pixy->m_bufPayload + 0) = (int16_t)x;
        *(int16_t *)(m_pixy->m_bufPayload + 2) = (int16_t)y;
        *(m_pixy->m_bufPayload + 4)             = (uint8_t)saturate;
        m_pixy->m_length = 5;
        m_pixy->m_type   = VIDEO_REQUEST_GET_RGB;

        m_pixy->sendPacket();
        if (m_pixy->recvPacket() == 0)
        {
            if (m_pixy->m_type   == PIXY_TYPE_RESPONSE_RESULT &&
                m_pixy->m_length == 4)
            {
                *b = *(m_pixy->m_buf + 0);
                *g = *(m_pixy->m_buf + 1);
                *r = *(m_pixy->m_buf + 2);
                return 0;
            }
            // Retry if program is changing
            else if (m_pixy->m_type == PIXY_TYPE_RESPONSE_ERROR &&
                     (int8_t)m_pixy->m_buf[0] == PIXY_RESULT_PROG_CHANGING)
            {
                delayMicroseconds(500);
                continue;
            }
        }
        return PIXY_RESULT_ERROR;
    }
}

#endif // _PIXY2VIDEO_H
