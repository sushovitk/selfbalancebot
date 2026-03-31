//
// Pixy2SPI_SS.h — STM32 HAL port of the Pixy2 SPI-with-slave-select link.
//
// Replaces the original Arduino version which used Arduino SPI.*,
// pinMode(), and digitalWrite().  This version uses STM32 HAL exclusively.
//
// Original Pixy2 library:
//   Copyright (c) Charmed Labs LLC / CMU — GNU GPL v2
// STM32 port modifications:
//   All Arduino API calls replaced with STM32 HAL equivalents.
//

#ifndef _PIXY2SPI_SS_H
#define _PIXY2SPI_SS_H

#include "TPixy2.h"           // template base — defines the Pixy2 packet protocol
#include "stm32l4xx_hal.h"    // HAL_SPI_TransmitReceive, HAL_GPIO_WritePin, etc.

// ---------------------------------------------------------------------------
//  Link2SPI_SS — STM32 HAL implementation of the Pixy2 SPI link interface
//
//  Protocol behaviour (matches the original Arduino Link2SPI_SS exactly):
//
//  send(buf, len):
//    CS low  → transmit all request bytes (discard simultaneous MISO data)
//    CS high  ← rising edge signals "request complete" to the Pixy2 slave
//
//  recv(buf, len):
//    CS low  → clock out 0x00 dummy bytes, capture MISO into buf → CS high
//    Each recv() call is its own CS assertion (same as Arduino).
//    TPixy2::getSync() calls recv(&c, 1) in a tight loop, so CS toggles once
//    per byte during response polling — this matches tested Pixy2 behaviour.
//
//  USAGE (from pixy2.cpp):
//    static TPixy2<Link2SPI_SS> pixyLib;
//    pixyLib.m_link.configure(&hspi1, GPIOD, GPIO_PIN_14);
//    pixyLib.init();   // internally calls open() then polls getVersion()
// ---------------------------------------------------------------------------
class Link2SPI_SS
{
public:
	// Must be called before init() to supply the STM32 SPI peripheral and the
	// GPIO port/pin used for the manual chip-select line.
	// The SPI peripheral itself is initialised by MX_SPI1_Init() in main.c;
	// this function just stores the handles for later use.
	void configure(SPI_HandleTypeDef *hspi, GPIO_TypeDef *csPort, uint16_t csPin)
	{
		m_hspi = hspi;
		m_csPort = csPort;
		m_csPin = csPin;
	}

	// Called by TPixy2::init().  On STM32 the SPI peripheral is already
	// running (MX_SPI1_Init did it), so we just park CS high.
	// arg is unused (on Arduino it was the SS pin number).
	int8_t open(uint32_t arg)
	{
		(void)arg;
		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
		return 0;
	}

	void close(void)
	{
		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
	}

	// Receive len bytes from the Pixy2.  Transmits 0x00 dummy bytes to clock
	// the slave's shift register.  Optionally accumulates a checksum in *cs.
	// Each call manages its own CS assertion (CS low → bytes → CS high).
	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL)
	{
		uint8_t tx = 0x00;
		if (cs) *cs = 0;

		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
		for (uint8_t i = 0; i < len; i++)
		{
			HAL_SPI_TransmitReceive(m_hspi, &tx, &buf[i], 1, 500U);
			if (cs) *cs += buf[i];
		}
		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);

		return (int16_t)len;
	}

	// Send len bytes to the Pixy2 in a single CS assertion.
	// Received bytes during transmission are discarded (Pixy2 output is
	// undefined while it is receiving a request packet).
	int16_t send(uint8_t *buf, uint8_t len)
	{
		uint8_t dummy;

		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
		for (uint8_t i = 0; i < len; i++)
		HAL_SPI_TransmitReceive(m_hspi, &buf[i], &dummy, 1, 500U);
		HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);

		return (int16_t)len;
	}

	void setArg(uint16_t arg) {(void)arg;}

private:
	SPI_HandleTypeDef *m_hspi = NULL;
	GPIO_TypeDef *m_csPort = NULL;
	uint16_t m_csPin = 0;
};

// Convenience typedef matching the Arduino library name.
typedef TPixy2<Link2SPI_SS> Pixy2SPI_SS;

#endif // _PIXY2SPI_SS_H
