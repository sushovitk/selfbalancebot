#pragma once
#include <stdint.h>
#include "main.h"
// RX[0] - 0xFF
// RX[1] - 0x41 / 0x73 - we want analog (0x73)
// RX[2] - 0x5a 
// RX[3] - button data 
// RX[4] - button data

// default: 0x7b (centered)
// top: 0x00
// bottom: 0xFF
// left: 0x00
// right: 0xFF
// RX[5] - Analog stick1 x (right stick) 
// RX[6] - Analog stick1 y
// RX[7] - analog stick2 x (left stick)
// RX[8] - analog stick2 y

// EX READOUT: 0xff, 0x73, 0x5a, 0xff, 0xff, 0x7b, 0x7b, 0x7b, 0x7b

// 0 is pressed, 1 is released
// Lookup table
// RX[3]
#define PS2_Select   	~0b00000001 
#define PS2_L3 		    ~0b00000010
#define PS2_R3 		    ~0b00000100
#define PS2_Start   	~0b00001000
#define PS2_Up		    ~0b00010000
#define PS2_Right    	~0b00100000
#define PS2_Down        ~0b01000000
#define PS2_Left	    ~0b10000000

// RX[4]
#define PS2_L2 	    	~0b00000001 
#define PS2_R2 	    	~0b00000010
#define PS2_L1 		    ~0b00000100
#define PS2_R1 		    ~0b00001000
#define PS2_Triangle    ~0b00010000
#define PS2_O		    ~0b00100000
#define PS2_X		    ~0b01000000
#define PS2_Square  	~0b10000000

extern SPI_HandleTypeDef hspi3;

void read_ps2(uint8_t *x, uint8_t *y);
