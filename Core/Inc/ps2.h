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

// 0 is pressed, 1 is released
// Lookup table
// RX[3]
#define Select   	~0b00000001 
#define L3 		    ~0b00000010
#define L3 		    ~0b00000100
#define Start   	~0b00001000
#define Up		    ~0b00010000
#define Right    	~0b00100000
#define Down        ~0b01000000
#define Left	    ~0b10000000

// RX[4]
#define L2 	    	~0b00000001 
#define R2 	    	~0b00000010
#define L1 		    ~0b00000100
#define R1 		    ~0b00001000
#define Triangle    ~0b00010000
#define O		    ~0b00100000
#define X		    ~0b01000000
#define Square  	~0b10000000

void read_ps2(){
    // initiate transaction with PS2 controller
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
	HAL_SPI_TransmitReceive(&hspi1, TX, RX, 8, 10);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);

    // read RX


}