// RX[0] - 0xFF
// RX[1] - 0x41 / 0x73 - we want analog (0x73)
// RX[2] - button data 
// RX[3] - button data
// RX[4] - Analog stick1 x
// RX[5] - Analog stick1 y
// RX[6] - analog stick2 x
// RX[7] - analog stick2 y

// 0 is pressed, 1 is released
// Lookup table
// RX[2]
#define Select   	~0b00000001 
#define L3 		    ~0b00000010
#define L3 		    ~0b00000100
#define Start   	~0b00001000
#define Up		    ~0b00010000
#define Right    	~0b00100000
#define Down  	  ~0b01000000
#define Left		  ~0b10000000

// RX[3]
#define L2 	    	~0b00000001 
#define R2 	    	~0b00000010
#define L1 		    ~0b00000100
#define R1 		    ~0b00001000
#define Triangle  ~0b00010000
#define O		      ~0b00100000
#define X		      ~0b01000000
#define Square  	~0b10000000
