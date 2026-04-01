#include "p2s.h"

uint8_t PS2_RX[9];
uint8_t PS2_TX[9] = { 0x01, 0x42};

void read_ps2(uint8_t *x, uint8_t *y){

    // initiate transaction with PS2 controller
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
	HAL_SPI_TransmitReceive(&hspi3, PS2_TX, PS2_RX, 9, 10);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);

    // read RX
    if(PS2_RX[5] != 0x7b || PS2_RX[6] != 0x7b){
        *x = PS2_RX[5];
        *y = PS2_RX[6];
    }
}
