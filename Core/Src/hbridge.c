/* hbridge.c — H-bridge / motor driver module
 *
 * This file contains only the Motor_* driver functions.
 * Peripheral initialisation (TIM1, TIM2, GPIO) and all HAL
 * boilerplate (main, SystemClock_Config, Error_Handler) live
 * exclusively in main.c.
 */

#include "main.h"

/* htim1 is defined in main.c; reference it here with extern. */
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN 0 */

/**
 * @brief  Set motor speed and direction.
 * @param  duty     PWM duty cycle, 0-1000 (maps to 0-100% on ARR=999)
 * @param  forward  1 = forward, 0 = reverse
 */
void Motor_Set(uint16_t duty, uint8_t forward)
{
    if (duty > 1000) duty = 1000;

    if (forward)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }

    /* PA11 = TIM1 CH4 (AF1) — this is where ENA is wired */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, duty);
}

void Motor_Brake(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000);
}

void Motor_Coast(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
}

/* USER CODE END 0 */
