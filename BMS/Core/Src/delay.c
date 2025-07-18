/*
 * delay.c
 *
 *  Created on: Jul 19, 2025
 *      Author: RMG
 */


#include "delay.h"

extern TIM_HandleTypeDef htim1;

void Delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}
