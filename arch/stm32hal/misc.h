/*
 * misc.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef ARCH_STM32HAL_MISC_H_
#define ARCH_STM32HAL_MISC_H_


#include "stdint.h"


#ifndef MIN
#define MIN(x, y)     (((x) < (y)) ? (x) : (y))
#endif
#ifndef MAX
#define MAX(x, y)     (((x) > (y)) ? (x) : (y))
#endif


#define PIN_TOGGLE(p) HAL_GPIO_TogglePin(p##_GPIO_Port, p##_Pin)
#define PIN_SET(p)    HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_SET)
#define PIN_CLR(p)    HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_RESET)
#define PIN_GET(p)    HAL_GPIO_ReadPin(p##_GPIO_Port, p##_Pin)
#define PIN_XOR(p)    PIN_TOGGLE(p)

#define IS_INTERRUPT()  ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)


void delay(volatile uint32_t loop_passes);
void delay_us(volatile uint32_t us);


#endif /* ARCH_STM32HAL_MISC_H_ */
