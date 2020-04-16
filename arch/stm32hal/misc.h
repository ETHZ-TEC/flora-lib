/*
 * misc.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef ARCH_STM32HAL_MISC_H_
#define ARCH_STM32HAL_MISC_H_


#include "stdint.h"


#define IS_INTERRUPT()  ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)


void delay(volatile uint32_t loop_passes);
void delay_us(volatile uint32_t us);


#endif /* ARCH_STM32HAL_MISC_H_ */
