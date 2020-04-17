/*
 * misc.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef UTILS_MISC_H_
#define UTILS_MISC_H_


#define IS_INTERRUPT()  ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0)


void delay(volatile uint32_t loop_passes);
void delay_us(volatile uint32_t us);

bool swo_println(const char* str);
bool swo_print(const char* str);

#endif /* UTILS_MISC_H_ */
