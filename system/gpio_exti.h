/*
 * gpio_exti.h
 *
 *  Created on: 04.11.2019
 *      Author: rtrueb
 */

#ifndef SYSTEM_GPIO_EXTI_H_
#define SYSTEM_GPIO_EXTI_H_

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void GPIO_Radio_Callback(void);
void GPIO_PIN_3_Callback(void);
void GPIO_SIG1_Callback(void);
void GPIO_SIG2_Callback(void);

#endif /* SYSTEM_GPIO_EXTI_H_ */
