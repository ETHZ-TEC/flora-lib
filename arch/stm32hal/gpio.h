/*
 * gpio.h
 *
 *  Created on: Apr 16, 2020
 *      Author: rdaforno
 */

#ifndef ARCH_STM32HAL_GPIO_H_
#define ARCH_STM32HAL_GPIO_H_


/* Includes */

#include "main.h"


/* Defines */

#ifndef SWO_ENABLE
#define SWO_ENABLE        1       /* enabled by default */
#endif /* SWO_ENABLE */

#ifndef SWO_Pin
#define SWO_Pin           COM_GPIO2_Pin
#define SWO_GPIO_Port     COM_GPIO2_GPIO_Port
#endif /* SWO_PIN */


/* Macros */

#define PIN_TOGGLE(p)     HAL_GPIO_TogglePin(p##_GPIO_Port, p##_Pin)
#define PIN_SET(p)        HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_SET)
#define PIN_CLR(p)        HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_RESET)
#define PIN_GET(p)        HAL_GPIO_ReadPin(p##_GPIO_Port, p##_Pin)
#define PIN_XOR(p)        PIN_TOGGLE(p)


/* Function Prototypes */

void gpio_init(void);


#endif /* ARCH_STM32HAL_GPIO_H_ */
