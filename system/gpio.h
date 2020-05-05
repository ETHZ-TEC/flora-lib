/*
 * gpio.h
 *
 *  Created on: Apr 16, 2020
 *      Author: rdaforno
 */

#ifndef SYSTEM_GPIO_H_
#define SYSTEM_GPIO_H_


/* Defines */

#ifndef SWO_ENABLE
#define SWO_ENABLE        1       /* enabled by default */
#endif /* SWO_ENABLE */

#ifndef SWO_Pin
#define SWO_Pin           COM_GPIO2_Pin
#define SWO_GPIO_Port     COM_GPIO2_GPIO_Port
#endif /* SWO_PIN */

#ifndef BASEBOARD
#define BASEBOARD         0
#endif /* BASEBOARD */

/* pin definitions for the Baseboard */
#define BASEBOARD_ENABLE_Pin              COM_GPIO2_Pin
#define BASEBOARD_ENABLE_GPIO_Port        COM_GPIO2_GPIO_Port
#define BASEBOARD_WAKE_Pin                COM_GPIO1_Pin
#define BASEBOARD_WAKE_GPIO_Port          COM_GPIO1_GPIO_Port
#define BASEBOARD_VEXT3_SWITCH_Pin        COM_PROG2_Pin
#define BASEBOARD_VEXT3_SWITCH_GPIO_Port  COM_PROG2_GPIO_Port
#define BASEBOARD_DEBUG_Pin               COM_PROG_Pin
#define BASEBOARD_DEBUG_GPIO_Port         COM_PROG_GPIO_Port


/* Macros */

#define PIN_TOGGLE(p)     HAL_GPIO_TogglePin(p##_GPIO_Port, p##_Pin)
#define PIN_SET(p)        HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_SET)
#define PIN_CLR(p)        HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_RESET)
#define PIN_GET(p)        HAL_GPIO_ReadPin(p##_GPIO_Port, p##_Pin)
#define PIN_XOR(p)        PIN_TOGGLE(p)


/* Function Prototypes */

void gpio_init(void);


#endif /* SYSTEM_GPIO_H_ */
