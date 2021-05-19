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

#ifndef SWCLK_Pin
#define SWCLK_Pin         GPIO_PIN_14
#define SWCLK_GPIO_Port   GPIOA
#endif /* SWCLK_Pin */

#ifndef SWDIO_Pin
#define SWDIO_Pin         GPIO_PIN_13
#define SWDIO_GPIO_Port   GPIOA
#endif /* SWDIO_Pin */

#ifndef BASEBOARD
#define BASEBOARD         0
#endif /* BASEBOARD */

/* pin definitions for the Baseboard */
#define BASEBOARD_ENABLE_Pin              COM_GPIO2_Pin
#define BASEBOARD_ENABLE_GPIO_Port        COM_GPIO2_GPIO_Port
#define BASEBOARD_WAKE_Pin                COM_GPIO1_Pin
#define BASEBOARD_WAKE_GPIO_Port          COM_GPIO1_GPIO_Port
#define BASEBOARD_EXT3_SWITCH_Pin         COM_PROG2_Pin
#define BASEBOARD_EXT3_SWITCH_GPIO_Port   COM_PROG2_GPIO_Port
#define BASEBOARD_DEBUG_Pin               COM_PROG_Pin
#define BASEBOARD_DEBUG_GPIO_Port         COM_PROG_GPIO_Port

#define BASEBOARD_IS_ENABLED()            PIN_STATE(BASEBOARD_ENABLE)
#define BASEBOARD_ENABLE()                PIN_SET(BASEBOARD_ENABLE)
#define BASEBOARD_DISABLE()               PIN_CLR(BASEBOARD_ENABLE)


/* Macros */

#define PIN_TOGGLE(p)     HAL_GPIO_TogglePin(p##_GPIO_Port, p##_Pin)
#define PIN_SET(p)        p##_GPIO_Port->BSRR = p##_Pin           // HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_SET)
#define PIN_CLR(p)        p##_GPIO_Port->BRR = p##_Pin            // HAL_GPIO_WritePin(p##_GPIO_Port, p##_Pin, GPIO_PIN_RESET)
#define PIN_GET(p)        ((p##_GPIO_Port->IDR & p##_Pin) != 0)   // HAL_GPIO_ReadPin(p##_GPIO_Port, p##_Pin)      // works for input pins only
#define PIN_STATE(p)      ((p##_GPIO_Port->ODR & p##_Pin) != 0)   // read the output pin state
#define PIN_XOR(p)        PIN_TOGGLE(p)


/* Parameter checks */

#if SWO_ENABLE && BASEBOARD
#error "SWO_ENABLE and BASEBOARD cannot be enabled at the same time"
#endif


/* Function Prototypes */

void gpio_init(void);


#endif /* SYSTEM_GPIO_H_ */
