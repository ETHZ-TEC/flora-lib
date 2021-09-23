/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SYSTEM_GPIO_H_
#define SYSTEM_GPIO_H_


/* Defines */

#ifndef SWO_ENABLE
#define SWO_ENABLE        1       /* enabled by default */
#endif /* SWO_ENABLE */

#ifndef SWO_Pin
#define SWO_Pin           GPIO_PIN_3
#define SWO_GPIO_Port     GPIOB
#endif /* SWO_PIN */

#ifndef SWCLK_Pin
#define SWCLK_Pin         GPIO_PIN_14
#define SWCLK_GPIO_Port   GPIOA
#endif /* SWCLK_Pin */

#ifndef SWDIO_Pin
#define SWDIO_Pin         GPIO_PIN_13
#define SWDIO_GPIO_Port   GPIOA
#endif /* SWDIO_Pin */

#ifdef BASEBOARD
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
#endif /* BASEBOARD */

#ifndef BASEBOARD
#define BASEBOARD         0
#endif /* BASEBOARD */

#if BASEBOARD && SWO_ENABLE
#error "SWO cannot be used with BASEBOARD"
#endif


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
void gpio_init_swd(void);
void gpio_init_swo(void);
void gpio_deinit_swd(void);
void gpio_deinit_swo(void);
void gpio_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool output);


#endif /* SYSTEM_GPIO_H_ */
