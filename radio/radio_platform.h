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

#ifndef RADIO_RADIO_PLATFORM_H_
#define RADIO_RADIO_PLATFORM_H_


// Platform specific commands

#define RADIO_SET_NRESET_PIN()    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_SET)       // release reset
#define RADIO_CLR_NRESET_PIN()    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_RESET)     // activate reset

#define RADIO_READ_NSS_PIN()      (RADIO_NSS_GPIO_Port->ODR & (uint32_t)RADIO_NSS_Pin)
#define RADIO_SET_NSS_PIN()       RADIO_NSS_GPIO_Port->BSRR = (uint32_t)RADIO_NSS_Pin
#define RADIO_CLR_NSS_PIN()       RADIO_NSS_GPIO_Port->BRR = (uint32_t)RADIO_NSS_Pin

#define RADIO_READ_BUSY_PIN()     (HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET)
#define RADIO_READ_DIO1_PIN()     (HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == GPIO_PIN_SET)


#ifndef RADIO_TX_START_IND
  #define RADIO_TX_START_IND()
  #define RADIO_TX_STOP_IND()
#endif /* RADIO_TX_START_IND */
#ifndef RADIO_RX_START_IND
  #define RADIO_RX_START_IND()
  #define RADIO_RX_STOP_IND()
#endif /* RADIO_RX_START_IND */


#endif /* RADIO_RADIO_PLATFORM_H_ */
