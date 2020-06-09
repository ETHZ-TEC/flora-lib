/*
 * radio_platform.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef RADIO_RADIO_PLATFORM_H_
#define RADIO_RADIO_PLATFORM_H_

// Platform specific commands

#define RADIO_SET_NRESET_PIN()    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_SET)       // release reset
#define RADIO_CLR_NRESET_PIN()    HAL_GPIO_WritePin(RADIO_NRESET_GPIO_Port, RADIO_NRESET_Pin, GPIO_PIN_RESET)     // activate reset

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
