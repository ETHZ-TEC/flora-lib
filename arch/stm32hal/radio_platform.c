/*
 * radio_platform.c
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */


#include "arch/arch.h"


inline void radio_set_nss_pin() {
  HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET);
}

inline bool radio_read_busy_pin() {
  return HAL_GPIO_ReadPin(RADIO_BUSY_GPIO_Port, RADIO_BUSY_Pin) == GPIO_PIN_SET;
}

inline bool radio_read_dio1_pin() {
  return HAL_GPIO_ReadPin(RADIO_DIO1_GPIO_Port, RADIO_DIO1_Pin) == GPIO_PIN_SET;
}
