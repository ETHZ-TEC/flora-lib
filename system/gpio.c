/*
 * gpio.c
 *
 *  Created on: Apr 16, 2020
 *      Author: rdaforno
 */

#include "flora_lib.h"


void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

#if !SWO_ENABLE
  /* SWO is not used -> configure as output */
 #if BASEBOARD
  /* configure for Baseboard: set as output high */
  HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 #else  /* BASEBOARD */
  HAL_GPIO_WritePin(SWO_GPIO_Port, SWO_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
 #endif /* BASEBOARD */
  GPIO_InitStruct.Pin   = SWO_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(SWO_GPIO_Port, &GPIO_InitStruct);
#endif /* SWO_ENABLE */

#if BASEBOARD
  /* configure COM_PROG and COM_PROG2 as output */
  HAL_GPIO_WritePin(BASEBOARD_EXT3_SWITCH_GPIO_Port, BASEBOARD_EXT3_SWITCH_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = BASEBOARD_DEBUG_Pin | BASEBOARD_EXT3_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASEBOARD_EXT3_SWITCH_GPIO_Port, &GPIO_InitStruct);
#endif /* BASEBOARD */

  (void)GPIO_InitStruct;
}

