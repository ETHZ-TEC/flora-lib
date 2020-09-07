/*
 * gpio.c
 *
 *  Created on: Apr 16, 2020
 *      Author: rdaforno
 */

#include "flora_lib.h"



void gpio_check_baseboard(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* store pin config */
  uint32_t prev_mode   = BASEBOARD_ENABLE_GPIO_Port->MODER;
  uint32_t prev_pull   = BASEBOARD_ENABLE_GPIO_Port->PUPDR;

  /* to check whether the comboard is indeed not on a baseboard: read state of enable pin (has external pullup) */
  GPIO_InitStruct.Pin   = BASEBOARD_ENABLE_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  HAL_GPIO_Init(BASEBOARD_ENABLE_GPIO_Port, &GPIO_InitStruct);
  /* read pin value */
  uint32_t c = 10;
  while (c) {
    delay_us(1000);
    if (PIN_GET(BASEBOARD_ENABLE) == 0) {
      break;
    }
    c--;
  }
  if (c == 0) {
    /* comboard is most likely installed on a baseboard! */
    LOG_ERROR("baseboard detected, but compiled without flag 'BASEBOARD'!");
  #if !LOG_PRINT_IMMEDIATELY
    log_flush();
  #endif /* LOG_PRINT_IMMEDIATELY */
    led_on(LED_EVENT);
    while (1);
  }

  /* restore previous pin config */
  BASEBOARD_ENABLE_GPIO_Port->PUPDR  = prev_pull;
  BASEBOARD_ENABLE_GPIO_Port->MODER  = prev_mode;
  /* make sure no interrupt is triggered by reconfiguring the pin */
  __HAL_GPIO_EXTI_CLEAR_IT(BASEBOARD_ENABLE_Pin);
}


void gpio_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  (void)GPIO_InitStruct;

#if !SWO_ENABLE
  /* SWO is not used */
 #if !BASEBOARD && !FLOCKLAB
  gpio_check_baseboard();
 #endif /* BASEBOARD */
#endif /* SWO_ENABLE */

#if BASEBOARD
  /* make sure the baseboard enable pin is high */
  HAL_GPIO_WritePin(BASEBOARD_ENABLE_GPIO_Port, BASEBOARD_ENABLE_Pin, GPIO_PIN_SET);
  /* configure COM_PROG and COM_PROG2 as output */
  HAL_GPIO_WritePin(BASEBOARD_EXT3_SWITCH_GPIO_Port, BASEBOARD_EXT3_SWITCH_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = BASEBOARD_DEBUG_Pin | BASEBOARD_EXT3_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASEBOARD_EXT3_SWITCH_GPIO_Port, &GPIO_InitStruct);
#endif /* BASEBOARD */
}

