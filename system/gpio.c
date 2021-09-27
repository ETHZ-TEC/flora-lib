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

#include "flora_lib.h"


void gpio_check_baseboard(void)
{
#ifdef BASEBOARD_ENABLE_Pin

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* store pin config */
  uint32_t prev_mode   = BASEBOARD_ENABLE_GPIO_Port->MODER;
  uint32_t prev_pull   = BASEBOARD_ENABLE_GPIO_Port->PUPDR;

  /* to check whether the comboard is indeed not on a baseboard: read state of enable pin (has external pullup) */
  GPIO_InitStruct.Pin  = BASEBOARD_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;      // first init with pull to remove potential charge on the pin
  HAL_GPIO_Init(BASEBOARD_ENABLE_GPIO_Port, &GPIO_InitStruct);
  delay_us(100);
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
    const char* error_msg = "baseboard detected, but compiled without flag 'BASEBOARD'!" LOG_NEWLINE;
    LOG_PRINT_FUNC((char*)error_msg, strlen(error_msg));
    led_on(LED_EVENT);
    delay_us(10000000);
    NVIC_SystemReset();
  }

  /* restore previous pin config */
  BASEBOARD_ENABLE_GPIO_Port->PUPDR = prev_pull;
  BASEBOARD_ENABLE_GPIO_Port->MODER = prev_mode;
  /* make sure no interrupt is triggered by reconfiguring the pin */
  __HAL_GPIO_EXTI_CLEAR_IT(BASEBOARD_ENABLE_Pin);

#endif /* BASEBOARD_ENABLE_Pin */
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
  HAL_GPIO_DeInit(BASEBOARD_ENABLE_GPIO_Port, BASEBOARD_ENABLE_Pin);
  HAL_GPIO_WritePin(BASEBOARD_ENABLE_GPIO_Port, BASEBOARD_ENABLE_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin   = BASEBOARD_ENABLE_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASEBOARD_ENABLE_GPIO_Port, &GPIO_InitStruct);
  /* configure COM_PROG and COM_PROG2 as output */
  HAL_GPIO_DeInit(BASEBOARD_EXT3_SWITCH_GPIO_Port, BASEBOARD_EXT3_SWITCH_Pin);
  HAL_GPIO_DeInit(BASEBOARD_DEBUG_GPIO_Port, BASEBOARD_DEBUG_Pin);
  HAL_GPIO_WritePin(BASEBOARD_EXT3_SWITCH_GPIO_Port, BASEBOARD_EXT3_SWITCH_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin   = BASEBOARD_DEBUG_Pin | BASEBOARD_EXT3_SWITCH_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BASEBOARD_EXT3_SWITCH_GPIO_Port, &GPIO_InitStruct);
#endif /* BASEBOARD */
}


/* init SWD interface pins */
void gpio_init_swd(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(SWDIO_GPIO_Port, SWDIO_Pin | SWCLK_Pin);
  GPIO_InitStruct.Pin       = SWDIO_Pin | SWCLK_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);
}


/* init SWO pin */
void gpio_init_swo(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(SWO_GPIO_Port, SWO_Pin);
  GPIO_InitStruct.Pin       = SWO_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF0_SWJ;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SWO_GPIO_Port, &GPIO_InitStruct);
}


/* de-init SWD pins */
void gpio_deinit_swo(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(SWO_GPIO_Port, SWO_Pin);
  GPIO_InitStruct.Pin   = SWO_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWO_GPIO_Port, &GPIO_InitStruct);
}


/* de-init SWO pin */
void gpio_deinit_swd(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(SWDIO_GPIO_Port, SWDIO_Pin | SWCLK_Pin);
  GPIO_InitStruct.Pin   = SWDIO_Pin | SWCLK_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SWDIO_GPIO_Port, &GPIO_InitStruct);
}


/* configure a pin in GPIO mode (input or output) */
void gpio_config(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, bool output)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = (output ? GPIO_MODE_OUTPUT_PP : GPIO_MODE_INPUT);
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
