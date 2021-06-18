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

volatile uint16_t FLOCKLAB_NODE_ID = 0xbeef;    // any value is ok, will be binary patched with respective node ID by FlockLab, see https://gitlab.ethz.ch/tec/public/flocklab/wiki/-/wikis/Man/HowTo#how-to-assign-node-ids

#if FLOCKLAB

/*
 * init GPIOs
 */
void flocklab_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* --- OUTPUTS --- */

  HAL_GPIO_DeInit(FLOCKLAB_INT1_GPIO_Port, FLOCKLAB_INT1_Pin);
  HAL_GPIO_WritePin(FLOCKLAB_INT1_GPIO_Port, FLOCKLAB_INT1_Pin, GPIO_PIN_RESET);
#if FLOCKLAB_SWD
  // SWCLK and SWDIO are connected to INT2 and LED3 -> cannot be used if SWD is enabled
  GPIO_InitStruct.Pin = FLOCKLAB_INT1_Pin;
#else
  HAL_GPIO_DeInit(FLOCKLAB_INT2_GPIO_Port, FLOCKLAB_INT2_Pin);
  HAL_GPIO_DeInit(FLOCKLAB_LED3_GPIO_Port, FLOCKLAB_LED3_Pin);
  HAL_GPIO_WritePin(FLOCKLAB_INT2_GPIO_Port, FLOCKLAB_INT2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FLOCKLAB_LED3_GPIO_Port, FLOCKLAB_LED3_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = FLOCKLAB_INT1_Pin | FLOCKLAB_INT2_Pin | FLOCKLAB_LED3_Pin;
#endif /* FLOCKLAB_SWD */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(FLOCKLAB_INT1_GPIO_Port, &GPIO_InitStruct);
#if !SWO_ENABLE
  /* SWO pin is shared with LED2 -> if not used as SWO, then use it for tracing */
  HAL_GPIO_DeInit(FLOCKLAB_LED2_GPIO_Port, FLOCKLAB_LED2_Pin);    /* required, otherwise high current drain in LPM can occur */
  HAL_GPIO_WritePin(FLOCKLAB_LED2_GPIO_Port, FLOCKLAB_LED2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = FLOCKLAB_LED2_Pin;
  HAL_GPIO_Init(FLOCKLAB_LED2_GPIO_Port, &GPIO_InitStruct);
#endif /* SWO_ENABLE */

  /* --- INPUTS --- */

  HAL_GPIO_DeInit(FLOCKLAB_SIG1_GPIO_Port, FLOCKLAB_SIG1_Pin);
  HAL_GPIO_DeInit(FLOCKLAB_SIG2_GPIO_Port, FLOCKLAB_SIG2_Pin);
  GPIO_InitStruct.Pin = FLOCKLAB_SIG1_Pin | FLOCKLAB_SIG2_Pin;
#if FLOCKLAB_SIG_INT
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
#else /* FLOCKLAB_SIG_INT */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#endif /* FLOCKLAB_SIG_INT */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(FLOCKLAB_SIG1_GPIO_Port, &GPIO_InitStruct);

  // LED1 is connected to RF_DIO1 on COM boards on FlockLab -> configure it as input
  HAL_GPIO_DeInit(FLOCKLAB_LED1_GPIO_Port, FLOCKLAB_LED1_Pin);
  GPIO_InitStruct.Pin = FLOCKLAB_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLOCKLAB_LED1_GPIO_Port, &GPIO_InitStruct);

  // BOLT ACK is floating on flocklab, needs a pulldown
  HAL_GPIO_DeInit(BOLT_ACK_GPIO_Port, BOLT_ACK_Pin);
  GPIO_InitStruct.Pin = BOLT_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOLT_ACK_GPIO_Port, &GPIO_InitStruct);

  /* --- INTERRUPTS --- */
#if FLOCKLAB_SIG_INT
  HAL_NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
#endif /* FLOCKLAB_SIG_INT */
  // make sure the time request interrupt on COM_TREQ (FLOCKLAB_LED2_Pin) is disabled
  HAL_NVIC_DisableIRQ(EXTI3_IRQn);
}

/*
 * set all FlockLab pins back to zero
 */
void flocklab_reset_pins(void)
{
  FLOCKLAB_PIN_CLR(FLOCKLAB_INT1);
#if !FLOCKLAB_SWD
  FLOCKLAB_PIN_CLR(FLOCKLAB_LED3);
  FLOCKLAB_PIN_CLR(FLOCKLAB_LED1);
#endif /* !FLOCKLAB_SWD */
#if !SWO_ENABLE
  FLOCKLAB_PIN_CLR(FLOCKLAB_LED2);
#endif /* SWO_ENABLE */
}

/*
 * set a FlockLab pin to 1 and right back to 0 for count times
 * pin: one of the pins defined in flocklab.h
 * count: number of flashes
 */
void flocklab_blink(flocklab_trace_pin_t pin, uint8_t count)
{
  switch (pin) {
    case FLOCKLAB_INT1:
      for (uint32_t i = 0; i < count; i++) {
        FLOCKLAB_PIN_SET(FLOCKLAB_INT1);
        FLOCKLAB_PIN_CLR(FLOCKLAB_INT1);
      }
      break;
#if !FLOCKLAB_SWD
    case FLOCKLAB_LED3:
      for (uint32_t i = 0; i < count; i++) {
        FLOCKLAB_PIN_SET(FLOCKLAB_LED3);
        FLOCKLAB_PIN_CLR(FLOCKLAB_LED3);
      }
      break;
    case FLOCKLAB_INT2:
      for (uint32_t i = 0; i < count; i++) {
        FLOCKLAB_PIN_SET(FLOCKLAB_INT2);
        FLOCKLAB_PIN_CLR(FLOCKLAB_INT2);
      }
      break;
#endif /* !FLOCKLAB_SWD */
  }
}

/*
 * get the FlockLab node ID
 */
uint16_t flocklab_node_id(void)
{
  const uint32_t dev_id[] = {
      0x62003e, 0x2b0059, 0x220039, 0x570026, 0x68003e, 0x41003c, 0x290059, 0x460027,   /* observers  1 -  8 */
      0x230040, 0x380040, 0x310060, 0x370025, 0x270040, 0,        0x520040, 0x2b002a,   /* observers  9 - 16 */
      0x43003a, 0,        0x57002c, 0x550029, 0x5b0060, 0x220060, 0x2b003c, 0x2c0070,   /* observers 17 - 24 */
      0x67004b, 0x270060, 0x3c0040, 0x250040, 0x590026, 0x420035, 0x18003f, 0x4c004a    /* observers 25 - 32 */
  };
  uint32_t i;
  for (i = 0; i < sizeof(dev_id) / sizeof(uint32_t); i++) {
    if (dev_id[i] == REGVAL32(DEVICE_ID_REG)) {
      return i + 1;
    }
  }
  return 0;
}

#endif /* FLOCKLAB */
