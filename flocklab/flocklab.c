/*
 * flocklab.c
 *
 *  Created on: Feb 12, 2019
 *      Author: kelmicha
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

  HAL_GPIO_WritePin(FLOCKLAB_INT1_GPIO_Port, FLOCKLAB_INT1_Pin, GPIO_PIN_RESET);
#if FLOCKLAB_SWD
  // SWCLK and SWDIO are connected to INT2 and LED3 -> cannot be used if SWD is enabled
  GPIO_InitStruct.Pin = FLOCKLAB_INT1_Pin;
#else
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
  HAL_GPIO_WritePin(FLOCKLAB_LED2_GPIO_Port, FLOCKLAB_LED2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = FLOCKLAB_LED2_Pin;
  HAL_GPIO_Init(FLOCKLAB_LED2_GPIO_Port, &GPIO_InitStruct);
#endif /* SWO_ENABLE */

  /* --- INPUTS --- */

  GPIO_InitStruct.Pin = FLOCKLAB_SIG1_Pin | FLOCKLAB_SIG2_Pin;
#if FLOCKLAB_SIG_INT
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
#else /* FLOCKLAB_SIG_INT */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
#endif /* FLOCKLAB_SIG_INT */
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(FLOCKLAB_SIG1_GPIO_Port, &GPIO_InitStruct);

  // LED1 is connected to RF_DIO1 on COM boards on FlockLab -> configure it as input
  GPIO_InitStruct.Pin = FLOCKLAB_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FLOCKLAB_LED1_GPIO_Port, &GPIO_InitStruct);

  // BOLT ACK is floating on flocklab, needs a pulldown
  GPIO_InitStruct.Pin = BOLT_ACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BOLT_ACK_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLOCKLAB_SIG1_Pin;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(FLOCKLAB_SIG1_GPIO_Port, &GPIO_InitStruct);

  /* --- INTERRUPTS --- */
#if FLOCKLAB_SIG_INT
  HAL_NVIC_SetPriority(EXTI0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
#endif /* FLOCKLAB_SIG_INT */
  // make sure the time request interrupt on COM_TREQ is disabled
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


#endif /* FLOCKLAB */
