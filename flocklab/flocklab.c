/*
 * flocklab.c
 *
 *  Created on: Feb 12, 2019
 *      Author: kelmicha
 */

#include "flora_lib.h"

volatile uint16_t FLOCKLAB_NODE_ID = 0xbeef;    // any value is ok, will be binary patched with respective node ID by FlockLab, see https://gitlab.ethz.ch/tec/public/flocklab/flocklab/wikis/Man/HowTo

#if FLOCKLAB

// /* GPIO interrupt callback (called from GPIO interrupt HAL_GPIO_EXTI_Callback implemeneted in gpio_exti.h) */
// void GPIO_SIG1_Callback(void) {
//   // NOP
// }
//
// /* GPIO interrupt callback (called from GPIO interrupt HAL_GPIO_EXTI_Callback implemeneted in gpio_exti.h) */
// void GPIO_SIG2_Callback(void) {
//   // NOP
// }

/*
 * init GPIOs
 */
void flocklab_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* outputs */
  HAL_GPIO_WritePin(FLOCKLAB_LED1_GPIO_Port, FLOCKLAB_LED1_Pin, GPIO_PIN_RESET);
#if !FLOCKLAB_SWD
  // SWCLK and SWDIO are connected to LED1 and LED3 -> those pins cannot be used if SWD shall be used
  HAL_GPIO_WritePin(FLOCKLAB_LED3_GPIO_Port, FLOCKLAB_LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FLOCKLAB_INT1_GPIO_Port, FLOCKLAB_INT1_Pin, GPIO_PIN_RESET);
#endif /* !FLOCKLAB_SWD */

#if FLOCKLAB_SWD
  GPIO_InitStruct.Pin = FLOCKLAB_INT1_Pin;
#else
  GPIO_InitStruct.Pin = FLOCKLAB_INT1_Pin | FLOCKLAB_LED3_Pin | FLOCKLAB_LED1_Pin;
#endif /* FLOCKLAB_SWD */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* input */
  GPIO_InitStruct.Pin = FLOCKLAB_SIG1_Pin | FLOCKLAB_SIG2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // LED2 is connected to RF_DIO1 on COM boards on FlockLab -> configure it as input to not drive LED2 signal from MCU as well
  GPIO_InitStruct.Pin = FLOCKLAB_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
      case FLOCKLAB_LED1:
      for (uint32_t i = 0; i < count; i++) {
        FLOCKLAB_PIN_SET(FLOCKLAB_LED1);
        FLOCKLAB_PIN_CLR(FLOCKLAB_LED1);
      }
      break;
#endif /* !FLOCKLAB_SWD */
  }
}


#endif /* FLOCKLAB */