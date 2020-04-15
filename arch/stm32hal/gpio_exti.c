/*
 * gpio_exti.c
 *
 *  Created on: 04.11.2019
 *      Author: rtrueb
 */

#include "main.h"
#include "arch/stm32hal/gpio_exti.h"
#include "stm32l4xx_hal.h"

/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    /* RADIO_DIO1 interrupt triggered (PC13) */
    GPIO_Radio_Callback();
  }
  else if (GPIO_Pin == GPIO_PIN_3)
  {
    GPIO_PIN_3_Callback();
  }

#if FLOCKLAB
  if (GPIO_Pin == FLOCKLAB_SIG1_Pin)
  {
    GPIO_SIG1_Callback();
  }
  else if (GPIO_Pin == FLOCKLAB_SIG2_Pin)
  {
    GPIO_SIG2_Callback();
  }
#endif /* FLOCKLAB */
}

__weak void GPIO_PIN_3_Callback(void) 
{
  // NOP
}

__weak void GPIO_Radio_Callback(void)
{
  // NOP
}

__weak void GPIO_SIG1_Callback(void)
{
  // NOP
}

__weak void GPIO_SIG2_Callback(void)
{
  // NOP
}
