/*
 * misc.c
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#include "flora_lib.h"


#ifdef HAL_RNG_MODULE_ENABLED
extern RNG_HandleTypeDef hrng;
#endif /* HAL_RNG_MODULE_ENABLED */


void delay(volatile uint32_t loop_passes)
{
  while (loop_passes) loop_passes--;
}

void delay_us(volatile uint32_t us)
{
  /* note: needs to be adjusted if the CPU clock is changed! */
  while (us) {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    us--;
  }
}

#if SWO_ENABLE

bool swo_println(const char* str)
{
  if (!str) {
    return false;
  }
  while (*str)
  {
    ITM_SendChar(*str);
    str++;
  }
  ITM_SendChar('\r');
  ITM_SendChar('\n');

  return true;
}

bool swo_print(const char* str, uint32_t len)
{
  if (!str) {
    return false;
  }
  if (len == 0) {
    len = strlen(str);
  }
  while (len)
  {
    ITM_SendChar(*str);
    str++;
    len--;
  }
  return true;
}

int _write(int32_t file, uint8_t *ptr, int32_t len)
{
  /* Implement your write code here, this is used by puts and printf for example */
  /* return len; */
  uint32_t i;
  for (i = 0; i < len; i++)
  {
    ITM_SendChar(ptr[i]);
  }
  return len;
}

#endif /* SWO_ENABLE */


void random_init(void)
{
  // seed the random number generator
#ifndef HAL_RNG_MODULE_ENABLED
  srand(hs_timer_get_counter() + NODE_ID);
#endif /* HAL_RNG_MODULE_ENABLED */
}


uint32_t random_rand16(void)
{
  // seed the random number generator
#ifdef HAL_RNG_MODULE_ENABLED
  // use the true random number generator of the STM32L4
  uint32_t rand_val = 0;
  HAL_RNG_GenerateRandomNumber(&hrng, &rand_val);
  return (uint16_t)rand_val;
#else /* HAL_RNG_MODULE_ENABLED */
  // use the stdlib random generator; LCG algorithm by default:  rand_val = ((prev_rand_val * 1103515245U) + 12345U) & 0x7fffffff
  return rand();
#endif /* HAL_RNG_MODULE_ENABLED */
}


#ifdef HAL_RNG_MODULE_ENABLED
uint32_t random_rand32(void)
{
  // use the true random number generator of the STM32L4
  uint32_t rand_val = 0;
  HAL_RNG_GenerateRandomNumber(&hrng, &rand_val);
  return rand_val;
}
#endif /* HAL_RNG_MODULE_ENABLED */


/* make sure the HAL tick is enabled */
bool check_hal_tick(void)
{
  if (!(HALTICK_TIMER.Instance->CR1 & TIM_CR1_CEN) ||
      !NVIC_GetEnableIRQ(TIM1_UP_TIM16_IRQn)       ||
      !__HAL_TIM_GET_IT_SOURCE(&HALTICK_TIMER, TIM_IT_UPDATE)) {
    return false;
  }
  return true;
}
