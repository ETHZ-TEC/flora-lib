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


#ifdef HAL_RNG_MODULE_ENABLED

uint32_t random_rand32(void)
{
  // use the true random number generator of the STM32L4
  uint32_t rand_val = 0;
  HAL_RNG_GenerateRandomNumber(&hrng, &rand_val);
  return rand_val;
}

#else

uint32_t random_rand32(void)
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


bool semaphore_acquire(semaphore_t* sem)
{
  bool acquired = false;
  if (sem) {
    ENTER_CRITICAL_SECTION();
    if (*sem > 0) {
      (*sem)--;
      acquired = true;
    }
    LEAVE_CRITICAL_SECTION();
  }
  return acquired;
}


void semaphore_release(semaphore_t* sem)
{
  if (sem) {
    ENTER_CRITICAL_SECTION();
    (*sem)++;
    LEAVE_CRITICAL_SECTION();
  }
}
