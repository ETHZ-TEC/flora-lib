/*
 * lptimer.c
 *
 *  Created on: 16.08.2019
 *      Author: rdaforno
 */

/*
 * uses LPTIM1 to provide a general purpose timer functionality that works also in the low-power modes
 *
 * note:
 * - LPTIM1 has 1 capture/compare register
 * - LPTIM1 is sourced by LSE and runs at 32kHz
 */


#include "time/lptimer.h"
#include "stm32l4xx_hal.h"

extern LPTIM_HandleTypeDef hlptim1;

static uint32_t         lptimer_ext = 0;
static lptimer_cb_func_t lptimer_cb = 0;
static uint64_t         lptimer_exp = 0;             /* next expiration time */
static LPTIM_HandleTypeDef* const hrtim = &hlptim1;    /* immutable pointer */

void lptimer_set(uint64_t t_exp, lptimer_cb_func_t cb)
{
  lptimer_cb = cb;
  lptimer_exp = t_exp;
  if (t_exp == 0 || !cb) {
    /* stop the timer */
    __HAL_LPTIM_COMPARE_SET(hrtim, 0);
    __HAL_LPTIM_DISABLE_IT(hrtim, LPTIM_IT_CMPM);
    __HAL_LPTIM_CLEAR_FLAG(hrtim, LPTIM_IT_CMPM);
  } else {
    __HAL_LPTIM_COMPARE_SET(hrtim, (uint16_t)t_exp);
    __HAL_LPTIM_ENABLE_IT(hrtim, LPTIM_IT_CMPM);
  }
}

uint64_t lptimer_get(void)
{
  return lptimer_exp;
}

/* this function must be called when an lptimer has expired */
void lptimer_expired(void)
{
  uint32_t t_now = lptimer_now();
  if (lptimer_exp <= t_now) {
    __HAL_LPTIM_DISABLE_IT(hrtim, LPTIM_IT_CMPM);
    if (lptimer_cb) {
      lptimer_cb();                            /* execute callback function */
    }
  }
}

void lptimer_clear(void)
{
  /* clear the timer value */
  __HAL_TIM_SET_COUNTER(hrtim, 0);
  lptimer_ext = 0;
}

/* this function must be called when an overflow occurs */
void lptimer_update(void)
{
  lptimer_ext++;
}

uint64_t lptimer_now(void)
{
  uint64_t timestamp;
  uint16_t hw, hw2;

  /* make sure this routine runs atomically */
  uint32_t int_status = __get_PRIMASK();
  if (!int_status) {
    __disable_irq();
  }

  while (1) {
    timestamp = lptimer_ext;
    timestamp <<= 16;
    do {
      hw  = __HAL_TIM_GET_COUNTER(hrtim);
      hw2 = __HAL_TIM_GET_COUNTER(hrtim);
    } while (hw != hw2);

    if (!__HAL_LPTIM_GET_FLAG(hrtim, LPTIM_FLAG_ARRM)) {
      break;
    }
    /* an overflow occurred -> get a new timestamp */
    __HAL_LPTIM_CLEAR_FLAG(hrtim, LPTIM_FLAG_ARRM);
    lptimer_update();
  }
  timestamp |= hw;

  if (!int_status) {
    __enable_irq();
  }

  return timestamp;
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_expired();
}
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_update();
}

