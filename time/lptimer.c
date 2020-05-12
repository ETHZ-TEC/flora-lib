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

#include "flora_lib.h"


extern LPTIM_HandleTypeDef hlptim1;
extern TIM_HandleTypeDef   htim2;

static uint32_t                   lptimer_ext = 0;           /* software extension */
static lptimer_cb_func_t          lptimer_cb  = 0;           /* callback function */
static uint64_t                   lptimer_exp = 0;           /* next expiration time */


void lptimer_set(uint64_t t_exp, lptimer_cb_func_t cb)
{
  lptimer_cb = cb;
  lptimer_exp = t_exp;
  if (t_exp == 0 || !cb) {
    /* stop the timer */
    __HAL_LPTIM_COMPARE_SET(&hlptim1, 0);
    __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_CMPM);
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_IT_CMPM);
  } else {
    __HAL_LPTIM_COMPARE_SET(&hlptim1, (uint16_t)t_exp);
    __HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_CMPM);
#if LPTIMER_CHECK_EXP_TIME
    uint64_t curr_timestamp = lptimer_now();
    if (t_exp <= curr_timestamp || t_exp > (curr_timestamp + LPTIMER_SECOND * 86400)) {
      LOG_WARNING("wakeup time is in the past or far in the future");
    }
#endif /* LPTIMER_CHECK_EXP_TIME */
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
    __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_CMPM);
    if (lptimer_cb) {
      lptimer_cb();                            /* execute callback function */
    }
  }
}

void lptimer_clear(void)
{
  /* clear the timer value */
  __HAL_TIM_SET_COUNTER(&hlptim1, 0);
  lptimer_ext = 0;
}

/* this function must be called when an overflow occurs */
void lptimer_update(void)
{
  lptimer_ext++;

#if LPTIMER_RESET_WDG_ON_OVF
 #ifdef HAL_IWDG_MODULE_ENABLED
  /* kick the watchdog */
  HAL_IWDG_Refresh(&hiwdg);
 #endif /* HAL_IWDG_MODULE_ENABLED */
#endif /* LPTIMER_RESET_WDG_ON_OVF */
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

  timestamp = lptimer_ext;
  do {
    hw  = __HAL_TIM_GET_COUNTER(&hlptim1);
    hw2 = __HAL_TIM_GET_COUNTER(&hlptim1);
  } while (hw != hw2);
  if (__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_ARRM)) {
    timestamp++;
    do {
      hw  = __HAL_TIM_GET_COUNTER(&hlptim1);
      hw2 = __HAL_TIM_GET_COUNTER(&hlptim1);
    } while (hw != hw2);
  }
  timestamp <<= 16;
  timestamp |= hw;

  if (!int_status) {
    __enable_irq();
  }

  return timestamp;
}

bool lptimer_now_synced(uint64_t* lp_timestamp, uint64_t* hs_timestamp)
{
  uint16_t lp_cnt, lp_cnt2;
  uint32_t hs_cnt, hs_cnt2;
  uint32_t hs_ext, lp_ext;

  /* arguments must be non-zero and timers must be running */
  if (!lp_timestamp || !hs_timestamp ||
      IS_INTERRUPT() ||
      !(hlptim1.Instance->CR & LPTIM_CR_ENABLE) ||
      !(htim2.Instance->CR1 & TIM_CR1_CEN)) {
    return false;
  }
  /* disable interrupts */
  uint32_t int_status = __get_PRIMASK();
  if (!int_status) {
    __disable_irq();
  }
  /* get counter extensions */
  hs_ext = hs_timer_get_counter_extension();
  lp_ext = lptimer_ext;
  /* is there a pending timer overflow/update interrupt? */
  if (__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_ARRM)) {
    lp_ext++;
  }
  if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
    hs_ext++;
  }
  /* get current hs timer value */
  hs_cnt = __HAL_TIM_GET_COUNTER(&htim2);
  /* sample until we have a consistent lptimer timestamp */
  do {
    lp_cnt  = __HAL_TIM_GET_COUNTER(&hlptim1);
    lp_cnt2 = __HAL_TIM_GET_COUNTER(&hlptim1);
  } while (lp_cnt != lp_cnt2);
  /* now wait for the timestamp to change */
  lp_cnt++;
  while (__HAL_TIM_GET_COUNTER(&htim2) != lp_cnt);
  /* read the hs timer value */
  hs_cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
  /* handle the unlikely case of a rollover */
  if (hs_cnt2 < hs_cnt) {
    hs_ext++;
  }
  if (lp_cnt2 == 0) {
    lp_ext++;
  }
  /* compose the final timestamps */
  *lp_timestamp = ((uint64_t)lp_ext << 16) | lp_cnt;
  *hs_timestamp = ((uint64_t)hs_ext << 32) | hs_cnt2;

  /* re-enable interrupts if necessary */
  if (!int_status) {
    __enable_irq();
  }
  return true;
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_expired();
}
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_update();
}

