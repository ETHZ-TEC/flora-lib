/*
 * lptimer.c
 *
 *  Created on: 16.08.2019
 *      Author: rdaforno
 */

/*
 * uses LPTIM1 to provide a general purpose timer functionality that works also in the low-power modes
 *
 * Notes:
 * - LPTIM1 has 1 capture/compare register
 * - LPTIM1 is sourced by LSE and runs at 32kHz
 * - there are 3 known limitations of LPTIM1 (see errata sheet: https://www.st.com/resource/en/errata_sheet/dm00218216-stm32l433xx443xx-device-errata-stmicroelectronics.pdf)
 * - do not disable the overflow interrupt in LPM (stop mode)
 * - LPTIM_IT_CMPM interrupt can only be enabled/disabled when the peripheral is disabled -> simplest is to enable interrupt before starting the timer
 */

#include "flora_lib.h"


extern LPTIM_HandleTypeDef hlptim1;
extern TIM_HandleTypeDef   htim2;

static uint32_t            lptimer_ext = 0;           /* software extension */
static lptimer_cb_func_t   lptimer_cb  = 0;           /* callback function */
static uint64_t            lptimer_exp = 0;           /* next expiration time */


void lptimer_set(uint64_t t_exp, lptimer_cb_func_t cb)
{
  lptimer_cb  = cb;
  lptimer_exp = t_exp;

  if (t_exp != 0 && cb) {

#if LPTIMER_CHECK_EXP_TIME
    uint64_t curr_timestamp = lptimer_now();
    if (t_exp <= curr_timestamp) {
      LOG_WARNING("expiration time is in the past (now: %llu, scheduled: %llu)", curr_timestamp, t_exp);
      /* schedule expiration in a few ticks */
      lptimer_exp = curr_timestamp + 3;
      /* remark: do not execute the callback here */

    } else if (t_exp > (curr_timestamp + LPTIMER_SECOND * 86400)) {
      LOG_WARNING("expiration time is more than 24h in the future (now: %llu, scheduled: %llu)", curr_timestamp, t_exp);
    }
#endif /* LPTIMER_CHECK_EXP_TIME */

    __HAL_LPTIM_COMPARE_SET(&hlptim1, (uint16_t)lptimer_exp);
  }
}


uint64_t lptimer_get(void)
{
  return lptimer_exp;
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


/* this function must be called when an lptimer has expired (CCR match) */
void lptimer_expired(void)
{
  if (lptimer_exp) {
    uint32_t curr_ext = lptimer_ext;
    if (__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_ARRM)) {
      curr_ext++;
    }
    if ((uint32_t)(lptimer_exp >> 16) <= curr_ext) {
      if (lptimer_cb) {
        lptimer_cb();                            /* execute callback function */
      }
    }
  }

#if LPTIMER_RESET_WDG_ON_EXP
 #ifdef HAL_IWDG_MODULE_ENABLED
  /* kick the watchdog */
  HAL_IWDG_Refresh(&hiwdg);
 #endif /* HAL_IWDG_MODULE_ENABLED */
#endif /* LPTIMER_RESET_WDG_ON_EXP */
}


uint64_t lptimer_now(void)
{
  uint64_t timestamp;
  uint16_t hw, hw2;

  /* make sure this routine runs atomically */
  ENTER_CRITICAL_SECTION();

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

  LEAVE_CRITICAL_SECTION();

  return timestamp;
}


bool lptimer_now_synced(uint64_t* lp_timestamp, uint64_t* hs_timestamp)
{
  uint16_t lp_cnt, lp_cnt2;
  uint32_t hs_cnt, hs_cnt2;
  uint32_t hs_ext, lp_ext;

  /* arguments must be non-zero and timers must be running */
  if (!lp_timestamp || !hs_timestamp ||
      !(hlptim1.Instance->CR & LPTIM_CR_ENABLE) ||
      !(htim2.Instance->CR1 & TIM_CR1_CEN)) {
    return false;
  }
  /* disable interrupts */
  ENTER_CRITICAL_SECTION();

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
  lp_cnt2++;
  while (lp_cnt != lp_cnt2) {
    lp_cnt = __HAL_TIM_GET_COUNTER(&hlptim1);
  }
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
  *lp_timestamp = ((uint64_t)lp_ext << 16) | lp_cnt2;
  *hs_timestamp = ((uint64_t)hs_ext << 32) | hs_cnt2;

  /* re-enable interrupts if necessary */
  LEAVE_CRITICAL_SECTION();

  return true;
}


void lptimer_enable_ovf_int(bool enable)
{
  if (enable) {
    __HAL_LPTIM_ENABLE_IT(&hlptim1, LPTIM_IT_ARRM);
  } else {
    __HAL_LPTIM_DISABLE_IT(&hlptim1, LPTIM_IT_ARRM);
  }
}


void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_expired();
}


void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  lptimer_update();
}

