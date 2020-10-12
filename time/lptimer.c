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
 * - there is a number of known quirks with the LPTIM peripheral (see e.g. https://gist.github.com/jefftenney/02b313fe649a14b4c75237f925872d72#file-lptimtick-c-L72)
 */

#include "flora_lib.h"


#define LPTIM_UPDATE_PENDING()    __HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_ARRM)


extern LPTIM_HandleTypeDef hlptim1;
extern TIM_HandleTypeDef   htim2;

static uint32_t            lptimer_ext = 0;           /* software extension */
static lptimer_cb_func_t   lptimer_cb  = 0;           /* callback function */
static uint64_t            lptimer_exp = 0;           /* next expiration time */


void lptimer_update(void);


uint16_t lptimer_count(void)
{
  uint16_t count;
  do {
    /* since timer and CPU clock are asynchronous: sample until we have a consistent counter value */
    count = __HAL_TIM_GET_COUNTER(&hlptim1);
  } while (count != __HAL_TIM_GET_COUNTER(&hlptim1));
  return count;
}


uint32_t lptimer_seconds(void)
{
  uint32_t secs = lptimer_ext * 2 + lptimer_count() / LPTIMER_SECOND;
  if (LPTIM_UPDATE_PENDING()) {
    secs += 2;
  }
  return secs;
}


void lptimer_set(uint64_t t_exp, lptimer_cb_func_t cb)
{
  lptimer_cb  = cb;
  lptimer_exp = t_exp;

  if (t_exp != 0 && cb) {

#if LPTIMER_CHECK_EXP_TIME
    uint64_t curr_timestamp = lptimer_now();
    if (t_exp <= curr_timestamp) {
      LOG_WARNING("expiration time is in the past (now: %llu, scheduled: %llu)", curr_timestamp, t_exp);
      /* remark: do not execute the callback here */

    } else if (t_exp > (curr_timestamp + LPTIMER_SECOND * 86400)) {
      LOG_WARNING("expiration time is more than 24h in the future (now: %llu, scheduled: %llu)", curr_timestamp, t_exp);
    }
#endif /* LPTIMER_CHECK_EXP_TIME */

    /* first, make sure no overflow interrupt is pending (causes false triggers when updating the compare register) */
    /*if (IS_INTERRUPT() && LPTIM_UPDATE_PENDING()) {
      __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_ARRM);
      lptimer_update();
    }*/
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPOK);
    __HAL_LPTIM_COMPARE_SET(&hlptim1, (uint16_t)t_exp);
    while (!__HAL_LPTIM_GET_FLAG(&hlptim1, LPTIM_FLAG_CMPOK));    /* wait until compare register set */
    __HAL_LPTIM_CLEAR_FLAG(&hlptim1, LPTIM_FLAG_CMPOK);
  }
}


uint64_t lptimer_get(void)
{
  return lptimer_exp;
}


void lptimer_clear(void)
{
#define LPTIM_CR_COUNTRST       (1UL << 3)

  /* clear the timer counter value by setting a bit in the control register */
  if ((hlptim1.Instance->CR & LPTIM_CR_COUNTRST) == 0) {    /* must be zero */
    hlptim1.Instance->CR |= LPTIM_CR_COUNTRST;
  }
  lptimer_ext = 0;
  lptimer_cb  = 0;
  lptimer_exp = 0;
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
  if (lptimer_cb) {
    uint32_t ext = lptimer_ext;
    uint16_t cnt = lptimer_count();
    if (LPTIM_UPDATE_PENDING()) {
      ext++;
      cnt = lptimer_count();
    }
    uint64_t now = ((uint64_t)ext << 16) | cnt;
    if (now >= lptimer_exp) {
      lptimer_cb_func_t cb = lptimer_cb;
      lptimer_cb = 0;                     /* must be reset before entering the callback since the timer could be reset within the callback */
      if (cb) {
        cb();                             /* execute callback function */
      }
      if (now > (lptimer_exp + LPTIMER_MS_TO_TICKS(LPTIMER_EXP_TIME_TH_MS))) {
        LOG_WARNING("timer fired %lums too late", (uint32_t)LPTIMER_TICKS_TO_MS(now - lptimer_exp));
      }
    } else if (ext == (uint32_t)(lptimer_exp >> 16)) {
      LOG_WARNING("tick count (%u) and compare register (%u) differ", cnt, (uint16_t)hlptim1.Instance->CMP);
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
  /* make sure this routine runs atomically */
  ENTER_CRITICAL_SECTION();

  uint64_t timestamp = lptimer_ext;
  uint16_t count     = lptimer_count();
  if (LPTIM_UPDATE_PENDING()) {    /* overflow occurred? */
    timestamp++;
    count = lptimer_count();
  }
  timestamp <<= 16;
  timestamp |= count;

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
  if (LPTIM_UPDATE_PENDING()) {
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

