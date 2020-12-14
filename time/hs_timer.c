/*
 * hs_timer.c
 *
 *  Created on: 14.05.2018
 *      Author: marku
 */

/*
 * Notes:
 * - hs_timer_handle_overflow() must be called from HAL_TIM_PeriodElapsedCallback(), which is located in the project specific files (typically main.c)
 */

#include "flora_lib.h"


extern TIM_HandleTypeDef htim2;

#if HS_TIMER_COMPENSATE_DRIFT
double_t hs_timer_offset = 0;
double_t hs_timer_drift = 1;
#endif /* HS_TIMER_COMPENSATE_DRIFT */

bool      hs_timer_recovered_by_rtc     = false;
uint64_t  hs_timer_scheduled_timestamp  = 0;
bool      hs_timer_scheduled            = false;

static hs_timer_cb_t schedule_callback = NULL;
static hs_timer_cb_t timeout_callback  = NULL;
static hs_timer_cb_t capture_callback  = NULL;
#if !BOLT_ENABLE
static hs_timer_cb_t generic_callback  = NULL;
#endif /* BOLT_ENABLE */

static uint32_t hs_timer_counter_extension          = 0;
static uint32_t hs_timer_capture_counter_extension  = 0;
static uint32_t hs_timer_schedule_counter_extension = 0;
static uint32_t hs_timer_timeout_counter_extension  = 0;
#if !BOLT_ENABLE
static uint32_t hs_timer_generic_counter_extension  = 0;
#endif /* BOLT_ENABLE */

static uint64_t timeout_timestamp = 0;



void hs_timer_init(void)
{
#if HS_TIMER_INIT_FROM_RTC
  hs_timer_recovered_by_rtc = true;
  uint64_t timestamp = rtc_get_timestamp(false);
  hs_timer_set_counter(timestamp);
#else
  hs_timer_counter_extension = 0;
#endif /* HS_TIMER_INIT_FROM_RTC */
}


#if HS_TIMER_COMPENSATE_DRIFT

void hs_timer_set_offset(double_t offset)
{
  hs_timer_offset = offset;
}

void hs_timer_adapt_offset(double_t delta)
{
  hs_timer_recovered_by_rtc = false;
  hs_timer_offset += delta;
}

double_t hs_timer_get_offset(void)
{
  return hs_timer_offset;
}

void hs_timer_set_drift(double_t drift)
{
  if (drift) {
    hs_timer_drift = drift;
  }
}

double_t hs_timer_get_drift(void)
{
  return hs_timer_drift;
}

#endif /* HS_TIMER_COMPENSATE_DRIFT */


void hs_timer_set_counter(uint64_t timestamp)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, (uint32_t)timestamp);
  hs_timer_counter_extension = (timestamp >> 32);
  HAL_TIM_Base_Start_IT(&htim2);
}


void hs_timer_set_lower_counter(uint32_t timestamp)
{
  __HAL_TIM_SET_COUNTER(&htim2, timestamp);
}


void hs_timer_set_schedule_timestamp(uint64_t timestamp)
{
#if HS_TIMER_COMPENSATE_DRIFT
  uint64_t ts = (uint64_t) round((timestamp - hs_timer_offset) / hs_timer_drift);
#else
  uint64_t ts = timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
  hs_timer_schedule_counter_extension = ts >> 32;
#ifndef DEVKIT
  htim2.Instance->CCR2 = (uint32_t) ts;
#else
  htim2.Instance->CCR1 = (uint32_t) ts;
#endif
}


void hs_timer_set_timeout_timestamp(uint64_t timestamp)
{
#if HS_TIMER_COMPENSATE_DRIFT
  uint64_t ts = (uint64_t) round((timestamp - hs_timer_offset) / hs_timer_drift);
#else
  uint64_t ts = timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
  hs_timer_timeout_counter_extension = ts >> 32;
  htim2.Instance->CCR3 = (uint32_t) ts;
}


#if !BOLT_ENABLE
void hs_timer_set_generic_timestamp(uint64_t timestamp)
{
 #if HS_TIMER_COMPENSATE_DRIFT
  uint64_t ts = (uint64_t) round((timestamp - hs_timer_offset) / hs_timer_drift);
 #else
  uint64_t ts = timestamp;
 #endif /* HS_TIMER_COMPENSATE_DRIFT */
  hs_timer_generic_counter_extension = ts >> 32;
 #ifndef DEVKIT
  htim2.Instance->CCR4 = (uint32_t) ts;
 #else
  htim2.Instance->CCR2 = (uint32_t) ts;
 #endif /* DEVIT */
}
#endif /* BOLT_ENABLE */


inline uint64_t hs_timer_get_current_timestamp(void)
{
  uint64_t timestamp = htim2.Instance->CNT;
  timestamp |= ((uint64_t) hs_timer_counter_extension) << 32;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint64_t hs_timer_now(void)
{
  if (IS_INTERRUPT()) {
    return 0;   /* this function must not be called from interrupt */
  }
  uint32_t hw_ts, hw_ts2, sw_ext, sw_ext2;

  do {
    hw_ts  = htim2.Instance->CNT;
    sw_ext = hs_timer_counter_extension;
    hw_ts2  = htim2.Instance->CNT;
    sw_ext2 = hs_timer_counter_extension;
  } while (sw_ext != sw_ext2 || hw_ts > hw_ts2);

  return ((uint64_t)sw_ext << 32) | hw_ts;
}


uint32_t hs_timer_get_schedule_timestamp(void)
{
#ifndef DEVKIT
  uint32_t timestamp = htim2.Instance->CCR2;
#else
  uint32_t timestamp = htim2.Instance->CCR1;
#endif /* DEVKIT */
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint32_t) (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint64_t hs_timer_get_capture_timestamp(void)
{
  uint64_t timestamp;
#ifndef DEVKIT
  timestamp = htim2.Instance->CCR1;
#else
  timestamp = htim2.Instance->CCR4;
#endif
  timestamp |= ((uint64_t) hs_timer_capture_counter_extension) << 32;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}

uint64_t hs_timer_get_compare_timestamp(void)
{
  uint64_t timestamp;
#ifndef DEVKIT
  timestamp = htim2.Instance->CCR2;
#else
  timestamp = htim2.Instance->CCR1;
#endif
  timestamp |= ((uint64_t) hs_timer_schedule_counter_extension) << 32;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint64_t hs_timer_get_timeout_timestamp(void)
{
  uint64_t timestamp;
  timestamp = htim2.Instance->CCR3;
  timestamp |= ((uint64_t) hs_timer_timeout_counter_extension) << 32;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


#if !BOLT_ENABLE
uint64_t hs_timer_get_generic_timestamp(void)
{
  uint64_t timestamp;
 #ifndef DEVKIT
  timestamp = htim2.Instance->CCR4;
 #else
  timestamp = htim2.Instance->CCR2;
 #endif
  timestamp |= ((uint64_t) hs_timer_generic_counter_extension) << 32;
 #if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
 #else
  return timestamp;
 #endif /* HS_TIMER_COMPENSATE_DRIFT */
}
#endif /* BOLT_ENABLE */


uint32_t hs_timer_get_counter_extension(void)
{
  return hs_timer_counter_extension;
}


void hs_timer_capture(hs_timer_cb_t callback)
{
  capture_callback = callback;
#ifndef DEVKIT
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
#else
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
#endif
}


void hs_timer_schedule(uint64_t timestamp, hs_timer_cb_t callback)
{
  uint64_t now = hs_timer_get_current_timestamp();

  if ((timestamp - now) < TIMER_GUARD_TIME || (timestamp - now) > (uint64_t) INT64_MAX) {
    callback();
    LOG_WARNING("Schedule too late!");
  }
  else {
    hs_timer_scheduled_timestamp = timestamp;
    hs_timer_scheduled = true;
    schedule_callback = callback;
    hs_timer_set_schedule_timestamp(timestamp);
#ifndef DEVKIT
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
#else
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
#endif
  }
}


void hs_timer_timeout(uint64_t timeout, hs_timer_cb_t callback)
{
  timeout_callback  = callback;
  timeout_timestamp = timeout;
  if (timeout_timestamp) {
    hs_timer_set_timeout_timestamp(timeout);
    //__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
  } else {
    HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
  }
}


#if !BOLT_ENABLE

void hs_timer_generic(uint64_t timestamp, hs_timer_cb_t callback) {
  uint64_t now = hs_timer_get_current_timestamp();

  if ((timestamp - now) < TIMER_GUARD_TIME || (timestamp - now) > (uint64_t) INT64_MAX) {
    callback();
  }
  else {
    generic_callback = callback;
    hs_timer_set_generic_timestamp(timestamp);
#ifndef DEVKIT
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
#else
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
#endif
  }
}

#endif /* BOLT_ENABLE */


void hs_timer_schedule_stop(void)
{
#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
#endif
}


void hs_timer_timeout_stop(void)
{
  timeout_timestamp = 0;
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
}

#if !BOLT_ENABLE
void hs_timer_generic_stop(void)
{
#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
#endif
}
#endif /* BOLT_ENABLE */


// TIM2 capture interrupt handler
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      //HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
      hs_timer_capture_counter_extension = hs_timer_counter_extension;

      if (capture_callback) {
        capture_callback();
      }
    }
  }
}


void hs_timer_handle_overflow(TIM_HandleTypeDef *htim)
{
  hs_timer_counter_extension++;
}


// TIM2 output compare interrupt handler
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
#ifndef DEVKIT
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
      if (schedule_callback) {
        schedule_callback();
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
  #if !BOLT_ENABLE
      if (generic_callback) {
        generic_callback();
      }
  #endif /* BOLT_ENABLE */
    }
#else /* DEVKIT */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      if((hs_timer_scheduled_timestamp >> 32) == hs_timer_counter_extension) {
        hs_timer_scheduled = false;
        HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
        if (schedule_callback) {
          schedule_callback();
        }
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
  #if !BOLT_ENABLE
      if (generic_callback) {
        generic_callback();
      }
  #endif /* BOLT_ENABLE */
    }
#endif /* DEVKIT */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
      timeout_timestamp = 0;
      if (timeout_callback) {
        timeout_callback();
      }
    }
  }
}

