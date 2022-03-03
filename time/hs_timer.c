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

/*
 * Notes:
 * - hs_timer_handle_overflow() must be called from HAL_TIM_PeriodElapsedCallback(), which is located in the project specific files (typically main.c)
 */

#include "flora_lib.h"


extern TIM_HandleTypeDef htim2;

#if HS_TIMER_COMPENSATE_DRIFT
double_t hs_timer_offset = 0;
double_t hs_timer_drift  = 1;
#endif /* HS_TIMER_COMPENSATE_DRIFT */


static hs_timer_cb_t schedule_callback = NULL;
static hs_timer_cb_t timeout_callback  = NULL;
static hs_timer_cb_t capture_callback  = NULL;
#if !BOLT_ENABLE
static hs_timer_cb_t generic_callback  = NULL;
#endif /* BOLT_ENABLE */

static uint64_t hs_timer_scheduled_timestamp        = 0;
static uint32_t hs_timer_counter_extension          = 0;
static uint32_t hs_timer_capture_counter_extension  = 0;
static uint32_t hs_timer_schedule_counter_extension = 0;
static uint32_t hs_timer_timeout_counter_extension  = 0;
#if !BOLT_ENABLE
static uint32_t hs_timer_generic_counter_extension  = 0;
#endif /* BOLT_ENABLE */



void hs_timer_init(void)
{
#if HS_TIMER_INIT_FROM_RTC
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


uint32_t hs_timer_get_counter(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}


void hs_timer_set_counter(uint64_t timestamp)
{
  HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, (uint32_t)timestamp);
  hs_timer_counter_extension = (timestamp >> 32);
  HAL_TIM_Base_Start_IT(&htim2);
}


void hs_timer_set_schedule_timestamp(uint64_t timestamp)
{
#if HS_TIMER_COMPENSATE_DRIFT
  uint64_t ts = (uint64_t) round((timestamp - hs_timer_offset) / hs_timer_drift);
#else
  uint64_t ts = timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
  hs_timer_schedule_counter_extension = ts >> 32;
  htim2.Instance->CCR2 = (uint32_t) ts;
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
  htim2.Instance->CCR4 = (uint32_t) ts;
}
#endif /* BOLT_ENABLE */


inline uint64_t hs_timer_get_current_timestamp(void)
{
  ENTER_CRITICAL_SECTION();
  uint64_t timestamp = __HAL_TIM_GET_COUNTER(&htim2);
  timestamp |= ((uint64_t) hs_timer_counter_extension) << 32;
  LEAVE_CRITICAL_SECTION();
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint32_t hs_timer_get_schedule_timestamp(void)
{
  uint32_t timestamp = htim2.Instance->CCR2;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint32_t) (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint64_t hs_timer_get_capture_timestamp(void)
{
  ENTER_CRITICAL_SECTION();
  uint64_t timestamp = htim2.Instance->CCR1;
  timestamp |= ((uint64_t) hs_timer_capture_counter_extension) << 32;
  LEAVE_CRITICAL_SECTION();
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}

uint64_t hs_timer_get_compare_timestamp(void)
{
  ENTER_CRITICAL_SECTION();
  uint64_t timestamp = htim2.Instance->CCR1;
  timestamp |= ((uint64_t) hs_timer_schedule_counter_extension) << 32;
  LEAVE_CRITICAL_SECTION();
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


uint64_t hs_timer_get_timeout_timestamp(void)
{
  ENTER_CRITICAL_SECTION();
  uint64_t timestamp = htim2.Instance->CCR3;
  timestamp |= ((uint64_t) hs_timer_timeout_counter_extension) << 32;
  LEAVE_CRITICAL_SECTION();
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}


#if !BOLT_ENABLE
uint64_t hs_timer_get_generic_timestamp(void)
{
  ENTER_CRITICAL_SECTION();
  uint64_t timestamp = htim2.Instance->CCR4;
  timestamp |= ((uint64_t) hs_timer_generic_counter_extension) << 32;
  LEAVE_CRITICAL_SECTION();
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
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}


void hs_timer_schedule_start(uint64_t timestamp, hs_timer_cb_t callback)
{
  if (callback) {
    uint64_t now = hs_timer_get_current_timestamp();
    if (timestamp < (now + HS_TIMER_GUARD_TIME)) {
      callback();
      LOG_WARNING("Schedule too late!");
    }
    else {
      hs_timer_scheduled_timestamp = timestamp;
      schedule_callback = callback;
      hs_timer_set_schedule_timestamp(timestamp);
      __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
      HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
    }
  }
}


void hs_timer_timeout_start(uint64_t timeout, hs_timer_cb_t callback)
{
  if (callback) {
    timeout_callback = callback;
    hs_timer_set_timeout_timestamp(timeout);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
  }
}


#if !BOLT_ENABLE

void hs_timer_generic_start(uint64_t timestamp, hs_timer_cb_t callback)
{
  if (callback) {
    uint64_t now = hs_timer_get_current_timestamp();
    if ((timestamp - now) < HS_TIMER_GUARD_TIME) {
      LOG_WARNING("Generic too late!");
      callback();
    }
    else if ((timestamp - now) > (uint64_t) INT64_MAX) {
      LOG_WARNING("Generic strange condition");
      callback();
    }
    else {
      generic_callback = callback;
      hs_timer_set_generic_timestamp(timestamp);
      HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
    }
  }
}

#endif /* BOLT_ENABLE */


void hs_timer_schedule_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
  schedule_callback = 0;
}


void hs_timer_timeout_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
  timeout_callback = 0;
}

#if !BOLT_ENABLE
void hs_timer_generic_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
  generic_callback = 0;
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
  if (htim->Instance == TIM2) {

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
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
      if (timeout_callback) {
        timeout_callback();
      }
    }
  }
}
