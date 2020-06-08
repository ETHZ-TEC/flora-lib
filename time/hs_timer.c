/*
 * hs_timer.c
 *
 *  Created on: 14.05.2018
 *      Author: marku
 */

#include "flora_lib.h"


extern TIM_HandleTypeDef htim2;

#if HS_TIMER_COMPENSATE_DRIFT
double_t hs_timer_offset = 0;
double_t hs_timer_drift = 1;
#endif /* HS_TIMER_COMPENSATE_DRIFT */

bool hs_timer_recovered_by_rtc = false;
uint64_t hs_timer_scheduled_timestamp = 0;
bool hs_timer_scheduled = false;

static void (*schedule_callback)() = NULL;
static void (*timeout_callback)() = NULL;
static void (*capture_callback)() = NULL;
#if !BOLT_ENABLE
static void (*generic_callback)() = NULL;
#endif /* BOLT_ENABLE */

uint32_t hs_timer_counter_extension = 0;
static uint32_t hs_timer_capture_counter_extension = 0;
static uint32_t hs_timer_schedule_counter_extension = 0;
static uint32_t hs_timer_timeout_counter_extension = 0;
#if !BOLT_ENABLE
static uint32_t hs_timer_generic_counter_extension = 0;
#endif /* BOLT_ENABLE */

bool hs_timer_initialized = false;
static uint64_t timeout_offset = 0;


#if DOZER_ENABLE

static void (*timeout2_callback)() = NULL;
volatile static uint64_t timeout2_offset = 0;


#ifndef DEVKIT
extern TIM_HandleTypeDef htim15;

uint16_t counter_extension_tim15 = 0;
bool tim15_initialized = false;

#else
extern TIM_HandleTypeDef htim5;

uint32_t counter_extension_tim5 = 0;
bool tim5_initialized = false;

#endif
static void (*rx_timeout_watchdog_callback)() = NULL;
static void (*data_gen_timer_callback)() = NULL;
//static void (*rec_rx_timeout_watchdog_callback)() = NULL;
//static void (*con_req_timer_callback)() = NULL;

#endif /* DOZER_ENABLE */


void hs_timer_init()
{
#if HS_TIMER_INIT_FROM_RTC
  hs_timer_recovered_by_rtc = true;
  uint64_t timestamp = rtc_get_timestamp(false);
  hs_timer_set_counter(timestamp);
#else
  hs_timer_counter_extension = 0;
#endif /* HS_TIMER_INIT_FROM_RTC */

  hs_timer_initialized = true;
}

#if HS_TIMER_COMPENSATE_DRIFT

void hs_timer_set_offset(double_t offset) {
  hs_timer_offset = offset;
}

void hs_timer_adapt_offset(double_t delta) {
  hs_timer_recovered_by_rtc = false;
  hs_timer_offset += delta;
}

double_t hs_timer_get_offset() {
  return hs_timer_offset;
}

void hs_timer_set_drift(double_t drift) {
  if (drift) {
    hs_timer_drift = drift;
  }
}

double_t hs_timer_get_drift() {
  return hs_timer_drift;
}

#endif /* HS_TIMER_COMPENSATE_DRIFT */

void hs_timer_set_counter(uint64_t timestamp) {
  HAL_TIM_Base_Stop_IT(&htim2);
  htim2.Instance->CNT = timestamp & 0xffffffffU;
  hs_timer_counter_extension = (timestamp >> 32);
  HAL_TIM_Base_Start_IT(&htim2);
}

void hs_timer_set_lower_counter(uint32_t timestamp) {
  htim2.Instance->CNT = timestamp;
}

void hs_timer_set_schedule_timestamp(uint64_t timestamp) {
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

void hs_timer_set_timeout_timestamp(uint64_t timestamp) {
#if HS_TIMER_COMPENSATE_DRIFT
  uint64_t ts = (uint64_t) round((timestamp - hs_timer_offset) / hs_timer_drift);
#else
  uint64_t ts = timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
  hs_timer_timeout_counter_extension = ts >> 32;
  htim2.Instance->CCR3 = (uint32_t) ts;
}

#if !BOLT_ENABLE
void hs_timer_set_generic_timestamp(uint64_t timestamp) {
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

inline uint64_t hs_timer_get_current_timestamp() {
  uint64_t timestamp = htim2.Instance->CNT;
  timestamp |= ((uint64_t) hs_timer_counter_extension) << 32;
#if HS_TIMER_COMPENSATE_DRIFT
  return (uint64_t) round(timestamp * hs_timer_drift + hs_timer_offset);
#else
  return timestamp;
#endif /* HS_TIMER_COMPENSATE_DRIFT */
}

uint64_t hs_timer_now() {
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

uint32_t hs_timer_get_schedule_timestamp() {
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

uint64_t hs_timer_get_capture_timestamp() {
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

uint64_t hs_timer_get_compare_timestamp() {
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

uint64_t hs_timer_get_timeout_timestamp() {
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
uint64_t hs_timer_get_generic_timestamp() {
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

uint32_t hs_timer_get_counter_extension() {
  return hs_timer_counter_extension;
}

void hs_timer_capture(void (*callback)) {
  capture_callback = callback;
#ifndef DEVKIT
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
#else
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);
#endif
}


void hs_timer_schedule(uint64_t timestamp, void (*callback)()) {
  uint64_t now = hs_timer_get_current_timestamp();

  if ((timestamp - now) < TIMER_GUARD_TIME || (timestamp - now) > (uint64_t) INT64_MAX) {
    // FLOCKLAB_PIN_SET(FLOCKLAB_LED1);
    // FLOCKLAB_PIN_CLR(FLOCKLAB_LED1);
    callback();
    LOG_WARNING("Schedule too late!");
  }
  else {
    hs_timer_scheduled_timestamp = timestamp;
    hs_timer_scheduled = true;
    schedule_callback = callback;
    hs_timer_set_schedule_timestamp(timestamp);
#ifndef DEVKIT
    //__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
#else
    //__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
#endif
  }
}

void hs_timer_timeout(uint64_t offset, void (*callback)) {
  timeout_callback = callback;
  timeout_offset = offset;
}

void hs_timer_timeout_start(uint64_t compare_timestamp) {
  if (timeout_offset) {
    hs_timer_set_timeout_timestamp(compare_timestamp + timeout_offset);
    //__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC3);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
  }
}

#if !BOLT_ENABLE
void hs_timer_generic(uint64_t timestamp, void (*callback)()) {
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


void hs_timer_schedule_stop() {
#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
#endif
}

void hs_timer_timeout_stop() {
  timeout_offset = 0;
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_FLAG_CC3);
}

#if !BOLT_ENABLE
void hs_timer_generic_stop() {
#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
#endif
}
#endif /* BOLT_ENABLE */

// TIM2 capture interrupt handler
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
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
#if DOZER_ENABLE

  if (hs_timer_initialized && htim->Instance == TIM2) {
    hs_timer_counter_extension++;
  }
 #ifndef DEVKIT
  else if (tim15_initialized && htim->Instance == TIM15) {
    counter_extension_tim15++;
  }
 #else
  else if (tim5_initialized && htim->Instance == TIM5) {
    counter_extension_tim5++;
  }
 #endif

#else /* DOZER_ENABLE */

  if (hs_timer_initialized) {
    hs_timer_counter_extension++;
  }

#endif /* DOZER_ENABLE */
}


#if !DOZER_ENABLE

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
      timeout_offset = 0;
      if (timeout_callback) {
        timeout_callback();
      }
    }
  }
}
#endif /* DOZER_ENABLE */


#if DOZER_ENABLE
// TIM2 output compare interrupt handler
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
 #ifndef DEVKIT
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
      if(schedule_callback) {
        schedule_callback();
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
      timeout2_offset = 0;
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);

      if(timeout2_callback) {
        timeout2_callback();
      }
    }
 #else /* DEVKIT */
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      if((hs_timer_scheduled_timestamp >> 32) == hs_timer_counter_extension) {
        hs_timer_scheduled = false;
        HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_1);
        if(schedule_callback) {
          schedule_callback();
        }
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      timeout2_offset = 0;
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);

      if(timeout2_callback) {
        timeout2_callback();
      }
    }
 #endif /* DEVKIT */
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
      HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
      timeout_offset = 0;
      if(timeout_callback) {
        timeout_callback();
      }
    }


  }
 #ifndef DEVKIT
  else if (htim->Instance == TIM15)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_1);

      if(rx_timeout_watchdog_callback) {
        rx_timeout_watchdog_callback();
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_2);

      if(data_gen_timer_callback) {
        data_gen_timer_callback();
      }
    }
  }
 #else /* DEVKIT */
  else if (htim->Instance == TIM5)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
      HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);

      if(rx_timeout_watchdog_callback) {
        cli_log("tim5 bt", "TIM5", CLI_LOG_LEVEL_WARNING);// TODO: delete
        rx_timeout_watchdog_callback();
      }
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
      HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_2);

      if(data_gen_timer_callback) {
        data_gen_timer_callback();
      }
    }
//    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
//      HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_3);
//
//      if(rec_rx_timeout_watchdog_callback) {
//        rec_rx_timeout_watchdog_callback();
//      }
//    }
//    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
//        HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_4);
//
//        if(con_req_timer_callback) {
//          con_req_timer_callback();
//        }
//      }



  }
 #endif /* DEVKIT */
}

#endif /* DOZER_ENABLE */




#if DOZER_ENABLE

void hs_timer_set_timeout2_timestamp(uint32_t timestamp) {
#ifndef DEVKIT
  htim2.Instance->CCR4 = timestamp;
#else
  htim2.Instance->CCR2 = timestamp;
#endif
}

void hs_timer_timeout2_start(uint64_t compare_timestamp, void (*callback)) {

  timeout2_callback = callback;

#ifndef DEVKIT
  hs_timer_set_timeout2_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC4);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);
#else
  hs_timer_set_timeout2_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC2);
  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
#endif
}

void hs_timer_timeout2_stop() {
  timeout2_offset = 0;

#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
#endif
}



#ifndef DEVKIT

uint32_t tim15_get_current_timestamp() {
  uint32_t timestamp = htim15.Instance->CNT;
  timestamp |= ((uint32_t) counter_extension_tim15) << 16;
  return timestamp;
}

void tim15_set_current_timestamp(uint32_t timestamp) {
  HAL_TIM_Base_Stop_IT(&htim15);
  htim15.Instance->CNT = timestamp & 0xffffU;
  counter_extension_tim15 = (timestamp >> 16);
  HAL_TIM_Base_Start_IT(&htim15);
}

void tim15_init()
{
  uint64_t timestamp = rtc_get_timestamp(false); // TODO: hs_timer ???
  tim15_set_current_timestamp((uint32_t)timestamp);
  tim15_initialized = true;
}


/*
 * Rx timeout watchdog
 */

void tim15_set_rx_timeout_watchdog_timestamp(uint32_t timestamp) {
  htim15.Instance->CCR1 = timestamp;
}


void tim15_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback)) {
  rx_timeout_watchdog_callback = callback;

  tim15_set_rx_timeout_watchdog_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_CC1);
  HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_1);
}

void tim15_rx_timeout_watchdog_stop() {
  HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_1);
}


/*
 *Data generation timer
 */

void tim15_set_data_gen_timer_timestamp(uint32_t timestamp) {
  htim15.Instance->CCR2 = timestamp;
}

void tim15_data_gen_timer_start(uint64_t compare_timestamp, void (*callback)) {
  data_gen_timer_callback = callback;

  tim15_set_data_gen_timer_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_CC2);
  HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_2);
}

void tim15_data_gen_timer_stop() {
  HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_2);
}
#else
void tim5_init()
{
  uint64_t timestamp = rtc_get_timestamp(false);
  tim5_set_current_timestamp(timestamp);
  tim5_initialized = true;
}

uint64_t tim5_get_current_timestamp() {
  uint64_t timestamp = htim5.Instance->CNT;
  timestamp |= ((uint64_t) counter_extension_tim5) << 32;
  return timestamp;
}

void tim5_set_current_timestamp(uint64_t timestamp) {
  HAL_TIM_Base_Stop_IT(&htim5);
  htim5.Instance->CNT = timestamp & 0xffffU;
  counter_extension_tim5 = (timestamp >> 32);
  HAL_TIM_Base_Start_IT(&htim5);
}


/*
 * Rx timeout watchdog
 */

void tim5_set_rx_timeout_watchdog_timestamp(uint32_t timestamp) {
  htim5.Instance->CCR1 = timestamp;
}


void tim5_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback)) {
  rx_timeout_watchdog_callback = callback;

  tim5_set_rx_timeout_watchdog_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
}

void tim5_rx_timeout_watchdog_stop() {
  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);
}


/*
 *Data generation timer
 */

void tim5_set_data_gen_timer_timestamp(uint32_t timestamp) {
  htim5.Instance->CCR2 = timestamp;
}

void tim5_data_gen_timer_start(uint64_t compare_timestamp, void (*callback)) {
  data_gen_timer_callback = callback;

  tim5_set_data_gen_timer_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);
}

void tim5_data_gen_timer_stop() {
  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_2);
}


/*
 * Beacon timer
 */

//void tim5_set_rec_rx_timeout_watchdog_timestamp(uint32_t timestamp) {
//  htim5.Instance->CCR3 = timestamp;
//}
//
//
//void tim5_rec_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback)) {
//  rec_rx_timeout_watchdog_callback = callback;
//
//  tim5_set_rec_rx_timeout_watchdog_timestamp(compare_timestamp);
//  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC3);
//  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_3);
//}
//
//void tim5_rec_rx_timeout_watchdog_stop() {
////  rec_rx_timeout_watchdog_offset = 0;
//  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_3);
//}


/*
 * Connection request timer
 */

//void tim5_set_con_req_timer_timestamp(uint32_t timestamp) {
//  htim5.Instance->CCR4 = timestamp;
//}
//
//
//void tim5_con_req_timer_start(uint64_t compare_timestamp, void (*callback)) {
//  con_req_timer_callback = callback;
//
//  tim5_set_con_req_timer_timestamp(compare_timestamp);
//  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC4);
//  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_4);
//}
//
//void tim5_con_req_timer_stop() {
//  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_4);
//}
#endif

#endif /* DOZER_ENABLE */
