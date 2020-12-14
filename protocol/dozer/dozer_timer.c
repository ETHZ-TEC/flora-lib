/*
 * dozer_timer.c
 *
 * tim15 handling (code was originally in hs_timer.c)
 *
 *      Author: kelmicha
 */


#include "flora_lib.h"


#if DOZER_ENABLE

#define TIM5_TIMER_FREQUENCY    1000      // ms timer (htim5) cycles per second
#define TIM5_TIMER_FREQUENCY_MS 1         // ms timer (htim5) cycles per ms
#define TIM5_TIMER_FREQUENCY_US 0.001     // ms timer (htim5) cycles per us

static void (*timeout2_callback)() = NULL;
volatile static uint64_t timeout2_offset = 0;

extern TIM_HandleTypeDef htim2;

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



void hs_timer_set_timeout2_timestamp(uint32_t timestamp)
{
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


void hs_timer_timeout2_stop(void)
{
  timeout2_offset = 0;

#ifndef DEVKIT
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_4);
#else
  HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_2);
#endif
}



#ifndef DEVKIT

uint32_t tim15_get_current_timestamp(void)
{
  uint32_t timestamp = htim15.Instance->CNT;
  timestamp |= ((uint32_t) counter_extension_tim15) << 16;
  return timestamp;
}


void tim15_set_current_timestamp(uint32_t timestamp)
{
  HAL_TIM_Base_Stop_IT(&htim15);
  htim15.Instance->CNT = timestamp & 0xffffU;
  counter_extension_tim15 = (timestamp >> 16);
  HAL_TIM_Base_Start_IT(&htim15);
}


void tim15_init(void)
{
  uint64_t timestamp = rtc_get_timestamp(false); // TODO: hs_timer ???
  tim15_set_current_timestamp((uint32_t)timestamp);
  tim15_initialized = true;
}


/*
 * Rx timeout watchdog
 */

void tim15_set_rx_timeout_watchdog_timestamp(uint32_t timestamp)
{
  htim15.Instance->CCR1 = timestamp;
}


void tim15_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback)) {
  rx_timeout_watchdog_callback = callback;

  tim15_set_rx_timeout_watchdog_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_CC1);
  HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_1);
}

void tim15_rx_timeout_watchdog_stop(void) {
  HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_1);
}


/*
 *Data generation timer
 */

void tim15_set_data_gen_timer_timestamp(uint32_t timestamp)
{
  htim15.Instance->CCR2 = timestamp;
}


void tim15_data_gen_timer_start(uint64_t compare_timestamp, void (*callback))
{
  data_gen_timer_callback = callback;

  tim15_set_data_gen_timer_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_CC2);
  HAL_TIM_OC_Start_IT(&htim15, TIM_CHANNEL_2);
}


void tim15_data_gen_timer_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim15, TIM_CHANNEL_2);
}

#else /* DEVKIT */

void tim5_init(void)
{
  uint64_t timestamp = rtc_get_timestamp(false);
  tim5_set_current_timestamp(timestamp);
  tim5_initialized = true;
}

uint64_t tim5_get_current_timestamp(void)
{
  uint64_t timestamp = htim5.Instance->CNT;
  timestamp |= ((uint64_t) counter_extension_tim5) << 32;
  return timestamp;
}

void tim5_set_current_timestamp(uint64_t timestamp)
{
  HAL_TIM_Base_Stop_IT(&htim5);
  htim5.Instance->CNT = timestamp & 0xffffU;
  counter_extension_tim5 = (timestamp >> 32);
  HAL_TIM_Base_Start_IT(&htim5);
}


/*
 * Rx timeout watchdog
 */

void tim5_set_rx_timeout_watchdog_timestamp(uint32_t timestamp)
{
  htim5.Instance->CCR1 = timestamp;
}


void tim5_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback))
{
  rx_timeout_watchdog_callback = callback;

  tim5_set_rx_timeout_watchdog_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC1);
  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_1);
}


void tim5_rx_timeout_watchdog_stop(void)
{
  HAL_TIM_OC_Stop_IT(&htim5, TIM_CHANNEL_1);
}


/*
 *Data generation timer
 */

void tim5_set_data_gen_timer_timestamp(uint32_t timestamp)
{
  htim5.Instance->CCR2 = timestamp;
}


void tim5_data_gen_timer_start(uint64_t compare_timestamp, void (*callback))
{
  data_gen_timer_callback = callback;

  tim5_set_data_gen_timer_timestamp(compare_timestamp);
  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);
  HAL_TIM_OC_Start_IT(&htim5, TIM_CHANNEL_2);
}


void tim5_data_gen_timer_stop(void)
{
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
#endif /* DEVKIT */


void tim5_handle_overflow(void)
{
  counter_extension_tim15++;
}

#endif /* DOZER_ENABLE */
