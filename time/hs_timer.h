/*
 * hs_timer.h
 *
 *  Created on: 14.05.2018
 *      Author: marku
 */

#ifndef TIME_HS_TIMER_H_
#define TIME_HS_TIMER_H_


#include <math.h>
#include <stdint.h>

#define HS_TIMER_FREQUENCY 8000000 // High speed timer (htim2) cycles per second
#define HS_TIMER_FREQUENCY_MS 8000 // High speed timer (htim2) cycles per ms
#define HS_TIMER_FREQUENCY_US 8 // High speed timer (htim2) cycles per us

#ifdef DOZER
#define TIM5_TIMER_FREQUENCY 1000     // ms timer (htim5) cycles per second
#define TIM5_TIMER_FREQUENCY_MS 1     // ms timer (htim5) cycles per ms
#define TIM5_TIMER_FREQUENCY_US 0.001 // ms timer (htim5) cycles per us
#endif

#define TIMER_GUARD_TIME 800 // 100us; worst case time needed so set the timer correctly (schedule and generic)

void hs_timer_init();

void hs_timer_set_offset(double_t offset);
void hs_timer_adapt_offset(double_t delta);
double_t hs_timer_get_offset();

void hs_timer_set_drift(double_t drift);
double_t hs_timer_get_drift();

void hs_timer_set_counter(uint64_t timestamp);
void hs_timer_set_lower_counter(uint32_t timestamp);
void hs_timer_set_schedule_timestamp(uint64_t timestamp);
void hs_timer_set_timeout_timestamp(uint64_t timestamp);
void hs_timer_set_generic_timestamp(uint64_t timestamp);

uint64_t hs_timer_get_current_timestamp();
uint64_t hs_timer_get_capture_timestamp();
uint32_t hs_timer_get_schedule_timestamp();
uint64_t hs_timer_get_compare_timestamp();
uint64_t hs_timer_get_timeout_timestamp();
uint64_t hs_timer_get_generic_timestamp();

void hs_timer_capture(void (*callback));
void hs_timer_capture4(void (*callback));
void hs_timer_schedule(uint64_t timestamp, void (*callback)());
void hs_timer_timeout(uint64_t offset, void (*callback));
void hs_timer_timeout_start(uint64_t compare_timestamp);
void hs_timer_generic(uint64_t timestamp, void (*callback)());

void hs_timer_schedule_stop();
void hs_timer_timeout_stop();
void hs_timer_generic_stop();

void hs_timer_handle_overflow(void);




#ifdef DOZER

void hs_timer_set_timeout2_timestamp(uint32_t timestamp);
void hs_timer_timeout2_start(uint64_t compare_timestamp, void (*callback));
void hs_timer_timeout2_stop();


#ifndef DEVKIT

void tim15_init();
void tim15_set_current_timestamp(uint32_t timestamp);
uint32_t tim15_get_current_timestamp();


// Rx timeout watchdog
void tim15_set_rx_timeout_watchdog_timestamp(uint32_t timestamp);
void tim15_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback));
void tim15_rx_timeout_watchdog_stop();

// Data generation
void tim15_set_data_gen_timer_timestamp(uint32_t timestamp);
void tim15_data_gen_timer_start(uint64_t compare_timestamp, void (*callback));
void tim15_data_gen_timer_stop();

#else

void tim5_init();
void tim5_set_current_timestamp(uint64_t timestamp);
uint64_t tim5_get_current_timestamp();


// Rx timeout watchdog
void tim5_set_rx_timeout_watchdog_timestamp(uint32_t timestamp);
void tim5_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback));
void tim5_rx_timeout_watchdog_stop();

// Data generation
void tim5_set_data_gen_timer_timestamp(uint32_t timestamp);
void tim5_data_gen_timer_start(uint64_t compare_timestamp, void (*callback));
void tim5_data_gen_timer_stop();

//void tim5_set_rec_rx_timeout_watchdog_timestamp(uint32_t timestamp);
//void tim5_rec_rx_timeout_watchdog_start(uint64_t compare_timestamp, void (*callback));
//void tim5_rec_rx_timeout_watchdog_stop();
//
//void tim5_set_con_req_timer_timestamp(uint32_t timestamp);
//void tim5_con_req_timer_start(uint64_t compare_timestamp, void (*callback));
//void tim5_con_req_timer_stop();

#endif
#endif


#endif /* TIME_HS_TIMER_H_ */
