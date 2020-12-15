/*
 * hs_timer.h
 *
 *  Created on: 14.05.2018
 *      Author: marku
 */

#ifndef TIME_HS_TIMER_H_
#define TIME_HS_TIMER_H_


#define HS_TIMER_FREQUENCY      8000000   // High speed timer (htim2) cycles per second
#define HS_TIMER_FREQUENCY_MS   8000      // High speed timer (htim2) cycles per ms
#define HS_TIMER_FREQUENCY_US   8         // High speed timer (htim2) cycles per us

#define HS_TIMER_GUARD_TIME     800       // 100us; worst case time needed so set the timer correctly (schedule and generic)

#ifndef HS_TIMER_COMPENSATE_DRIFT
#define HS_TIMER_COMPENSATE_DRIFT   1
#endif /* HS_TIMER_COMPENSATE_DRIFT */

#ifndef HS_TIMER_INIT_FROM_RTC
#define HS_TIMER_INIT_FROM_RTC  1
#endif /* HS_TIMER_INIT_FROM_RTC */


typedef void (* hs_timer_cb_t)(void);


void hs_timer_init(void);

#if HS_TIMER_COMPENSATE_DRIFT

void     hs_timer_set_offset(double_t offset);
void     hs_timer_adapt_offset(double_t delta);
double_t hs_timer_get_offset(void);
void     hs_timer_set_drift(double_t drift);
double_t hs_timer_get_drift(void);

#else /* HS_TIMER_COMPENSATE_DRIFT */

#define hs_timer_set_offset(offset)
#define hs_timer_adapt_offset(delta)
#define hs_timer_get_offset()     0
#define hs_timer_set_drift(drift)
#define hs_timer_get_drift()      1

#endif /* HS_TIMER_COMPENSATE_DRIFT */

void hs_timer_set_counter(uint64_t timestamp);
void hs_timer_set_schedule_timestamp(uint64_t timestamp);
void hs_timer_set_timeout_timestamp(uint64_t timestamp);
void hs_timer_set_generic_timestamp(uint64_t timestamp);

uint32_t hs_timer_get_counter(void);
uint64_t hs_timer_get_current_timestamp(void);
uint64_t hs_timer_get_capture_timestamp(void);
uint32_t hs_timer_get_schedule_timestamp(void);
uint64_t hs_timer_get_compare_timestamp(void);
uint64_t hs_timer_get_timeout_timestamp(void);
uint64_t hs_timer_get_generic_timestamp(void);
uint32_t hs_timer_get_counter_extension(void);

/* non-deterministic and uncompensated version of hs_timer_get_current_timestamp()
 * the returned timestamp is consistent
 * this function can't be called from an interrupt context */
uint64_t hs_timer_now(void);

void hs_timer_capture(hs_timer_cb_t callback);
void hs_timer_schedule(uint64_t timestamp, hs_timer_cb_t callback);
void hs_timer_timeout(uint64_t timeout, hs_timer_cb_t callback);

#if !BOLT_ENABLE
void hs_timer_generic(uint64_t timestamp, hs_timer_cb_t callback);
#endif /* BOLT_ENABLE */

void hs_timer_schedule_stop(void);
void hs_timer_timeout_stop(void);
void hs_timer_generic_stop(void);

void hs_timer_handle_overflow(TIM_HandleTypeDef *htim);


#endif /* TIME_HS_TIMER_H_ */
