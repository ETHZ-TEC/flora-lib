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

#ifndef TIME_HS_TIMER_H_
#define TIME_HS_TIMER_H_


#define HS_TIMER_FREQUENCY          8000000   // High speed timer (htim2) cycles per second
#define HS_TIMER_FREQUENCY_MS       8000      // High speed timer (htim2) cycles per ms
#define HS_TIMER_FREQUENCY_US       8         // High speed timer (htim2) cycles per us

#define HS_TIMER_GUARD_TIME         800       // 100us; worst case time needed so set the timer correctly (schedule and generic)

#ifndef HS_TIMER_COMPENSATE_DRIFT
#define HS_TIMER_COMPENSATE_DRIFT   1
#endif /* HS_TIMER_COMPENSATE_DRIFT */

#ifndef HS_TIMER_INIT_FROM_RTC
#define HS_TIMER_INIT_FROM_RTC      1
#endif /* HS_TIMER_INIT_FROM_RTC */


#define HS_TIMER_TICKS_TO_LPTIMER(hs_ticks)   ((hs_ticks) * LPTIMER_FREQUENCY / HS_TIMER_FREQUENCY)
#define HS_TIMER_TICKS_TO_S(hs_ticks)         ((hs_ticks) / HS_TIMER_FREQUENCY)
#define HS_TIMER_TICKS_TO_MS(hs_ticks)        ((hs_ticks) / HS_TIMER_FREQUENCY_MS)
#define HS_TIMER_TICKS_TO_US(hs_ticks)        ((hs_ticks) / HS_TIMER_FREQUENCY_US)
#define HS_TIMER_TICKS_TO_NS(hs_ticks)        ((hs_ticks) * 1000 / HS_TIMER_FREQUENCY_US)
#define HS_TIMER_S_TO_TICKS(s)                ((s)  * HS_TIMER_FREQUENCY)
#define HS_TIMER_MS_TO_TICKS(ms)              ((ms) * HS_TIMER_FREQUENCY_MS)
#define HS_TIMER_US_TO_TICKS(us)              ((us) * HS_TIMER_FREQUENCY_US)
#define HS_TIMER_NS_TO_TICKS(ns)              ((ns) * HS_TIMER_FREQUENCY_US / 1000)

#define hs_timer_now                          hs_timer_get_current_timestamp    // Alias

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
void hs_timer_schedule_start(uint64_t timestamp, hs_timer_cb_t callback);
void hs_timer_timeout_start(uint64_t timeout, hs_timer_cb_t callback);

#if !BOLT_ENABLE
void hs_timer_generic_start(uint64_t timestamp, hs_timer_cb_t callback);
#endif /* BOLT_ENABLE */

void hs_timer_schedule_stop(void);
void hs_timer_timeout_stop(void);
void hs_timer_generic_stop(void);

void hs_timer_handle_overflow(TIM_HandleTypeDef *htim);


#endif /* TIME_HS_TIMER_H_ */
