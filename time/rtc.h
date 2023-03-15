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

#ifndef TIME_RTC_H_
#define TIME_RTC_H_


#ifdef HAL_RTC_MODULE_ENABLED

#ifndef RTC_WAKEUP_IRQ_PRIORITY
#define RTC_WAKEUP_IRQ_PRIORITY   6
#endif


void rtc_init();

bool rtc_set_date(uint32_t year, uint32_t month, uint32_t day);
bool rtc_set_time(uint32_t hour, uint32_t minute, uint32_t second);
bool rtc_set_unix_timestamp(uint32_t timestamp);
bool rtc_set_unix_timestamp_ms(uint64_t timestamp, uint32_t* out_wait_time_ms);
bool rtc_compensate_drift(int32_t offset_ppm);

void rtc_get_time(uint32_t* hour, uint32_t* minute, uint32_t* second);
uint32_t rtc_get_daytime_sec(void);
uint32_t rtc_get_unix_timestamp(void);
uint64_t rtc_get_unix_timestamp_ms(void);
uint64_t rtc_get_timestamp(bool hs_timer);
uint32_t rtc_get_next_timestamp_at_daytime(time_t curr_time, uint32_t hour, uint32_t minute, uint32_t second);
void rtc_delay(uint32_t delay);
bool rtc_format_time(char* buffer, uint8_t buffer_size);

bool rtc_parse_date_string(RTC_DateTypeDef* rtc_date, RTC_TimeTypeDef* rtc_time, char* date_string);
bool rtc_set_alarm(uint64_t timestamp, void* callback);
bool rtc_set_alarm_daytime(uint32_t hour, uint32_t minute, uint32_t second, void (*callback)(void));
uint32_t rtc_time_to_next_alarm(void);
void rtc_start_wakeup_timer(uint32_t period_s);
void rtc_stop_wakeup_timer(void);

#else /* HAL_RTC_MODULE_ENABLED */

#define rtc_init(x)
#define rtc_get_timestamp(x)        0
#define rtc_delay(x)
#define rtc_format_time(x, y)       0

#endif /* HAL_RTC_MODULE_ENABLED */

#endif /* TIME_RTC_H_ */
