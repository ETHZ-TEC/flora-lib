/*
 * rtc.h
 *
 *  Created on: 26.04.2018
 *      Author: marku
 */

#ifndef TIME_RTC_H_
#define TIME_RTC_H_


#ifdef HAL_RTC_MODULE_ENABLED


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
void rtc_delay(uint32_t delay);
bool rtc_format_time(char* buffer, uint8_t buffer_size);

bool rtc_parse_date_string(RTC_DateTypeDef* rtc_date, RTC_TimeTypeDef* rtc_time, char* date_string);
bool rtc_set_alarm(uint64_t timestamp, void* callback);
bool rtc_set_alarm_daytime(uint32_t hour, uint32_t minute, uint32_t second, void (*callback)(void));
uint32_t rtc_time_to_next_alarm(void);
void rtc_try_to_sleep();

#else /* HAL_RTC_MODULE_ENABLED */

#define rtc_init(x)
#define rtc_get_timestamp(x)        0
#define rtc_delay(x)
#define rtc_format_time(x, y)       0

#endif /* HAL_RTC_MODULE_ENABLED */

#endif /* TIME_RTC_H_ */
