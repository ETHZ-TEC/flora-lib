/*
 * rtc.h
 *
 *  Created on: 26.04.2018
 *      Author: marku
 */

#ifndef TIME_RTC_H_
#define TIME_RTC_H_


#include "stm32l4xx_hal.h" // potentially defines HAL_RTC_MODULE_ENABLED => needs to stay before "#ifdef HAL_RTC_MODULE_ENABLE"

#ifdef HAL_RTC_MODULE_ENABLED

void rtc_init();
/**
 * @brief           Returns the current rtc timestamp
 *
 * @param hs_timer  Should HS timer offset be added or not
 * @return uint64_t Current rtc timestamp in HS Timer Frequency
 */
uint64_t rtc_get_timestamp(bool hs_timer);
void rtc_delay(uint32_t delay);
bool rtc_format_time(char* buffer, uint8_t buffer_size);

bool rtc_parse_date_string(RTC_DateTypeDef* rtc_date, RTC_TimeTypeDef* rtc_time, char* date_string);
void rtc_set_alarm(uint64_t timestamp, void* callback);
void rtc_try_to_sleep();

#else /* HAL_RTC_MODULE_ENABLED */

#define rtc_init(x)
#define rtc_get_timestamp(x)        0
#define rtc_delay(x)
#define rtc_format_time(x, y)       0

#endif /* HAL_RTC_MODULE_ENABLED */

#endif /* TIME_RTC_H_ */
