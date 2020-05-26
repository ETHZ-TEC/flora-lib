/*
 * rtc.c
 *
 *  Created on: 26.04.2018
 *      Author: marku
 */

#include "flora_lib.h"

#ifdef HAL_RTC_MODULE_ENABLED

RTC_TimeTypeDef rtc_time = {0};
RTC_DateTypeDef rtc_date = {0};
static volatile void (*rtc_alarm_callback) = NULL;

extern RTC_HandleTypeDef hrtc;
extern bool     hs_timer_scheduled;
extern uint64_t hs_timer_scheduled_timestamp;
extern bool     hs_timer_recovered_by_rtc;
extern double_t hs_timer_offset;


void rtc_init()
{
  /* make sure the alarm is disabled */
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  char buffer[32];
  rtc_format_time(buffer, 32);
  LOG_VERBOSE(buffer);

  return;
}


bool rtc_set_date(uint32_t year, uint32_t month, uint32_t day)
{
  rtc_date.Year    = year % 100;  // 0..99
  rtc_date.Month   = month % 12;  // 1..12
  rtc_date.Date    = day % 31;    // 1..31
  rtc_date.WeekDay = 0;

  return HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) == HAL_OK;
}


bool rtc_set_time(uint32_t hour, uint32_t minute, uint32_t second)
{
  memset(&rtc_time, 0, sizeof(rtc_time));
  rtc_time.Hours   = hour % 23;
  rtc_time.Minutes = minute % 59;
  rtc_time.Seconds = second % 59;

  return HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) == HAL_OK;
}


bool rtc_set_unix_timestamp(uint32_t timestamp)
{
  struct tm ts;

  gmtime_r((time_t*)&timestamp, &ts);

  rtc_date.Year    = ts.tm_year % 100;
  rtc_date.Month   = ts.tm_mon + 1;
  rtc_date.Date    = ts.tm_mday;
  rtc_date.WeekDay = ts.tm_wday;

  rtc_time.Hours   = ts.tm_hour;
  rtc_time.Minutes = ts.tm_min % 59;
  rtc_time.Seconds = ts.tm_sec;
  rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
  rtc_time.TimeFormat     = 0;

  if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) {
    return false;
  }
  LOG_VERBOSE("time set");
  return true;
}


bool rtc_format_time(char* buffer, uint8_t buffer_size)
{
  if (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) == HAL_OK &&
      HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) == HAL_OK) {
    snprintf(buffer, buffer_size, "%d-%02d-%02d %02d:%02d:%02d", (rtc_date.Year + 2000),
                                                                 rtc_date.Month, rtc_date.Date,
                                                                 rtc_time.Hours, rtc_time.Minutes,
                                                                 rtc_time.Seconds);

    return true;
  }
  return false;
}


bool rtc_parse_date_string(RTC_DateTypeDef* rtc_date, RTC_TimeTypeDef* rtc_time, char* date_string)
{
  int year = 0;
  int month = 0;
  int date = 0;

  int hours = 0;
  int minutes = 0;
  int seconds = 0;

  sscanf(date_string, "%4d-%2d-%2d %2d:%2d:%2d", &year, &month, &date, &hours, &minutes, &seconds);

  // Rough check if parsed values are valid
  if (year >= 2000 && year < 2100 &&
      month >= 1 && month < 13 &&
      date >= 1 && date < 32 &&
      hours >= 0 && hours < 24 &&
      minutes >= 0 && minutes < 60 &&
      seconds >= 0 && seconds < 60) {
    rtc_date->Year = (year - 2000);
    rtc_date->Month = month;
    rtc_date->Date = date;

    rtc_time->Hours = hours;
    rtc_time->Minutes = minutes;
    rtc_time->Seconds = seconds;

    return true;

  } else
  {
    return false;
  }
}


uint32_t rtc_get_unix_timestamp(void)
{
  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

  struct tm curr_time;

  curr_time.tm_year = rtc_date.Year + 100;
  curr_time.tm_mday = rtc_date.Date;
  curr_time.tm_mon  = rtc_date.Month - 1;

  curr_time.tm_hour = rtc_time.Hours;
  curr_time.tm_min  = rtc_time.Minutes;
  curr_time.tm_sec  = rtc_time.Seconds;

  time_t seconds = mktime(&curr_time);

  return seconds;
}


uint64_t rtc_get_timestamp(bool hs_timer)
{
  HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

  struct tm curr_time;

  curr_time.tm_year = rtc_date.Year + 100;
  curr_time.tm_mday = rtc_date.Date;
  curr_time.tm_mon  = rtc_date.Month - 1;

  curr_time.tm_hour = rtc_time.Hours;
  curr_time.tm_min  = rtc_time.Minutes;
  curr_time.tm_sec  = rtc_time.Seconds;

  time_t seconds = mktime(&curr_time);

  uint64_t timestamp = ((uint64_t) seconds) * HS_TIMER_FREQUENCY + (HS_TIMER_FREQUENCY - ((uint64_t) rtc_time.SubSeconds * HS_TIMER_FREQUENCY / 32768 ));
#if HS_TIMER_COMPENSATE_DRIFT
  if (hs_timer) {
    timestamp += hs_timer_offset;
  }
#endif /* HS_TIMER_COMPENSATE_DRIFT */

  return timestamp;
}


void rtc_set_alarm(uint64_t timestamp, void* callback)
{
  rtc_alarm_callback = callback;

  uint64_t rtc_alarm_timestamp = timestamp - hs_timer_offset;
  time_t rtc_alarm_seconds_timestamp = rtc_alarm_timestamp / HS_TIMER_FREQUENCY;
  uint16_t rtc_alarm_subsecond_timestamp = 32768 - (rtc_alarm_timestamp - rtc_alarm_seconds_timestamp * HS_TIMER_FREQUENCY) * 32768 / HS_TIMER_FREQUENCY;
  struct tm* datetime = localtime(&rtc_alarm_seconds_timestamp);

  RTC_TimeTypeDef rtc_time = {
      .Hours = datetime->tm_hour,
      .Minutes = datetime->tm_min,
      .Seconds = datetime->tm_sec,
      .TimeFormat = RTC_HOURFORMAT12_PM,
      .SubSeconds = rtc_alarm_subsecond_timestamp,
      .DayLightSaving = RTC_DAYLIGHTSAVING_NONE,
      .StoreOperation = RTC_STOREOPERATION_RESET,
  };

  RTC_AlarmTypeDef alarm = {
      .AlarmTime = rtc_time,
      .AlarmMask = RTC_ALARMMASK_DATEWEEKDAY, // Prevent MCU being sleeping for more than 1 day. //RTC_ALARMMASK_NONE,
      .AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE,
      .AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE,
      .AlarmDateWeekDay = datetime->tm_mday,
      .Alarm = RTC_ALARM_A,
  };

  HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN);
}


/*
 * delay in ms
 */
void rtc_delay(uint32_t delay)
{
  uint64_t start = rtc_get_timestamp(false);
  while(rtc_get_timestamp(false) < (start + (uint64_t) delay * HS_TIMER_FREQUENCY_MS))
  {
    __NOP();
    //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
  }
}


void rtc_try_to_sleep()
{
  if (hs_timer_scheduled && (hs_timer_scheduled_timestamp - hs_timer_get_current_timestamp()) >= HS_TIMER_FREQUENCY_MS * 4 && (hs_timer_scheduled_timestamp - hs_timer_get_current_timestamp()) <= INT64_MAX) {
    hs_timer_recovered_by_rtc = true;
    rtc_set_alarm(hs_timer_scheduled_timestamp - HS_TIMER_FREQUENCY_MS * 2, NULL);
    system_sleep(false);
  }
}


/*
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  return;
}
*/

#endif /* HAL_RTC_MODULE_ENABLED */
