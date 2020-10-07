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
static void (*rtc_alarm_callback)(void) = NULL;

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


/* local helper function */
bool rtc_update_datetime(void)
{
  /* note: GetDate() must always be called after GetTime()! */
  return (HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) == HAL_OK &&
          HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) == HAL_OK);
}


bool rtc_set_date(uint32_t year, uint32_t month, uint32_t day)
{
  rtc_date.Year    = year % 100;  // 0..99
  rtc_date.Month   = month % 13;  // 1..12
  rtc_date.Date    = day % 32;    // 1..31
  rtc_date.WeekDay = 0;

  return HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) == HAL_OK;
}


bool rtc_set_time(uint32_t hour, uint32_t minute, uint32_t second)
{
  memset(&rtc_time, 0, sizeof(rtc_time));
  rtc_time.Hours   = hour % 24;
  rtc_time.Minutes = minute % 60;
  rtc_time.Seconds = second % 60;

  return HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) == HAL_OK;
}


/* shift (adjust) the RTC by a fraction of a second (positive offset means the clock will be advanced) */
uint32_t rtc_shift(int32_t offset_ms, bool block_until_rtc_updated)
{
  if (offset_ms >= 1000 || offset_ms <= -1000) {
    return 0;
  }
  if (IS_INTERRUPT()) {
    /* it is not recommended to run this operation from interrupt context since there is a busy wait of up to 1 second involved! */
    LOG_WARNING("running rtc_shift() from interrupt context!");
  }

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

  /* can only write to the shift register if no shift operation is pending (SHPF == 0) */
  while (hrtc.Instance->ISR & RTC_ISR_SHPF);

  /* set bit 31 to add one second and set bits 0..14 (SUBFS) to subtract a fraction of a second:
   *   advance[s] = ((SHIFTR >> 31) ? 1 : 0) - SUBFS / (PREDIV_S + 1)
   * where PREDIV_S is 0xFF by default
   */
  uint32_t shiftval = 0;
  if (offset_ms > 0) {
    shiftval  = RTC_SHIFTR_ADD1S;   // add 1s
    offset_ms = 1000 - offset_ms;
  } else {
    offset_ms = -offset_ms;
  }
  shiftval |= offset_ms * hrtc.Init.SynchPrediv / 1000;
  hrtc.Instance->SHIFTR = shiftval;

  /* wait until RSF == 1 and SHPF == 0 */
  while (!(hrtc.Instance->ISR & RTC_ISR_RSF) || (hrtc.Instance->ISR & RTC_ISR_SHPF));

  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  /* bugfix: wait for RTC to settle */
  if (block_until_rtc_updated) {
    delay_us(offset_ms * 1000);
  }

  LOG_VERBOSE("RTC shifted by %dms", offset_ms);

  return offset_ms;
}


/* set the drift compensation in ppm (positive value will add ticks and thus slows down the RTC) */
bool rtc_compensate_drift(int32_t offset_ppm)
{
  if (offset_ppm > 488 || offset_ppm < -488) {
    return false;
  }
  /* use the smooth digital calibration feature to compensate for RTC drift (see RM0394 p.1094 for details) */

  /* calibration register: RTC_CALR
   * add ticks to slow down or mask (skip) ticks to speed up
   * set CALM[8:0] to set the #ticks to mask in a 32s cycle (2^20 pulses) or set the CALP bit (0x8000) to increase the frequency / slow down by 488.5ppm (16 pulses per second)
   * calibrated frequency: FCAL = 32768Hz x [1 + (CALP x 512 - CALM) / (2^20 + CALM - CALP x 512)]
   * extra pulses per second: ((512 * CALP) - CALM) / 32, 0.9537ppm granularity */

  offset_ppm = (int32_t)((float)offset_ppm * 1.05f);
  if (offset_ppm > 0) {
    offset_ppm = RTC_CALR_CALP | (488 - offset_ppm);
  } else {
    offset_ppm = -offset_ppm;
  }

  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
  hrtc.Instance->CALR = offset_ppm;
  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  return true;
}


bool rtc_set_unix_timestamp(uint32_t timestamp)
{
  struct tm ts;

  time_t t = timestamp;                   /* must first be converted to time_t! */
  gmtime_r((time_t*)&t, &ts);

  rtc_date.Year    = ts.tm_year % 100;    /* ts.tm_year contains year since 1900 */
  rtc_date.Month   = ts.tm_mon + 1;
  rtc_date.Date    = ts.tm_mday;
  rtc_date.WeekDay = ts.tm_wday;

  rtc_time.Hours   = ts.tm_hour;
  rtc_time.Minutes = ts.tm_min % 60;
  rtc_time.Seconds = ts.tm_sec;
  rtc_time.SubSeconds     = 0;
  rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
  rtc_time.TimeFormat     = 0;

  if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) {
    return false;
  }
  LOG_VERBOSE("time set to %d-%02d-%02d %02d:%02d:%02d", (rtc_date.Year + 2000),
                                                          rtc_date.Month, rtc_date.Date,
                                                          rtc_time.Hours, rtc_time.Minutes,
                                                          rtc_time.Seconds);
  return true;
}


/* note: this function may block for up to 1 second if no 2nd argument is provided. otherwise the required amount of wait time is written to the 2nd argument. */
bool rtc_set_unix_timestamp_ms(uint64_t timestamp_ms, uint32_t* out_wait_time_ms)
{
  struct tm ts;
  int32_t granularity_ms = (1000UL / (rtc_time.SecondFraction + 1));

  /* calculate offset */
  int64_t offset_ms  = (int64_t)timestamp_ms - (int64_t)rtc_get_unix_timestamp_ms();
  if (offset_ms <= granularity_ms && offset_ms >= -granularity_ms) {
    LOG_VERBOSE("current offset (%ldms) is below the threshold", (int32_t)offset_ms);
    if (out_wait_time_ms) {
      *out_wait_time_ms = 0;
    }
    return true;      /* don't adjust offset if it is in the order of the RTC granularity */
  }

  time_t t = (uint32_t)(timestamp_ms / 1000);                   /* must first be converted to time_t! */
  gmtime_r((time_t*)&t, &ts);

  rtc_date.Year    = ts.tm_year % 100;    /* ts.tm_year contains year since 1900 */
  rtc_date.Month   = ts.tm_mon + 1;
  rtc_date.Date    = ts.tm_mday;
  rtc_date.WeekDay = ts.tm_wday;

  rtc_time.Hours          = ts.tm_hour;
  rtc_time.Minutes        = ts.tm_min % 60;
  rtc_time.Seconds        = ts.tm_sec;
  rtc_time.SubSeconds     = 0;            /* has no impact */
  rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;
  rtc_time.TimeFormat     = 0;

  if (HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN) != HAL_OK ||
      HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN) != HAL_OK) {
    return false;
  }

  /* note: HAL_RTC_SetTime() does not set the subseconds register (it is a read-only register) -> need to use RTC_SHIFTR to compensate down to ms level */
  uint32_t shift_ms = rtc_shift(timestamp_ms % 1000, out_wait_time_ms == 0);
  if (out_wait_time_ms) {
    *out_wait_time_ms = shift_ms;
  }

  LOG_VERBOSE("time set to %d-%02d-%02d %02d:%02d:%02d (offset was %lldms)", (rtc_date.Year + 2000),
                                                                             rtc_date.Month, rtc_date.Date,
                                                                             rtc_time.Hours, rtc_time.Minutes,
                                                                             rtc_time.Seconds, offset_ms);
  return true;
}


bool rtc_format_time(char* buffer, uint8_t buffer_size)
{
  if (rtc_update_datetime()) {
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


void rtc_get_time(uint32_t* hour, uint32_t* minute, uint32_t* second)
{
  rtc_update_datetime();
  if (hour) {
    *hour = rtc_time.Hours;
  }
  if (minute) {
    *minute = rtc_time.Minutes;
  }
  if (second) {
    *second = rtc_time.Seconds;
  }
}


uint32_t rtc_get_unix_timestamp(void)
{
  rtc_update_datetime();

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


uint64_t rtc_get_unix_timestamp_ms(void)
{
  uint64_t seconds = rtc_get_unix_timestamp();
  /* granularity: 1 Sec / (SecondFraction + 1) => 1 / 256 by default */
  uint32_t subsecs = (rtc_time.SecondFraction - rtc_time.SubSeconds) * 1000UL / (rtc_time.SecondFraction + 1);
  if (rtc_time.SubSeconds > rtc_time.SecondFraction) {
    seconds--;
  }
  return seconds * 1000ULL + subsecs;
}


uint64_t rtc_get_timestamp(bool hs_timer)
{
  struct tm curr_time;

  rtc_update_datetime();

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


bool rtc_set_alarm(uint64_t timestamp, void* callback)
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

  return HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN) == HAL_OK;
}


bool rtc_set_alarm_daytime(uint32_t hour, uint32_t minute, uint32_t second, void (*callback)(void))
{
  rtc_alarm_callback = callback;

  if (!callback) {
    __HAL_RTC_ALARMA_DISABLE(&hrtc);
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    return false;
  }

  rtc_update_datetime();

  if (hour < 24 && minute < 60 && second < 60) {
    rtc_time.Hours   = hour;
    rtc_time.Minutes = minute;
    rtc_time.Seconds = second;
    rtc_time.SubSeconds = rtc_time.SecondFraction;              /* 0ms */
    RTC_AlarmTypeDef alarm = {
        .AlarmTime = rtc_time,
        .AlarmMask = RTC_ALARMMASK_DATEWEEKDAY,
        .AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE,      /* sub seconds must match */
        .AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE,
        .AlarmDateWeekDay = 0,
        .Alarm = RTC_ALARM_A,
    };
    if (HAL_RTC_SetAlarm_IT(&hrtc, &alarm, RTC_FORMAT_BIN) != HAL_OK) {
      LOG_ERROR("failed to set alarm");
      return false;
    }

    LOG_VERBOSE("alarm set for %02u:%02u:%02u", hour, minute, second);
    return true;
  }
  return false;
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


void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  if (rtc_alarm_callback) {
    rtc_alarm_callback();
  }
}


#endif /* HAL_RTC_MODULE_ENABLED */
