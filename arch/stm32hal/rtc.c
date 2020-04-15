/*
 * rtc.c
 *
 *  Created on: 26.04.2018
 *      Author: marku
 */

#ifdef HAL_RTC_MODULE_ENABLED

#include <stdbool.h>
#include <time/rtc.h>
#include <time.h>

#include "stm32l4xx_hal.h"
#include "cli/cli.h"
#include "cli/command.h"

#include "time/hs_timer.h"
#include "system/system.h"


extern RTC_HandleTypeDef hrtc;
extern bool hs_timer_scheduled;
extern uint64_t hs_timer_scheduled_timestamp;
extern bool hs_timer_recovered_by_rtc;

extern int64_t hs_timer_offset;

RTC_TimeTypeDef rtc_time = {0};
RTC_DateTypeDef rtc_date = {0};

bool rtc_initialized = false;

static volatile void (*rtc_alarm_callback) = NULL;

static command_return_t rtc_time_command_handler(command_execution_t execution);
static command_return_t rtc_sleep_command_handler(command_execution_t execution);

static parameter_t rtc_command_time_parameters[] = {
  {
    .name = "date",
    .description = "date & time in ISO 8601 (e.g. \"2018-08-31 13:37:00\")",
    .type = CMD_PARAMETER_DATE,
    .options = NULL,
    .option_count = 0,
    .flag = 'd',
    .optional = true,
    .has_flag = true,
    .has_countable_options = false,
  }
};

static command_t rtc_time_command = {
  .execution_ptr = &rtc_time_command_handler,
  .name = "time",
  .description = "Get or set date & time",
  .prompt = "",
  .parameters = (parameter_t**) &rtc_command_time_parameters,
  .parameter_count = PARAM_COUNT(rtc_command_time_parameters),
  .parent = NULL,
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static parameter_t rtc_command_sleep_parameters[] = {
  {
    .name = "offset",
    .description = "Offset in HS timer ticks",
    .type = CMD_PARAMETER_INTEGER,
    .options = NULL,
    .option_count = 0,
    .flag = '\0',
    .optional = false,
    .has_flag = false,
    .has_countable_options = false,
  }
};

static command_t rtc_sleep_command = {
  .execution_ptr = &rtc_sleep_command_handler,
  .name = "sleep",
  .description = "Put node in sleep (standby with RAM2 retention)",
  .prompt = "",
  .parameters = (parameter_t**) &rtc_command_sleep_parameters,
  .parameter_count = PARAM_COUNT(rtc_command_sleep_parameters),
  .parent = NULL,
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false,
};


static command_t* rtc_subcommands[] = {&rtc_time_command, &rtc_sleep_command};

static command_t rtc_command = {
  .execution_ptr = NULL,
  .name = "rtc",
  .description = "",
  .prompt = "",
  .parameters = NULL,
  .parameter_count = 0,
  .children = (command_t**) &rtc_subcommands,
  .children_count = PARAM_COUNT(rtc_subcommands),
  .executable = false,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* rtc_commands[] = {&rtc_command};


void rtc_init(){
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  rtc_initialized = true;

  return;
}

void rtc_register_commands() {
  command_register(NULL, rtc_commands, 1);
}


bool rtc_format_time(char* buffer, uint8_t buffer_size)
{
  if (rtc_initialized) {
    HAL_RTC_GetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

    snprintf(
        buffer,
        buffer_size,
        "%i-%02d-%02d %02d:%02d:%02d::%03d",
        (rtc_date.Year + 2000),
        rtc_date.Month, rtc_date.Date,
        rtc_time.Hours, rtc_time.Minutes,
        rtc_time.Seconds,
        1000 - (int) (rtc_time.SubSeconds / 32.768)
    );

    return true;
  }
  else {
    return false;
  }
}

bool rtc_parse_date_string(RTC_DateTypeDef* rtc_date, RTC_TimeTypeDef* rtc_time, char* date_string){
  int year = 0;
  int month = 0;
  int date = 0;

  int hours = 0;
  int minutes = 0;
  int seconds = 0;

  sscanf(date_string, "%4d-%2d-%2d %2d:%2d:%2d", &year, &month, &date, &hours, &minutes, &seconds);

  // Rough check if parsed values are valid
  if(
    year >= 2000 && year < 2100 &&
    month >= 1 && month < 13 &&
    date >= 1 && date < 32 &&

    hours >= 0 && hours < 24 &&
    minutes >= 0 && minutes < 60 &&
    seconds >= 0 && seconds < 60
  ) {
    rtc_date->Year = (year - 2000);
    rtc_date->Month = month;
    rtc_date->Date = date;

    rtc_time->Hours = hours;
    rtc_time->Minutes = minutes;
    rtc_time->Seconds = seconds;

    return true;
  }
  else
  {
    return false;
  }
}

command_return_t rtc_time_command_handler(command_execution_t execution)
{
  // If no parameter was given, print time
  if(execution.value_count == 0)
  {
    char buffer[64];
    rtc_format_time((char*) buffer, sizeof(buffer));

    char hs_timer_buffer[16];
    snprintf(hs_timer_buffer, sizeof(hs_timer_buffer), " @ (%u)", (unsigned int) hs_timer_get_current_timestamp());
    strcat(buffer, hs_timer_buffer);

    cli_log_inline(buffer, CLI_LOG_LEVEL_DEFAULT, true, true, true);

    sprintf(buffer, "rtc ts: \t%llu", rtc_get_timestamp(true));
    cli_log_inline(buffer, CLI_LOG_LEVEL_DEFAULT, true, true, true);

    sprintf(buffer, "hs ts: \t\t%llu", hs_timer_get_current_timestamp());
    cli_log_inline(buffer, CLI_LOG_LEVEL_DEFAULT, true, true, true);

    return CMD_RET_SUCCESS;
  }
  // Else check if could set
  else if (execution.value_count == 1) {
    char* date_string = execution.values[0].value;

    rtc_date.WeekDay = 0;
    rtc_time.TimeFormat = 0;
    rtc_time.SubSeconds = 0;
    rtc_time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    rtc_time.StoreOperation = RTC_STOREOPERATION_RESET;

    if (rtc_parse_date_string(&rtc_date, &rtc_time, date_string))
    {
      HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);
      HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

      char buffer[64];
      rtc_format_time((char*) buffer, sizeof(buffer));
      cli_log_inline(buffer, CLI_LOG_LEVEL_DEFAULT, true, false, true);

      return CMD_RET_SUCCESS;
    }
    else
    {
      cli_log_inline("Wrong date-time format. Should be \"YYYY-MM-DD HH:MM:SS\" (e.g. \"2018-04-30 20:34\")", CLI_LOG_LEVEL_ERROR, true, false, true);

      char buffer[64];
      rtc_format_time((char*) buffer, sizeof(buffer));
      cli_log_inline(buffer, CLI_LOG_LEVEL_DEFAULT, true, false, true);

      return CMD_RET_FAILURE;
    }
  }
  else {
    return CMD_RET_FAILURE;
  }
}

uint64_t rtc_get_timestamp(bool hs_timer){
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
  if (hs_timer) {
    timestamp += hs_timer_offset;
  }

  return timestamp;
}

void rtc_set_alarm(uint64_t timestamp, void* callback){
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
void rtc_delay(uint32_t delay){
  if (rtc_initialized) {
    uint64_t start = rtc_get_timestamp(false);
    while(rtc_get_timestamp(false) < (start + (uint64_t) delay * HS_TIMER_FREQUENCY_MS))
    {
      __NOP();
      //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
    }
  }
}

void rtc_try_to_sleep() {
  if (hs_timer_scheduled && (hs_timer_scheduled_timestamp - hs_timer_get_current_timestamp()) >= HS_TIMER_FREQUENCY_MS * 4 && (hs_timer_scheduled_timestamp - hs_timer_get_current_timestamp()) <= INT64_MAX) {
    hs_timer_recovered_by_rtc = true;
    rtc_set_alarm(hs_timer_scheduled_timestamp - HS_TIMER_FREQUENCY_MS * 2, NULL);
    system_sleep(false);
  }
}

static command_return_t rtc_sleep_command_handler(command_execution_t execution)
{
  if (execution.value_count == 1) {
    uint32_t offset = strtol(execution.values[0].value, NULL, 10);
    uint64_t sleep_until_timestamp = hs_timer_get_current_timestamp() + offset;

    hs_timer_recovered_by_rtc = true;
    rtc_set_alarm(sleep_until_timestamp, NULL);
    system_sleep(false);

    return CMD_RET_SUCCESS;
  }
  else {
    return CMD_RET_FAILURE;
  }
}

/*
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  return;
}
*/

#endif /* HAL_RTC_MODULE_ENABLED */
