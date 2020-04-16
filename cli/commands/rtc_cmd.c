/*
 * rtc_cmd.c
 *
 *  Created on: May 8, 2018
 *      Author: marku
 */

#include "cli/cli.h"

#if CLI_ENABLE

#ifdef HAL_RTC_MODULE_ENABLED


extern RTC_HandleTypeDef hrtc;
extern bool rtc_initialized;
extern bool hs_timer_recovered_by_rtc;
extern RTC_TimeTypeDef rtc_time;
extern RTC_DateTypeDef rtc_date;


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

void rtc_register_commands() {
  command_register(NULL, rtc_commands, 1);
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

#endif /* HAL_RTC_MODULE_ENABLED */

#endif /* CLI_ENABLE */
