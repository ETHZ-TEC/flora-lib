/*
 * led_cmd.c
 *
 *  Created on: May 8, 2018
 *      Author: marku
 */

#include "flora_lib.h"

#if CLI_ENABLE

extern bool leds_initialized;

command_return_t led_blink_command_handler(command_execution_t execution);
command_return_t led_enable_command_handler(command_execution_t execution);

static command_t led_blink_command = {
  .execution_ptr = &led_blink_command_handler,
  .name = "blink",
  .description = "Briefly flashes up the red led light. Useful for identification",
  .prompt = "",
  .parameters = NULL,
  .parameter_count = 0,
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static parameter_t led_enable_parameter_value = {
    .name = "value",
    .description = "Sets the leds according.",

    .type = CMD_PARAMETER_BOOL,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = false,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};


static parameter_t* led_enable_parameters[] = {
    &led_enable_parameter_value,
};

static command_t led_enable_command = {
  .execution_ptr = &led_enable_command_handler,
  .name = "enable",
  .description = "Enable/disable leds (useful for power measurements)",
  .prompt = "",
  .parameters = (parameter_t**) led_enable_parameters,
  .parameter_count = PARAM_COUNT(led_enable_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* led_subcommands[] = {
  &led_blink_command,
  &led_enable_command,
};

static command_t led_command = {
  .execution_ptr = NULL,
  .name = "led",
  .description = "",
  .prompt = "",
  .parameters = NULL,
  .parameter_count = 0,
  .children = NULL,
  .children_count = 0,
  .executable = false,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* led_commands[] = {&led_command};

void led_register_commands()
{
  command_register(&led_command, led_subcommands, COMMAND_COUNT(led_subcommands));
  command_register(NULL, led_commands, 1);
}

command_return_t led_blink_command_handler(command_execution_t execution)
{
  if (execution.value_count == 1)
  {
    uint16_t blink_time = (uint16_t) strtol(execution.values[0].value, NULL, 10) * 10; // s  (to 100 ms steps)
    led_set_event_blink(blink_time, 0);
  }
  else
  {
    led_set_event_blink(0, 0);
  }

  return CMD_RET_SUCCESS;
}

command_return_t led_enable_command_handler(command_execution_t execution)
{
  if (execution.value_count == 1)
  {
    leds_initialized = command_cast_parameter_to_bool(command_get_unnamed_parameter_at_index(&execution, 0));
    if (leds_initialized) {
      // initialize leds and turn on pulsing led
      leds_init();
    }
    else {
      // turn off all leds
      leds_deinit();
    }
  }
  else
  {
    leds_initialized = !leds_initialized;
    if (leds_initialized) {
      // initialize leds and turn on pulsing led
      leds_init();
    }
    else {
      // turn off all leds
      leds_deinit();
    }
  }

  return CMD_RET_SUCCESS;
}

#endif /* CLI_ENABLE */
