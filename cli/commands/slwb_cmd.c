/*
 * slwb_cmd.c
 *
 *  Created on: Dec 7, 2018
 *      Author: kelmicha
 */

#include "cli/cli.h"

#if CLI_ENABLE

static command_return_t slwb_start_command_handler(command_execution_t execution);


static parameter_t slwb_start_parameter_lr_mod = {
    .name = "lr_modulation",
    .description = "defines the modulation to use for long range communication",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'l',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t slwb_start_parameter_mod = {
    .name = "modulation",
    .description = "defines the modulation",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'm',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t slwb_start_parameter_lr_pwr = {
    .name = "lr_power",
    .description = "defines the power used for long range communication",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'o',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t slwb_start_parameter_pwr = {
    .name = "power",
    .description = "defines the power",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'p',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t slwb_start_parameter_rnd = {
    .name = "round_period",
    .description = "defines the round period in s",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'r',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


static parameter_t* slwb_start_parameters[] = {
    &slwb_start_parameter_lr_mod,
    &slwb_start_parameter_mod,
    &slwb_start_parameter_lr_pwr,
    &slwb_start_parameter_pwr,
    &slwb_start_parameter_rnd,
};

static command_t slwb_start_command = {
  .execution_ptr = &slwb_start_command_handler,
  .name = "start",
  .description = "Initiates slwb.",
  .prompt = "",
  .parameters = (parameter_t**) slwb_start_parameters,
  .parameter_count = PARAM_COUNT(slwb_start_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_return_t slwb_start_command_handler(command_execution_t execution) {
#ifdef RADIO_LOG
  set_radio_log(true);
#else
  set_radio_log(false);
#endif
  value_t* param = NULL;

  uint8_t lr_mod = 0;
  param = command_get_parameter(&execution, 'l');
  if (param != NULL) {
    lr_mod = strtol(param->value, NULL, 10);
  }
  else {
    return CMD_RET_FAILURE;
  }

  uint8_t mod = 0;
  param = command_get_parameter(&execution, 'm');
  if (param != NULL) {
    mod = strtol(param->value, NULL, 10);
  }
  else {
    return CMD_RET_FAILURE;
  }

  uint8_t lr_pwr = slwb_default_power_levels[lr_mod];
  param = command_get_parameter(&execution, 'o');
  if (param != NULL) {
    lr_pwr = strtol(param->value, NULL, 10);
  }

  uint8_t pwr = slwb_default_power_levels[mod];
  param = command_get_parameter(&execution, 'p');
  if (param != NULL) {
    pwr = strtol(param->value, NULL, 10);
  }

  uint8_t rnd = 0;
  param = command_get_parameter(&execution, 'r');
  if (param != NULL) {
    rnd = strtol(param->value, NULL, 10);
  }
  else {
    return CMD_RET_FAILURE;
  }

  slwb_start(lr_mod, mod, lr_pwr, pwr, rnd);

  return CMD_RET_SUCCESS;
}


static command_t* slwb_subcommands[] = {&slwb_start_command};

static command_t slwb_command = {
  .execution_ptr = NULL,
  .name = "slwb",
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

static command_t* slwb_commands[] = {&slwb_command};

void slwb_register_commands() {
  command_register(&slwb_command, slwb_subcommands, COMMAND_COUNT(slwb_subcommands));
  command_register(NULL, slwb_commands, 1);
}

#endif /* CLI_ENABLE */
