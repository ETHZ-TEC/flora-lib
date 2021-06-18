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

#include "flora_lib.h"

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
  //set_radio_log(false);  FIXME
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

  slwb_start(lr_mod, mod, lr_pwr, pwr, rnd);    // FIXME

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
