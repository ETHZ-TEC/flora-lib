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

extern bool cli_interactive_mode;

static command_return_t cli_help_command_handler(command_execution_t execution);
static command_return_t cli_interactive_command_handler(command_execution_t execution);


static parameter_t cli_help_parameter_command = {
    .name = "command",
    .description = "Shows help about the given command.",
    .type = CMD_PARAMETER_NONE,
    .options = NULL,
    .option_count = 0,
    .flag = '\0',
    .optional = true,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};


static parameter_t* cli_help_parameters[] = {
    &cli_help_parameter_command,
};


static command_t cli_help_command = {
  .execution_ptr = &cli_help_command_handler,
  .name = "help",
  .description = "Get help regarding the shell or a given command",
  .prompt = "",
  .parameters = (parameter_t**) cli_help_parameters,
  .parameter_count = sizeof(cli_help_parameters) / sizeof(parameter_t*),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = true,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};


static parameter_t cli_interactive_parameter_value = {
    .name = "value",
    .description = "Sets the interactive mode accordingly.",

    .type = CMD_PARAMETER_BOOL,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = true,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};


static parameter_t* cli_interactive_parameters[] = {
    &cli_interactive_parameter_value,
};


static command_t cli_interactive_command = {
  .execution_ptr = &cli_interactive_command_handler,
  .name = "interactive",
  .description = "Enables or disables the interactive mode for CLI (toggles mode if no parameter is given). Use 'config set interactive [0,1]' to set persistent configuration.",
  .prompt = "",
  .parameters = (parameter_t**) cli_interactive_parameters,
  .parameter_count = PARAM_COUNT(cli_interactive_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = true,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t* cli_commands[] = {&cli_help_command, &cli_interactive_command};

void cli_register_commands(){
  command_register(NULL, cli_commands, COMMAND_COUNT(cli_commands));
}

static command_return_t cli_help_command_handler(command_execution_t execution) {
  if (execution.value_count > 0) {
    char buf[CLI_MAX_INPUT+1] = {0};
    int i;
    for (i = 0; i < execution.value_count; i++) {
      strcat(buf, execution.values[i].value);
      if(i != execution.value_count - 1) {
        strcat(buf, " ");
      }
    }

    command_execution_t execution = command_get(NULL, buf, strlen(buf));
    if (!execution.invalid) {
      if (execution.command->executable) {
        command_print_description(execution.command);
      }
      else {
        command_print_subcommands(execution.command, 0);
      }

      return CMD_RET_SUCCESS;
    }
    else {
      cli_log_inline("Command is not valid!", CLI_LOG_LEVEL_ERROR, true, false, true);
      return CMD_RET_FAILURE;
    }
  }
  else {
    cli_println("List of commands:\r\n");
    command_print_subcommands(NULL, 0);
  }

  return CMD_RET_SUCCESS;
}

static command_return_t cli_interactive_command_handler(command_execution_t execution) {
  int count = command_get_unnamed_parameter_count(&execution);
  if (count == 1) {
    cli_interactive_mode = command_cast_parameter_to_bool(command_get_unnamed_parameter_at_index(&execution, 0));
    // if (VT100_OUTPUT) {
    //   if (cli_interactive_mode) {
    //     cli_print(CLI_INTERACTIVE_NACK);
    //   }
    //   else {
    //     cli_print(CLI_INTERACTIVE_ACK);
    //   }
    // }
    return CMD_RET_SUCCESS;
  }
  else if (!count) {
    cli_interactive_mode = !cli_interactive_mode;
    // if (VT100_OUTPUT) {
    //   if (cli_interactive_mode) {
    //     cli_print(CLI_INTERACTIVE_NACK);
    //   }
    //   else {
    //     cli_print(CLI_INTERACTIVE_ACK);
    //   }
    // }
    return CMD_RET_SUCCESS;
  }
  else {
    return CMD_RET_FAILURE;
  }
}

#endif /* CLI_ENABLE */
