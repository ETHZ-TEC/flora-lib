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

command_return_t system_reset_command_handler(command_execution_t execution);
command_return_t system_bootloader_command_handler(command_execution_t execution);
command_return_t system_sleep_command_handler(command_execution_t execution);


static command_t system_reset_command = {
  .execution_ptr = &system_reset_command_handler,
  .name = "reset",
  .description = "Soft-resets the flora module",
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


static command_t system_bootloader_command = {
  .execution_ptr = &system_bootloader_command_handler,
  .name = "bootloader",
  .description = "Soft-resets the flora module and jumps into ROM bootloader",
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

static command_t system_sleep_command = {
  .execution_ptr = &system_sleep_command_handler,
  .name = "sleep",
  .description = "Set system into sleep mode",
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

static command_t* system_subcommands[] = {&system_reset_command, &system_bootloader_command, &system_sleep_command};

static command_t system_command = {
  .execution_ptr = NULL,
  .name = "system",
  .description = "Manages system state (power, operation) and functionality",
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

static command_t* system_commands[] = {&system_command};


void system_register_commands() {
  command_register(&system_command, system_subcommands, COMMAND_COUNT(system_subcommands));
  command_register(NULL, system_commands, 1);
}


command_return_t system_reset_command_handler(command_execution_t execution)
{
  system_reset();
  return CMD_RET_SUCCESS;
}

command_return_t system_bootloader_command_handler(command_execution_t execution)
{
  system_reset_into_bootloader();
  return CMD_RET_SUCCESS;
}

command_return_t system_sleep_command_handler(command_execution_t execution)
{
  system_sleep(false);
  return CMD_RET_SUCCESS;
}

#endif /* CLI_ENABLE */
