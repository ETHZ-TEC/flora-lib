/*
 * system_cmd.c
 *
 *  Created on: 01.05.2018
 *      Author: marku
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
