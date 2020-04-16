/*
 * config_cmd.c
 *
 *  Created on: May 9, 2018
 *      Author: marku
 */

#include "cli/cli.h"

#if CLI_ENABLE

command_return_t config_get_command_handler(command_execution_t execution);
command_return_t config_set_command_handler(command_execution_t execution);
command_return_t config_load_command_handler(command_execution_t execution);
command_return_t config_store_command_handler(command_execution_t execution);
command_return_t config_command_handler(command_execution_t execution);

static command_t config_get_command = {
  .execution_ptr = &config_get_command_handler,
  .name = "get",
  .description = "Gets all or a certain config entry",
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

static command_t config_set_command = {
  .execution_ptr = &config_set_command_handler,
  .name = "set",
  .description = "Sets a certain config entry",
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

static command_t config_load_command = {
  .execution_ptr = &config_load_command_handler,
  .name = "load",
  .description = "Loads the config from flash",
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

static command_t config_store_command = {
  .execution_ptr = &config_store_command_handler,
  .name = "store",
  .description = "Stores the config in flash",
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


static command_t* config_subcommands[] = {&config_store_command, &config_load_command, &config_set_command, &config_get_command};

static command_t config_command = {
  .execution_ptr = NULL,
  .name = "config",
  .description = "Manages the system's config",
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

static command_t* config_commands[] = {&config_command};


void config_register_commands()
{
  command_register(&config_command, config_subcommands, COMMAND_COUNT(config_subcommands));
  command_register(NULL, config_commands, COMMAND_COUNT(config_commands));
}




command_return_t config_get_command_handler(command_execution_t execution) {
  if (execution.value_count == 1)
  {
    config_print(execution.values[0].value);
  }
  else
  {
    config_print("");
  }

  return CMD_RET_SUCCESS;
}
command_return_t config_set_command_handler(command_execution_t execution) {

  if (execution.value_count == 2)
  {
    uint32_t value = atoi(execution.values[1].value);
    config_set_entry(execution.values[0].value, value);
  }

  return CMD_RET_SUCCESS;
}
command_return_t config_load_command_handler(command_execution_t execution) {
  config_load();
  config_print("");

  return CMD_RET_SUCCESS;
}
command_return_t config_store_command_handler(command_execution_t execution)
{
  config_store(NULL);
  config_load();
  config_print("");

  return CMD_RET_SUCCESS;
}

#endif /* CLI_ENABLE */
