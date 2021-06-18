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

static char null_string[] = "";

bool command_initialized = false;



command_t root = {
    .execution_ptr = NULL,
    .name = "",
    .description = "FlOS 0.1 Shell",
    .prompt = "flora >",
    .parameters = NULL,
    .parameter_count = 0,
    .children = NULL,
    .children_count = 0,
    .executable = false,
    .built_in = false,
    .has_prompt = true,
    .hidden = false,
    .children_only_in_prompt_invokable = true
};


command_t* current_prompt = &root;


static char buffer[CLI_MAX_INPUT + 1]; // Length of input plus terminating null byte


static void command_parse_input(command_execution_t* execution, char* input, uint16_t length);


void command_init()
{
  command_initialized = true;
}

void command_register(command_t* parent, command_t** commands, uint8_t command_count) {
  if (parent == NULL) {
    parent = &root;
  }

  uint8_t old_count = parent->children_count;
  uint8_t new_count = old_count + command_count;

  command_t** children = (command_t**) realloc(parent->children, new_count * sizeof(command_t*));
  memcpy(children + old_count, commands, command_count * sizeof(command_t*));
  parent->children = children;
  parent->children_count = new_count;

  int i;
  for (i = 0; i < command_count; i++) {
    commands[i]->parent = parent;
  }


}

uint8_t command_search(command_t* search_root, char* expansion, char* search, uint16_t search_length, command_t** results)
{
  // If no root was defined, assume it to be the current prompt level
  if (search_root == NULL)
  {
    search_root = current_prompt;
    results[0] = current_prompt;
  }


  if(search_length)
  {
    // Remove prefixing spaces
    while (search_length && isspace(search[0])) {
      search_length--;
      search++;
    }

    if (!search_length)
    {
      return 0;
    }


    // Get size of first word in search text
    int i = 0;
    for (i = 0; i < search_length; i++) {
      if(isspace(search[i]))
        break;
    }

    if (i <= 0 || i > (sizeof(buffer) - 1)) {
      cli_log_inline("Subcommand too long!", CLI_LOG_LEVEL_ERROR, true, true, true);
      return 0;
    }

    // Copy first word in search text to temporary buffer
    memcpy((char*) buffer, search, i);
    buffer[i] = '\0';


    bool matching[16] = {false};
    uint8_t matching_count = 0;

    __DSB();

    // Get index of matching commands in children set
    int j;


    if (search_root->children_count)
    {
      for (j = 0; j < search_root->children_count; j++) {

        if (strncasecmp(search_root->children[j]->name, buffer, i) == 0)
        {
          matching[j] = true;
          command_t* command = search_root->children[j];
          results[matching_count] = command;
          matching_count++;
        }
      }
    }
    else
    {
      return 0;
    }

    if (matching_count)
    {
      search_length -= i;
      search += i;

      uint8_t trailing_spaces_count = 0;
      // Remove trailing spaces
      while (search_length && isspace(search[0])) {
        trailing_spaces_count++;
        search_length--;
        search++;
      }

      if (matching_count == 1)
      {
        command_t* command = results[0];

        for (j = 0; j < search_root->children_count; j++)
        {
          if (matching[j])
            break;
        }

        strcat(expansion, command->name);

        if (command->children_count && search_length)
        {
          strcat(expansion, " ");
          return command_search(command, expansion, search, search_length, results);
        }

        else {
          if(search_length) {
            strcat(expansion, " ");
            strncat(expansion, search, search_length);
          }
          else if (trailing_spaces_count) {
            strcat(expansion, " ");
          }

          return 1;
        }


      }
      else
      {
        if(search_length)
        {
          cli_log_inline("Invalid command path!", CLI_LOG_LEVEL_ERROR, true, false, true);
          return 0;
        }
        else
        {
          strncat(expansion, search-i, i);

          // Autocomplete
          bool different = false;
          char tmp;

          while (!different)
          {
            tmp = '\0';

            for (j = 0; j < matching_count; j++)
            {
              char tmp_cmd = results[j]->name[j+i];

              if (tmp_cmd != '\0')
              {
                if (tmp == '\0')
                {
                  tmp = tmp_cmd;
                }

                if (tmp != tmp_cmd || tmp_cmd == '\0')
                {
                  different = true;
                  break;
                }
              }
              else
              {
                different = true;
                break;
              }
            }

            if (!different)
              strncat(expansion, &tmp, 1);
          }

          return matching_count;
        }
      }
    }
    else {
      cli_log_inline("Invalid command path!", CLI_LOG_LEVEL_ERROR, true, false, true);
      return 0;
    }
  }
  else
    return 0;
}

command_execution_t command_get(command_t* search_root, char* search, uint16_t search_length)
{
  command_execution_t execution = { .command = search_root, .values = {{.parameter = NULL, .value = null_string, .length = 0}}, .value_count = 0, .invalid = true };

  if(search_length)
  {
    // If no root was defined, assume it to be the current prompt level
    if (search_root == NULL)
    {
      search_root = current_prompt;
      execution.command = search_root;
    }

    // Remove prefixing spaces
    while (search_length && isspace(search[0])) {
      search_length--;
      search++;
    }

    if (!search_length)
    {
      return execution;
    }


    // Get index of last character of first word in search text
    int i = 0;
    for (i = 0; i < search_length; i++) {
      if(isspace(search[i]))
        break;
    }

    if (i > (sizeof(buffer) - 1)) {
      return execution;
    }

    // Copy first word in search text to temporary buffer
    memcpy((char*) buffer, search, i);
    buffer[i] = '\0';


    // Get index of matching commands in children set
    uint8_t matching_count = 0;
    command_t* command = NULL;


    if (search_root->children_count)
    {
      int j;
      for (j = 0; j < search_root->children_count; j++) {
        if (strncasecmp(search_root->children[j]->name, buffer, i) == 0)
        {
          command = search_root->children[j];
          matching_count++;
        }
      }
    }

    // If command was not found handle parameters
    if (matching_count == 0)
    {
      command_parse_input(&execution, search, search_length);

      if (execution.command != current_prompt) {
        execution.invalid = false;
      }

      return execution;
    }
    else if(matching_count == 1) {
      search_length -= i;
      search += i;

      // Remove trailing spaces
      while (search_length && isspace(search[0])) {
        search_length--;
        search++;
      }

      // If there is still further search text, search in the matching command recursively, return the matching command else
      if (search_length > 0)
      {
        if (!command->children_only_in_prompt_invokable || (command->children_only_in_prompt_invokable && current_prompt == search_root))
          return command_get(command, search, search_length);
        else {
          cli_set_color(CLI_LOG_LEVEL_ERROR);
          cli_println("Subcommand only accessible in sub-prompt!");
          cli_set_color(CLI_LOG_LEVEL_DEFAULT);
          execution.value_count = 0;
          execution.invalid = true;

          return execution;
        }
      }
      else
      {
        execution.invalid = false;
        execution.command = command;
        return execution;
      }
    }
    else {
      execution.command = search_root;
      execution.invalid = true;

      return execution;
    }
  }
  else {
    return execution;
  }
}

command_t* command_get_prompt()
{
  if (command_initialized)
    return current_prompt;
  else
    return NULL;
}

void command_print_parameter_usage(parameter_t* parameter) {
  if (parameter->optional) {
    cli_print("[");
  }
  else if (parameter->has_value && parameter->requires_flag_or_name) {
    cli_print("(");
  }

  if (parameter->has_flag) {
    cli_print("-");
    cli_nprint(&(parameter->flag), 1);

    if (parameter->requires_flag_or_name) {
      cli_print(",");
    }
    else if (parameter->has_value) {
      cli_print(" ");
    }
  }

  if (parameter->requires_flag_or_name) {
    cli_print("--");
    cli_print(parameter->name);
    if (parameter->has_value) {
      cli_print(" ");
    }

  }

  if (parameter->has_value) {
    if (parameter->requires_flag_or_name) {
      switch (parameter->type) {
      case CMD_PARAMETER_NONE:
        cli_print("<string>");
        break;
      case CMD_PARAMETER_INTEGER:
        cli_print("<int>");
        break;
      case CMD_PARAMETER_BOOL:
        cli_print("<bool>");
        break;
      case CMD_PARAMETER_DATE:
        cli_print("<date>");
        break;
      case CMD_PARAMETER_TIME:
        cli_print("<time>");
        break;
      case CMD_PARAMETER_FREQUENCY:
        cli_print("<freq>");
        break;
      case CMD_PARAMETER_POWER:
        cli_print("<power>");
        break;
      default:
        break;

      }
    }
    else {
      cli_print(parameter->name);
    }
  }


  if (parameter->optional) {
    cli_print("]");
  }
  else if (parameter->has_value && parameter->requires_flag_or_name) {
    cli_print(")");
  }

  cli_print(" ");
}

void command_print_usage(command_t* cmd) {
  cli_print("Usage: ");
  command_print_full_name(cmd);

  int i;
  for (i = 0; i < cmd->parameter_count; i++) {
    command_print_parameter_usage(cmd->parameters[i]);
  }

  cli_println("");
}

void command_print_full_name(command_t* cmd) {
  if(cmd->parent != NULL && cmd->parent != current_prompt) {
    command_print_full_name(cmd->parent);
  }

  cli_print(cmd->name);
  cli_print(" ");
}

void command_print_description(command_t* cmd) {
  cli_print(cmd->description);
  cli_print("\r\n\r\n");

  command_print_usage(cmd);

  command_print_parameters(cmd);
}

void command_print_parameters(command_t* cmd) {
  int i;
  for (i = 0; i < cmd->parameter_count; i++) {

    cli_print("\t");

    if (cmd->parameters[i]->has_flag) {
      cli_print("-");
      cli_nprint(&cmd->parameters[i]->flag, 1);
    }

    cli_print("\t");
    cli_print(cmd->parameters[i]->name);

    if (cmd->parameters[i]->type != CMD_PARAMETER_NONE) {
      cli_print(" ");

      switch (cmd->parameters[i]->type) {
      case CMD_PARAMETER_NONE:
        break;
      case CMD_PARAMETER_INTEGER:
        cli_print("<int>");
        break;
      case CMD_PARAMETER_BOOL:
        cli_print("<bool>");
        break;
      case CMD_PARAMETER_DATE:
        cli_print("<date>");
        break;
      case CMD_PARAMETER_TIME:
        cli_print("<time>");
        break;
      case CMD_PARAMETER_FREQUENCY:
        cli_print("<freq>");
        break;
      case CMD_PARAMETER_POWER:
        cli_print("<power>");
        break;
      default:
        break;
      }
    }


    cli_print(":\t");
    cli_println(cmd->parameters[i]->description);
  }
}


value_t* command_get_parameter(command_execution_t* execution, char tag)
{
  int i;
  for (i = 0; i < execution->value_count; i++) {
    if (execution->values[i].parameter != NULL && execution->values[i].parameter->flag == tag) {
      return &(execution->values[i]);
    }
  }

  return NULL;
}

bool command_cast_parameter_to_bool(value_t* parameter)
{
  if (strcasecmp(parameter->value, "true") == 0) {
    return true;
  }
  else if (strcasecmp(parameter->value, "false") == 0) {
    return false;
  }
  else {
    int tmp = strtol(parameter->value, NULL, 10);

    if (tmp) {
      return true;
    }
    else {
      return false;
    }
  }
}

uint8_t command_get_unnamed_parameter_count(command_execution_t* execution) {
  if (execution->value_count) {
    int counter = 0;
    int i;
    for (i = 0; i < execution->value_count; i++) {
      if (
        (execution->values[i].parameter != NULL && !execution->values[i].parameter->requires_flag_or_name) ||
        (execution->values[i].parameter == NULL)
      ) {
        counter++;
      }
    }

    return counter;
  }
  else {
    return 0;
  }
}

value_t* command_get_unnamed_parameter_at_index(command_execution_t* execution, uint8_t index)
{
  if (index < execution->value_count) {
    int counter = 0;
    int i;
    for (i = 0; i < execution->value_count; i++) {
      if (
          (execution->values[i].parameter != NULL && !execution->values[i].parameter->requires_flag_or_name) ||
          (execution->values[i].parameter == NULL)
      ) {
        if (counter == index) {
          return &(execution->values[i]);
        }
        else {
          counter++;
        }
      }
    }
  }

  return NULL;
}

parameter_t* command_get_parameter_from_command(command_execution_t* execution, char tag)
{
  if (execution->command != NULL) {
    int i;
    for (i = 0; i < execution->command->parameter_count; i++) {
      if (execution->command->parameters[i]->flag == tag) {
        return execution->command->parameters[i];
      }
    }
  }

  return NULL;
}

parameter_t* command_get_parameter_from_name(command_execution_t* execution, char* name, uint16_t size) {
  if (execution->command != NULL) {
    int i;
    for (i = 0; i < execution->command->parameter_count; i++) {
      if (strlen(execution->command->parameters[i]->name) == size && strncmp(execution->command->parameters[i]->name, name, size) == 0) {
        return execution->command->parameters[i];
      }
    }
  }

  return NULL;
}

void command_parse_input(command_execution_t* execution, char* input, uint16_t length)
{
  execution->value_count = 0;

  bool in_flag = false;

  while (length)
  {
    // Remove prefixing spaces
    while (length && isspace(input[0])) {
      length--;
      input++;
    }

    if(length) {
      int i;

      char flag = '\0';
      char escaped = '\0';
      bool in_escape = false;
      bool in_escape_code = false;

      for (i = 0; i < length; i++) {
        if(!escaped)
        {
          if(isspace(input[i])) {
            break;
          }

          else if(i == 0 && input[0] == '-') {
            if (in_flag) {
              execution->value_count++;
              in_flag = false;
            }

            input++;
            length--;

            if(input[0] >= 48 && input[0] < 58)
            {
              in_flag = false;
              input--;
              length++;
              i++;
              continue;
            }

            while (input[i] != '\0' && !isspace(input[i]) && input[i] != '-') {
              flag = input[i];
              input++;
              length--;

              parameter_t* parameter = command_get_parameter_from_command(execution, flag);
              if (parameter != NULL) {
                if (parameter->has_value){
                  if (!isspace(input[i]) || input[i] == '\0') {
                    if (execution->value_count < 16) {
                      execution->values[execution->value_count].length = 0;
                      execution->values[execution->value_count].parameter = parameter;
                      execution->values[execution->value_count].value = null_string;
                      execution->value_count++;
                    }
                  }
                  else {
                    execution->values[execution->value_count].length = 0;
                    execution->values[execution->value_count].parameter = parameter;
                    execution->values[execution->value_count].value = null_string;
                    in_flag = true;
                    break;
                  }
                }
                else {
                  if (execution->value_count < 16) {
                    execution->values[execution->value_count].length = 0;
                    execution->values[execution->value_count].parameter = parameter;
                    execution->values[execution->value_count].value = null_string;
                    execution->value_count++;
                  }
                }
              }
            }

            if (input[i] == '-') {
              i = 0;

              input++;
              length--;

              while (input[i] != '\0' && !isspace(input[i]) && input[i] != '-') {
                i++;
              }

              if (i) {
                parameter_t* parameter = command_get_parameter_from_name(execution, input, i);

                if (parameter != NULL) {
                  if (parameter->has_value){
                    if (!isspace(input[i]) || input[i] == '\0') {
                      if (execution->value_count < 16) {
                        execution->values[execution->value_count].length = 0;
                        execution->values[execution->value_count].parameter = parameter;
                        execution->values[execution->value_count].value = null_string;
                        execution->value_count++;
                      }
                    }
                    else {
                      execution->values[execution->value_count].length = 0;
                      execution->values[execution->value_count].parameter = parameter;
                      execution->values[execution->value_count].value = null_string;
                      break;
                    }
                  }
                  else {
                    if (execution->value_count < 16) {
                      execution->values[execution->value_count].length = 0;
                      execution->values[execution->value_count].parameter = parameter;
                      execution->values[execution->value_count].value = null_string;
                      execution->value_count++;
                    }
                  }
                }
              }

              input += i;
              length -= i;
            }

            while (length && isspace(input[0])) {
              length--;
              input++;
            }

            i = 0;
            break;
          }

          else if(input[i] == '"') {
            in_flag = false;

            if (i > 0) {
              break;
            }
            else {
              escaped = '"';
              in_escape = true;
            }
          }
          else if(input[i] == '\'') {
            in_flag = false;

            if (i > 0)
            {
              break;
            }
            else {
              escaped = '\'';
              in_escape = true;
            }
          }
        }
        else {
          if(input[i] == escaped && !in_escape_code) {
            in_escape = false;
            i++;
            break;
          }
          else {
            in_escape_code = (!in_escape_code && input[i] == '\\');
          }
        }
      }

      if (!i) {
        continue;
      }


      if (escaped) {
        input++;
        i -= 2;
      }


      if (in_escape) {
        cli_set_color(CLI_LOG_LEVEL_ERROR);
        cli_println("Command parameter/s are too long or not correctly escaped!");
        cli_set_color(CLI_LOG_LEVEL_DEFAULT);
        execution->value_count = 0;
        execution->invalid = true;
        return;
      }

      if (execution->value_count < 16) {
        execution->values[execution->value_count].value = input;
        execution->values[execution->value_count].length = i;
        in_flag = false;

        if(escaped) {
          execution->values[execution->value_count].value[i] = '\0';
          length -= i + 2;
          input += i + 1;

          // Remove trailing spaces
          while (length && isspace(input[0])) {
            length--;
            input++;
          }
        }
        else {
          length -= i;
          input += i;

          // Remove trailing spaces
          while (length && isspace(input[0])) {
            length--;
            input++;
          }

          execution->values[execution->value_count].value[i] = '\0';
        }

        execution->value_count++;


      }
      else
      {
        cli_set_color(CLI_LOG_LEVEL_ERROR);
        cli_println("Command has too many parameters!");
        cli_set_color(CLI_LOG_LEVEL_DEFAULT);
        execution->value_count = 0;
        execution->invalid = true;

        return;
      }
    }
    else
    {
      return;
    }
  }


  return;
}

bool command_execution_check(command_execution_t execution)
{
  if (execution.command != NULL && !execution.invalid && execution.command->executable)
  {
    return true;
  }
  else
  {
    cli_log_inline("Command is not valid or executable!", CLI_LOG_LEVEL_ERROR, true, false, true);

    return false;
  }
}


void command_print_subcommands(command_t* command, uint8_t level)
{
  if (command == NULL)
  {
    command = current_prompt;
  }
  else
  {
    int j;
    for (j = 0; j < level; j++) {
      cli_print("\t");
    }

    cli_set_color(CLI_LOG_LEVEL_INFO);
    cli_print(command->name);
    cli_set_color(CLI_LOG_LEVEL_DEFAULT);
    if(command->description)
    {
      j += (strlen(command->name) / 8);

      if (j < 5) {
        for (;j < 5; j++) {
          cli_print("\t");
        }
      }
      else {
        cli_print("\t");
      }
      cli_println(command->description);
    }
    else
    {
      cli_println("");
    }
  }

  int i;
  for (i = 0; i < command->children_count; i++)
  {
    command_print_subcommands(command->children[i], level + 1);
  }
}

#endif /* CLI_ENABLE */
