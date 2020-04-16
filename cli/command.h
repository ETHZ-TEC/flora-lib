/*
 * command.h
 *
 *  Created on: 26.04.2018
 *      Author: marku
 */

#ifndef CLI_COMMAND_H_
#define CLI_COMMAND_H_


/* include all commands */
#include "cli/commands/gloria_cmd.h"
#include "cli/commands/develop_cmd.h"
#include "cli/commands/slwb_cmd.h"
#include "cli/commands/system_cmd.h"
#include "cli/commands/radio_cmd.h"
#include "cli/commands/config_cmd.h"
#include "cli/commands/led_cmd.h"
#include "cli/commands/cli_cmd.h"
#include "cli/commands/test_cmd.h"
#include "cli/commands/rtc_cmd.h"


#define MAX_CHILDREN_COMMANDS 20

#define PARAM_COUNT(params)  (sizeof(params) / sizeof(parameter_t*))
#define COMMAND_COUNT(commands)  (sizeof(commands) / sizeof(command_t*))

#if defined(CLI_VERBOSE)
#define VERBOSE_STRING(string) (string)
#else
#define VERBOSE_STRING(string) ("")
#endif


typedef enum command_return_s {
  CMD_RET_SUCCESS = 0x00,
  CMD_RET_FAILURE,
  CMD_RET_PENDING
} command_return_t;


typedef enum {
  /** There are missing arguments for the command */
  CMD_ERR_ARGCOUNT = 0x00,
  /** The program received an argument that is out of range */
  CMD_ERR_OUTOFRANGE,
  /** The program received an argument with a value different than expected */
  CMD_ERR_VALUE,
  /** Invalid action requested for the current state */
  CMD_ERR_ACTION,
  /** Cannot parse the user input */
  CMD_ERR_PARSE,
  /** Cannot access storage device or memory device */
  CMD_ERR_STORAGE,
  /** IO device error caused program interruption */
  CMD_ERR_IO,
  /** Other kinds of errors */
  CMD_ERROR_UNKNOWN,
} command_error_t;

typedef enum {
  CMD_PARAMETER_NONE = 0x00,
  CMD_PARAMETER_INTEGER,
  CMD_PARAMETER_BOOL,
  CMD_PARAMETER_DATE,
  CMD_PARAMETER_TIME,
  CMD_PARAMETER_FREQUENCY,
  CMD_PARAMETER_POWER
} parameter_type_t;


typedef struct {
  char* name;
  char* description;
} option_t;

typedef struct {
  char* name;
  char* description;

  parameter_type_t type;
  option_t** options;
  uint8_t option_count;
  char flag;

  bool optional : 1;
  bool has_value : 1;
  bool has_flag : 1;
  bool requires_flag_or_name : 1;
  bool has_countable_options : 1;
} parameter_t;

typedef struct value_s {
  parameter_t* parameter;
  char* value;
  uint16_t length;
} value_t;


typedef struct command_execution_s {
  struct command_s* command;
  struct value_s values[16];
  uint8_t value_count;
  bool invalid;
} command_execution_t;

typedef command_return_t (*command_execution_ptr) ( struct command_execution_s execution);

typedef struct command_s {
  command_execution_ptr execution_ptr;
  char* name;
  char* description;
  char* prompt;

  parameter_t** parameters;
  uint8_t parameter_count;

  struct command_s* parent;

  struct command_s** children;
  uint8_t children_count;

  bool executable : 1;
  bool built_in: 1; // e.g. exit, quit
  bool has_prompt : 1;
  bool hidden : 1;
  bool children_only_in_prompt_invokable : 1;

} command_t;


void command_init();
void command_register(command_t* parent, command_t** commands, uint8_t command_count);
uint8_t command_search(command_t* search_root, char* expansion, char* search, uint16_t search_length, command_t** results);
command_execution_t command_get(command_t* search_root, char* search, uint16_t search_length);
void command_print_usage(command_t* cmd);
void command_print_full_name(command_t* cmd);
command_t* command_get_prompt();
void command_print_description(command_t* cmd);
void command_print_parameters(command_t* cmd);
value_t* command_get_parameter(command_execution_t* execution, char tag);
bool command_cast_parameter_to_bool(value_t* parameter);
uint8_t command_get_unnamed_parameter_count(command_execution_t* execution);
value_t* command_get_unnamed_parameter_at_index(command_execution_t* execution, uint8_t index);
bool command_execution_check(command_execution_t execution);
void command_print_subcommands(command_t* command, uint8_t level);
parameter_t* command_get_parameter_from_name(command_execution_t* execution, char* name, uint16_t size);


#endif /* CLI_COMMAND_H_ */
