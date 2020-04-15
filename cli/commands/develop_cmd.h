/*
 * develop_cmd.h
 *
 *  Created on: May 7, 2018
 *      Author: marku
 */

#ifndef CLI_COMMANDS_DEVELOP_CMD_H_
#define CLI_COMMANDS_DEVELOP_CMD_H_

typedef enum develop_command_s {
  CMD_NONE = 0x00,
  CMD_LINKTESTMODE,
  CMD_TESTLINK
} develop_command_t;

void develop_register_commands();

#endif /* CLI_COMMANDS_DEVELOP_CMD_H_ */
