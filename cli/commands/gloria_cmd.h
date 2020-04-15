/*
 * gloria_cmd.h
 *
 *  Created on: Aug 7, 2018
 *      Author: marku
 */

#ifndef CLI_COMMANDS_GLORIA_CMD_H_
#define CLI_COMMANDS_GLORIA_CMD_H_

#include "protocol/gloria/gloria.h"

typedef enum {
  GLORIA_MESSAGE,
} gloria_msg_types;

void gloria_register_commands(void);
void gloria_print_flood_periodic(void);
void gloria_print_flood(gloria_flood_t *print_flood);

#endif /* CLI_COMMANDS_GLORIA_CMD_H_ */
