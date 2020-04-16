/*
 * cli.h
 *
 *  Created on: Apr 20, 2018
 *      Author: marku
 */

#ifndef CLI_CLI_H_
#define CLI_CLI_H_


/* include lib and global defines */
#include "flora_lib.h"

#ifndef CLI_ENABLE
#define CLI_ENABLE                  1
#endif /* CLI_ENABLE */


#define CLI_VERSION_STRING          "FlOS CLI 0.1"
#define CLI_MAX_INPUT               (304 - 1)

#define CLI_INTERACTIVE_MODE_CTRL   '\x14' // ASCII Record Separator (RS)

#ifndef CLI_INTERACTIVE_ENABLE
#define CLI_INTERACTIVE_ENABLE      1
#endif /* CLI_INTERACTIVE_ENABLE */


#define CLI_ASCII_NUL               0x00
#define CLI_ASCII_BEL               0x07
#define CLI_ASCII_BS                0x08
#define CLI_ASCII_HT                0x09
#define CLI_ASCII_LF                0x0A
#define CLI_ASCII_CR                0x0D
#define CLI_ASCII_ESC               0x1B
#define CLI_ASCII_DEL               0x7F
#define CLI_ASCII_US                0x1F
#define CLI_ASCII_SP                0x20
#define CLI_VT100_ARROWUP           'A'
#define CLI_VT100_ARROWDOWN         'B'
#define CLI_VT100_ARROWRIGHT        'C'
#define CLI_VT100_ARROWLEFT         'D'


/* include all CLI files */
#include "cli/cJSON/cJSON.h"
#include "cli/cli_print.h"
#include "cli/command.h"
#include "cli/history.h"


void cli_init();
void cli_update();

#endif /* CLI_CLI_H_ */
