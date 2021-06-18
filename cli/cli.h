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

#ifndef CLI_CLI_H_
#define CLI_CLI_H_


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
