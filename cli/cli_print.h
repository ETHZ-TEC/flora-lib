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

#ifndef CLI_CLI_PRINT_H_
#define CLI_CLI_PRINT_H_


#define VT100_OUTPUT cli_interactive_mode

#define CLI_OUTPUT_BEGIN "\x02" // ASCII Start of Text (STX). Can be nested if there is an asynchronous log item or output to separate execution output.
#define CLI_OUTPUT_END "\x03" // ASCII End of Text (ETX)
#define CLI_JSON "\x1e" // ASCII Record Separator (RS)

#define CLI_INTERACTIVE_MODE_TOGGLE '\x14' // ASCII Device Control Four (DC4): Toggles the interactive mode. Useful for machine control.
#define CLI_INTERACTIVE_ACK "\x06" // ASCII Acknowledge (ACK): Signals change to machine mode.
#define CLI_INTERACTIVE_NACK "\x15" // ASCII Negative Acknowledge (NACK): Signals change to interactive mode.

#define CLI_FG_BLACK 30
#define CLI_FG_RED 31
#define CLI_FG_GREEN 32
#define CLI_FG_YELLOW 33
#define CLI_FG_BLUE 34
#define CLI_FG_MAGENTA 35
#define CLI_FG_CYAN 36
#define CLI_FG_WHITE 37
#define CLI_FG_BRIGHT_RED 91
#define CLI_FG_BRIGHT_GREEN 92
#define CLI_FG_BRIGHT_YELLOW 93
#define CLI_FG_BRIGHT_CYAN 96

#define CLI_BG_BLACK 40
#define CLI_BG_RED 41
#define CLI_BG_GREEN 42
#define CLI_BG_YELLOW 43
#define CLI_BG_BLUE 44
#define CLI_BG_MAGENTA 45
#define CLI_BG_CYAN 46
#define CLI_BG_WHITE 47

#define CLI_RESET_ALL 0
#define CLI_BOLD 1

typedef enum {
  CLI_LOG_LEVEL_DEFAULT = CLI_RESET_ALL,
  CLI_LOG_LEVEL_ERROR = CLI_FG_BRIGHT_RED,
  CLI_LOG_LEVEL_WARNING = CLI_FG_BRIGHT_YELLOW,
  CLI_LOG_LEVEL_INFO = CLI_FG_BRIGHT_CYAN,
  CLI_LOG_LEVEL_DEBUG = CLI_FG_BRIGHT_GREEN,
} cli_log_level_t;


#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define CLI_LOG(text, level) cli_log(text, __FILE__ ":" TOSTRING(__LINE__), level)

void cli_set_vt100_modes();

void cli_print(char* buffer);
void cli_nprint(char* buffer, uint16_t size);
void cli_println(char* buffer);

void cli_log(char* buffer, char* module, cli_log_level_t log_level);
void cli_log_json(cJSON* content, char* module, cli_log_level_t log_level);
void cli_log_inline(char* buffer, cli_log_level_t log_level, bool newline, bool last, bool separate);
void cli_log_inline_json(char* buffer, cli_log_level_t log_level);

void cli_print_prompt(bool newline);
bool cli_delete_prompt();

void cli_set_color(uint8_t color);
bool cli_string_is_printable(const char *s, uint16_t size);


/*
#define cli_print(str)
#define cli_nprint(str, n)
#define cli_println(str)
#define cli_log(str, mod, lvl)
#define cli_log_json(cont, mod, lvl)
#define cli_log_inline(str, lvl, nl, l, s)
#define cli_log_inline_json(str, lvl)
#define cli_print_prompt(nl)
#define cli_delete_prompt()
#define cli_set_color(c)
#define cli_string_is_printable(str, n)   0
*/

#endif /* CLI_CLI_PRINT_H_ */
