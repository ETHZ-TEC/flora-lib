/*
 * cli.c
 *
 *  Created on: Apr 20, 2018
 *      Author: marku
 */

#include "flora_lib.h"

#if CLI_ENABLE

bool cli_interactive_mode = true; // Enables interactive command line mode (character get echoed back)

char cli_input_buffer[CLI_MAX_INPUT + 1] = {0};
uint16_t input_count = 0;
uint16_t input_cursor = 0;
bool cli_inside_execution = false;

#if defined(CLI_VERBOSE)
char* cli_splash_text =
        "\r\n\r\n"
        "    ________ ___       ________  ________  ________\r\n"
        "   |\\   ___\\\\  \\     |\\   __  \\|\\   __  \\|\\   __  \\\r\n"
        "   \\ \\  \\__/\\ \\  \\    \\ \\  \\|\\  \\ \\  \\|\\  \\ \\  \\|\\  \\\r\n"
        "    \\ \\   __\\\\ \\  \\    \\ \\  \\\\\\  \\ \\   _  _\\ \\   __  \\\r\n"
        "     \\ \\  \\_| \\ \\  \\____\\ \\  \\\\\\  \\ \\  \\\\  \\\\ \\  \\ \\  \\\r\n"
        "      \\ \\__\\   \\ \\_______\\ \\_______\\ \\__\\\\ _\\\\ \\__\\ \\__\\\r\n"
        "       \\|__|    \\|_______|\\|_______|\\|__|\\|__|\\|__|\\|__|"
        "\r\n\r\n"
        "               Flora Operating System CLI 0.1            \r\n\r\n"
        "   (Copyright 2018 by M. Wegmann. Licensed under MIT.)   \r\n\r\n";
#else
const char* cli_splash_text = "Flora Operating System CLI 0.1 (Copyright 2018 by M. Wegmann. Licensed under MIT.)\r\n";
#endif

volatile bool cli_initialized = false;
static bool cli_autocompleted = false;
static bool cli_escape = false;
static bool cli_vt100_escape = false;
static char cli_vt100_buffer[16] = "";

static char cli_char_buffer = '\0';

static void cli_process_input();
static bool cli_autocomplete(bool list);
static void cli_execute();

static void cli_insert_at_cursor(char chr);
static void cli_remove_at_cursor();


void cli_init() {
  command_init();
  history_init();

#ifdef HAL_RTC_MODULE_ENABLED
  rtc_register_commands();
#endif /* HAL_RTC_MODULE_ENABLED */
#if CLI_ENABLE
  led_register_commands();
  radio_register_commands();
  system_register_commands();
  test_register_commands();
  gloria_register_commands();
  cli_register_commands();
  develop_register_commands();
  slwb_register_commands();
#endif /* CLI_ENABLE */

  cli_set_vt100_modes();
  if (cli_interactive_mode) {
      cli_print((char*) cli_splash_text);
      cli_print_prompt(true);
  }

  cli_log("Initialized CLI", "cli", CLI_LOG_LEVEL_INFO);

	char buf[32];
  snprintf((char*) buf, 32, "HOST_ID: %i", (int) HOST_ID);    // FIXME
	cli_log(buf, "cli", CLI_LOG_LEVEL_INFO);
	snprintf((char*) buf, 32, "NODE_ID: %i", (int) NODE_ID);
	cli_log(buf, "cli", CLI_LOG_LEVEL_INFO);

  cli_initialized = true;
}

void cli_update() {
  if(cli_initialized) {
      cli_process_input();
  }
}

static void cli_process_input() {
  while (uart_read(&cli_char_buffer)) {
      if (cli_char_buffer != CLI_ASCII_HT) {
          cli_autocompleted = false;
      }

      if (cli_escape)
      {
          switch(cli_char_buffer)
          {
              case '[':
                  cli_vt100_escape = true;
                  cli_vt100_buffer[0] = '\0';
                  break;

              default:
                  break;
          }

          cli_escape = false;
      }
      else if (cli_vt100_escape)
      {
          switch(cli_char_buffer)
          {
              case '\e':
                  cli_vt100_escape = false;
                  cli_escape = true;
                  break;

              case '[':
                  break;

              case CLI_VT100_ARROWUP:
                  cli_delete_prompt();
                  input_count = history_get_previous(cli_input_buffer, input_count);
                  cli_print_prompt(false);
                  cli_vt100_escape = false;
                  break;

              case CLI_VT100_ARROWDOWN:
                  cli_delete_prompt();
                  input_count = history_get_next(cli_input_buffer, input_count);
                  cli_print_prompt(false);
                  cli_vt100_escape = false;
                  break;

              case CLI_VT100_ARROWLEFT:
                  if(input_cursor > 0) {
                      input_cursor--;
                      if (cli_interactive_mode) {
                          if (VT100_OUTPUT) {
                              cli_print("\e[D"); // Move cursor left
                          }
                      }
                  }
                  cli_vt100_escape = false;
                  break;
              case CLI_VT100_ARROWRIGHT:
                  if(input_cursor < input_count) {
                      input_cursor++;
                      if (cli_interactive_mode) {
                          if (VT100_OUTPUT) {
                              cli_print("\e[C"); // Move cursor right
                          }
                      }
                  }
                  cli_vt100_escape = false;
                  break;

              case 'H': // HOME key
                  if (!cli_inside_execution && input_cursor > 0) {
                      char buf[16];
                      snprintf(buf, sizeof(buf), "\e[%dD", input_cursor);
                      input_cursor = 0;
                      if (VT100_OUTPUT) {
                          cli_print(buf);
                      }
                      cli_vt100_escape = false;
                  }
              break;

              case 'F': // END key
                  if (!cli_inside_execution && (input_count - input_cursor) > 0) {
                      char buf[16];
                      snprintf(buf, sizeof(buf), "\e[%dC", input_count - input_cursor);
                      input_cursor = input_count;
                      if (VT100_OUTPUT) {
                          cli_print(buf);
                      }
                      cli_vt100_escape = false;
                  }
              break;

              default:
                  if (strlen(cli_vt100_buffer) < (sizeof(cli_vt100_buffer) - 2)) {
                      strncat(cli_vt100_buffer, (char*) &cli_char_buffer, 1);

                      // If DELETE key was pressed
                      if(strcmp(cli_vt100_buffer, "3~") == 0) {
                          cli_delete_prompt();
                          input_count = 0;
                          cli_print_prompt(false);
                          cli_vt100_escape = false;
                      }
                  }
                  else {
                      cli_vt100_escape = false;
                  }

                  break;
          }
      }
      else {
          switch(cli_char_buffer) {
              case'\e':
                  cli_escape = true;
                  break;
              case CLI_INTERACTIVE_MODE_TOGGLE:
                  cli_interactive_mode = !cli_interactive_mode;
                  if (cli_interactive_mode) {
                      if (VT100_OUTPUT) {
                          cli_print(CLI_INTERACTIVE_NACK);
                      }
                      cli_print_prompt(true);
                  }
                  else {
                      cli_println("");
                      if (VT100_OUTPUT) {
                          cli_print(CLI_INTERACTIVE_ACK);
                      }
                  }
                  break;
              case CLI_ASCII_BEL:
              case CLI_ASCII_DEL:
                  cli_print("\a");
                  break;
              case CLI_ASCII_HT:
                  if (cli_autocompleted) {
                      cli_autocomplete(true);
                  }
                  else {
                      cli_autocompleted = cli_autocomplete(false);
                  }
                  break;

              case '?':
                  cli_autocomplete(true);
                  break;

              case CLI_ASCII_BS:
                  if (input_count > 0 && input_cursor > 0) {
                      cli_remove_at_cursor();
                      input_count--;
                  }
                  else {
                      cli_print("\a");
                  }
                  break;

              case CLI_ASCII_CR:
                  cli_execute();
                  input_cursor = 0;
                  input_count = 0;
                  cli_print_prompt(true);
                  break;

              default:
                  // Add input character only if there is enough free buffer left and character is printable
                  if (input_count < CLI_MAX_INPUT && cli_char_buffer >= 0x20 && cli_char_buffer < 0x7F) {
                      cli_insert_at_cursor(cli_char_buffer);
                      input_count++;
                  }

                  break;
          }
      }
    }
}

static void cli_insert_at_cursor(char chr) {
  if (input_count < CLI_MAX_INPUT) {
      if (input_cursor < input_count) {
          memmove(cli_input_buffer + (input_cursor+1), cli_input_buffer + (input_cursor), input_count - input_cursor);
      }

      cli_input_buffer[input_cursor] = chr;

      if (cli_interactive_mode) {
          cli_nprint(&chr, 1);
      }

      input_cursor++;
  }
  else {
      cli_print("\a");
  }
}

static void cli_remove_at_cursor() {
  if (input_count > 0) {
      if (input_cursor < input_count) {
          memmove(cli_input_buffer + (input_cursor-1), cli_input_buffer + (input_cursor), input_count - input_cursor);
      }

      cli_input_buffer[input_count-1] = '\0';

      if (cli_interactive_mode) {
          cli_print("\b");
          if (VT100_OUTPUT) {
              cli_print("\e[1P"); // Delete one character at cursor. Move other characters to left.
          }
      }
      input_cursor--;
  }
}


static bool cli_autocomplete(bool list) {
  command_t* commands[16];
  char expansion[CLI_MAX_INPUT];
  expansion[0] = '\0';

  uint8_t matching_count = command_search(NULL, expansion, cli_input_buffer, input_count, commands);

  if(matching_count)
  {
      strcpy(cli_input_buffer, expansion);

      if(matching_count == 1 && commands[0]->children_count && list) {
          input_count = strlen(expansion);
          int i;
          for (i = 0; i < commands[0]->children_count; i++) {
              cli_log_inline(commands[0]->children[i]->name, CLI_LOG_LEVEL_INFO, true, false, true);
          }
          cli_print_prompt(true);
      }
      else if (matching_count > 1 && list) {
          input_count = strlen(expansion);
          int i;
          for (i = 0; i < matching_count; i++) {
              cli_log_inline(commands[i]->name, CLI_LOG_LEVEL_INFO, true, false, true);
          }
          cli_print_prompt(true);
      }
      else {
          cli_delete_prompt();
          input_count = strlen(expansion);
          cli_print_prompt(false);
      }

      return true;
  }
  else {
      if(commands[0]->children_count) {
          int i;
          for (i = 0; i < commands[0]->children_count; i++) {
              cli_log_inline(commands[0]->children[i]->name, CLI_LOG_LEVEL_INFO, true, false, true);
          }

          cli_print_prompt(true);
      }

      return false;
  }
}

static void cli_execute() {
  if (input_count > 0)
  {
      history_insert(cli_input_buffer, input_count);

      if (cli_interactive_mode) {
          cli_println("");
      }

      cli_input_buffer[input_count] = '\0';
      command_execution_t execution = command_get(NULL, cli_input_buffer, input_count);

      if (command_execution_check(execution)) {
          cli_inside_execution = true;
          // if (VT100_OUTPUT) {
          //     cli_print(CLI_OUTPUT_BEGIN); // produces unreadable symbols in CLI output
          // }
          execution.command->execution_ptr(execution);
          // if (VT100_OUTPUT) {
          //     cli_print(CLI_OUTPUT_END);
          // }
          cli_inside_execution = false;
      }
  }

  return;
}

#endif /* CLI_ENABLE */
