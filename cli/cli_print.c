/*
 * cli_print.c
 *
 *  Created on: 10.06.2018
 *      Author: marku
 */

#include "flora_lib.h"

#if CLI_ENABLE

extern bool cli_interactive_mode;
extern char cli_input_buffer[CLI_MAX_INPUT + 1];
extern uint16_t input_count;
extern uint16_t input_cursor;
extern bool cli_inside_execution;

static bool cli_is_newline = false;

void cli_clear_screen() {
  if (VT100_OUTPUT) {
    cli_print("\e[2J"); // Erases complete display. All lines are erased and changed to single-width. Cursor does not move.
    cli_print("\e[H"); // Moves cursor to home
  }
  cli_set_vt100_modes();
}

void cli_set_vt100_modes() {
  if (VT100_OUTPUT) {
    cli_print("\e[20h"); // Enable Linefeed/New Line Mode (LNM)
    cli_print("\e[7h"); // Enable Line-Wrap mode
    cli_print("\e[4h"); // Enable INSERT mode
    cli_print("\e[33h"); // Enable zWFM Wrap Forward Mode
    cli_print("\e[34h"); // Enable zWBM Wrap Backward Mode
    cli_set_color(CLI_RESET_ALL); // Reset text style (e.g. color)
  }
}

void cli_print(char* buffer) {
  uint16_t string_length = strlen(buffer);
  uart_tx(buffer, string_length);
  if ((buffer[0] >= 0x20 && buffer[0] < 127) || buffer[0] == 0x0a || buffer[0] == 0x0d) {
    cli_is_newline = (buffer[string_length-1] == '\n' );
  }
}

void cli_nprint(char* buffer, uint16_t size) {
  uint16_t string_length = strnlen(buffer, size);
  uart_tx(buffer, string_length);
  if ((buffer[0] >= 0x20 && buffer[0] < 127) || buffer[0] == 0x0a || buffer[0] == 0x0d) {
    cli_is_newline = (buffer[string_length-1] == '\n' );
  }
}

void cli_println(char* buffer) {
  cli_print(buffer);
  cli_print("\r\n");
  cli_is_newline = true;
}

void cli_log_inline(char* buffer, cli_log_level_t log_level, bool newline, bool last, bool separate) {
  if (cli_interactive_mode) {
    if (separate & !cli_is_newline) {
      cli_println("");
    }

    cli_set_color(log_level);
    if (newline) {
      cli_println(buffer);
    }
    else {
      cli_print(buffer);
    }

    cli_set_color(CLI_RESET_ALL);

    if (last && !cli_inside_execution) {
      cli_print_prompt(true);
    }
  }
  else {
    cli_log_inline_json(buffer, log_level);
  }
}

void cli_log_inline_json(char* buffer, cli_log_level_t log_level) {

  cJSON* content_item = cJSON_CreateObject();
  if (content_item == NULL) {
    goto end;
  }

  if (cJSON_AddStringToObject(content_item, "type", "log") == NULL) {
    goto end;
  }

  if (cJSON_AddStringToObject(content_item, "text", buffer) == NULL) {
    goto end;
  }

  char* log_level_name;

  switch (log_level) {
  case CLI_LOG_LEVEL_DEBUG:
    log_level_name = "debug";
    break;
  case CLI_LOG_LEVEL_DEFAULT:
    log_level_name = "default";
    break;
  case CLI_LOG_LEVEL_ERROR:
    log_level_name = "error";
    break;
  case CLI_LOG_LEVEL_INFO:
    log_level_name = "info";
    break;
  case CLI_LOG_LEVEL_WARNING:
    log_level_name = "warning";
    break;
  default:
    log_level_name = "";
    break;
  }

  if (cJSON_AddStringToObject(content_item, "level", log_level_name) == NULL) {
    goto end;
  }

  char* string = cJSON_PrintUnformatted(content_item);
  if (string == NULL) {
    cli_log_inline("Failed to print log item JSON", CLI_LOG_LEVEL_WARNING, true, false, true);
  }
  else {
    // if (VT100_OUTPUT) {
    //   cli_print(CLI_OUTPUT_BEGIN); // Print Record Separator (RS) to detect beginning and end of JSON
    // }
    cli_print(string);
    // if (VT100_OUTPUT) {
    //   cli_print(CLI_OUTPUT_END);
    // }
    cli_println("");

    cJSON_free(string);
  }

end:
  cJSON_Delete(content_item);
}

void cli_log(char* buffer, char* module, cli_log_level_t log_level) {
  if (cli_interactive_mode) {
    // if (VT100_OUTPUT) {
    //   cli_print(CLI_OUTPUT_BEGIN);
    // }
    char buf[128];

    cli_delete_prompt();

    if (rtc_format_time(buf, sizeof(buf))) {
      if (VT100_OUTPUT) {
        cli_set_color(CLI_FG_BRIGHT_CYAN);
      }
      cli_print("[");
      cli_print(buf);
      cli_print("]@");
    }

    snprintf(buf, sizeof(buf), "%lu", (long unsigned) hs_timer_get_current_timestamp());
    cli_print("(");
    cli_print(buf);
    cli_print(")\t");

    if (VT100_OUTPUT) {
      cli_set_color(CLI_FG_BRIGHT_YELLOW);
    }
    cli_print(module);
    cli_print("\t");

    if (VT100_OUTPUT) {
      cli_set_color(log_level);
    }
    cli_println(buffer);
    if (VT100_OUTPUT) {
      cli_set_color(CLI_LOG_LEVEL_DEFAULT);
      // cli_print(CLI_OUTPUT_END);
    }

    if(!cli_inside_execution) {
      cli_print_prompt(true);
    }
  }
  else if (strlen(buffer)) {
    cJSON* content = cJSON_CreateObject();
    if (content != NULL) {
      if (cJSON_AddStringToObject(content, "text", buffer) == NULL) {
        cJSON_Delete(content);
      }
      else {
        cli_log_json(content, module, log_level);
      }
    }
  }
}

void cli_log_json(cJSON* content, char* module, cli_log_level_t log_level) {
  char buf[64] = "";
  cJSON* log_item = content;

  if (rtc_format_time(buf, sizeof(buf))) {
    cJSON *rtc_time = cJSON_CreateString(buf);
    if (rtc_time == NULL) {
      goto end;
    }

    cJSON_AddItemToObject(log_item, "rtc_time", rtc_time);
  }

  if (cJSON_AddNumberToObject(log_item, "hs_time", (uint32_t) hs_timer_get_current_timestamp()) == NULL) {
    goto end;
  }

  if (cJSON_AddStringToObject(log_item, "module", module) == NULL) {
    goto end;
  }

  char* log_level_name;

  switch (log_level) {
  case CLI_LOG_LEVEL_DEBUG:
    log_level_name = "debug";
    break;
  case CLI_LOG_LEVEL_DEFAULT:
    log_level_name = "default";
    break;
  case CLI_LOG_LEVEL_ERROR:
    log_level_name = "error";
    break;
  case CLI_LOG_LEVEL_INFO:
    log_level_name = "info";
    break;
  case CLI_LOG_LEVEL_WARNING:
    log_level_name = "warning";
    break;
  default:
    log_level_name = "";
    break;
  }

  cJSON *level = cJSON_CreateString(log_level_name);
  cJSON_AddItemToObject(log_item, "level", level);

  char* string = cJSON_PrintUnformatted(log_item);
  if (string == NULL) {
    cli_log_inline("Failed to print log item JSON", CLI_LOG_LEVEL_WARNING, true, false, true);
  }

  // if (VT100_OUTPUT) {
  //   cli_print(CLI_OUTPUT_BEGIN); // Print Record Separator (RS) to detect beginning and end of JSON
  // }
  cli_print(string);
  // if (VT100_OUTPUT) {
  //   cli_print(CLI_OUTPUT_END);
  // }
  cli_println("");

  cJSON_free(string);

end:
  cJSON_Delete(log_item);
}


void cli_print_prompt(bool newline)
{
  cli_set_vt100_modes();
  if (!cli_is_newline && newline) {
    cli_println("");
  }

  if (cli_interactive_mode) {
    char buf[64];
    command_t* current_prompt = command_get_prompt();
    if (current_prompt != NULL) {
      snprintf(buf, sizeof(buf), "%s ", current_prompt->prompt);
      cli_print(buf);
    }

    cli_nprint(cli_input_buffer, input_count);
    input_cursor = input_count;
  }
}

bool cli_delete_prompt()
{
  if (!cli_inside_execution && cli_interactive_mode) {
    if (VT100_OUTPUT) {
      cli_print("\e[2K"); // Erase complete line
      cli_print("\e[0`"); // Move cursor to beginning of line
    }
    return true;
  }
  else {
    return false;
  }
}

void cli_set_color(uint8_t color) {
  if (VT100_OUTPUT) {
  char buf[8];
    snprintf(buf, sizeof(buf), "\e[%dm", color); // String interpolate color mode VT100 command
    cli_print(buf);
  }
}

bool cli_string_is_printable( const char *s, uint16_t size ) {
  int i;
  for (i = 0; i < (size - 1); i++) {
    if (*s >= 32 && *s < 127) {
      s++;
      continue;
    }
    else {
      return 0;
    }
  }

  return (*s == '\0');
}

#endif /* CLI_ENABLE */
