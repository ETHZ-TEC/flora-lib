/*
 * develop_cmd.c
 *
 *  Created on: Oct 9, 2018
 *      Author: rtrueb
 */

#include "flora_lib.h"

#if CLI_ENABLE

/* global variables */
extern bool cli_interactive_mode;

/* private variables */
develop_command_t current_command = CMD_NONE;
bool colibriwake_set = false;


static void develop_radio_rx_callback(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error);
static void develop_radio_tx_callback();
static void testlink_rx_callback(uint8_t* payload, uint8_t size);
static void testlink_tx_callback();
static void linktestmode_rx_callback(uint8_t* payload, uint8_t size);
static void linktestmode_tx_callback();
static void develop_colibriwake_callback(void);

static command_return_t develop_random_number_command_handler(command_execution_t execution);
static command_return_t develop_radioconfig_command_handler(command_execution_t execution);
static command_return_t develop_linktestmode_command_handler(command_execution_t execution);
static command_return_t develop_testlink_command_handler(command_execution_t execution);
static command_return_t develop_colibrienable_command_handler(command_execution_t execution);
static command_return_t develop_colibriwake_command_handler(command_execution_t execution);

/******************************************************************************
 * PARAMETERS
 ******************************************************************************/

/******************************************************************************
 * PARAMS: radioconfig
 ******************************************************************************/

static parameter_t develop_radioconfig_parameter_mod = {
    .name = "mod",
    .description = "Modulation (0: LoRa; 1: FSK)",

    .type = CMD_PARAMETER_BOOL,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = false,
    .has_value = true,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t develop_radioconfig_parameter_freq = {
    .name = "freq",
    .description = "Frequency.",

    .type = CMD_PARAMETER_FREQUENCY,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = false,
    .has_value = true,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t develop_radioconfig_parameter_datarate = {
    .name = "datarate",
    .description =  "\r\n\t\t\tLoRa: Spreading factor SF (5-12)"
          "\r\n\t\t\tFSK: The FSK modulations's bitrate in Hz (600 - 300000 bits/s). Frequency deviation gets calculated automatically.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = false,
    .has_value = true,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t develop_radioconfig_parameter_power = {
    .name = "txpower",
    .description = "Transmit power from -9 up to 22 dBm (default 0 dBm).",

    .type = CMD_PARAMETER_POWER,

    .options = NULL,
    .option_count = 0,

    .flag = 't',

    .optional = true,
    .has_value = true,
    .has_flag = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t develop_radioconfig_parameter_bandwidth = {
    .name = "bandwidth",
    .description =  "Bandwidth for modulation:\r\n\r\n"
            "\tLoRa:\r\n"

            "\t\t0: 125kHz (default)\r\n"
            "\t\t1: 250kHz\r\n"
            "\t\t2: 500kHz\r\n"


            "\tFSK:\r\n"

            "\t\t234.3kHz (default)\r\n"
            "\t\t312.0kHz\r\n"
            "\t\t373.6kHz\r\n"
            "\t\t467.0kHz\r\n"
            "\t\t500.0kHz",

    .type = CMD_PARAMETER_FREQUENCY,

    .options = NULL,
    .option_count = 0,

    .flag = 'w',

    .optional = true,
    .has_value = true,
    .has_flag = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t develop_radioconfig_parameter_implicit = {
    .name = "implicit",
    .description = "Sets the package engine into implicit header mode for LoRa and FSK, i.e. to a fixed payload length.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'i',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t develop_radioconfig_parameter_preamble = {
    .name = "preamble_len",
    .description = "Sets the preamble length (default 8 bytes/symbols).",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'p',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t develop_radioconfig_parameter_crc = {
    .name = "crc",
    .description = "Enables or disables CRC. By default ON.",

    .type = CMD_PARAMETER_BOOL,

    .options = NULL,
    .option_count = 0,

    .flag = 'c',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t develop_radioconfig_parameter_irq = {
    .name = "irq",
    .description = "Handles IRQ directly in DIO1 callback, rather than in system loop",

    .type = CMD_PARAMETER_POWER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = true,
    .has_value = false,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


static parameter_t* develop_radioconfig_parameters[] = {
    &develop_radioconfig_parameter_mod,
    &develop_radioconfig_parameter_freq,
    &develop_radioconfig_parameter_datarate,
    &develop_radioconfig_parameter_power,
    &develop_radioconfig_parameter_bandwidth,
    &develop_radioconfig_parameter_implicit,
    &develop_radioconfig_parameter_preamble,
    &develop_radioconfig_parameter_crc,
    &develop_radioconfig_parameter_irq,
};


/******************************************************************************
 * PARAMS: linktestmode
 ******************************************************************************/

static parameter_t develop_linktestmode_parameter_rx = {
    .name = "rx",
    .description = "Dummy paramter with no effect. ",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',
    .optional = true,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t* develop_linktestmode_parameters[] = {
    &develop_linktestmode_parameter_rx,
};

/******************************************************************************
 * PARAMS: testlink
 ******************************************************************************/

static parameter_t develop_testlink_parameter_rx = {
    .name = "rx",
    .description = "Dummy paramter with no effect. ",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',
    .optional = true,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};


static parameter_t* develop_testlink_parameters[] = {
  &develop_testlink_parameter_rx,
};

/******************************************************************************
 * PARAMS: colibrienable parameters
 ******************************************************************************/

static parameter_t develop_colibrienable_parameter_value = {
    .name = "value",
    .description = "Value of the Colibri_EN (COM_GPIO2) pin",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',
    .optional = false,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t* develop_colibrienable_parameters[] = {
    &develop_colibrienable_parameter_value,
};

/******************************************************************************
 * PARAMS: colibriwake parameters
 ******************************************************************************/

static parameter_t develop_colibriwake_parameter_delay = {
    .name = "delay",
    .description = "Time in seconds until the DPP_Colibri_WAKE (COM_GPIO1) pin is toggled",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',
    .optional = false,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t* develop_colibriwake_parameters[] = {
    &develop_colibriwake_parameter_delay,
};


/******************************************************************************
 * COMMANDS
 ******************************************************************************/

static command_t develop_random_number_command = {
  .execution_ptr = &develop_random_number_command_handler,
  .name = "random",
  .description = "Get random 32-bit integer from SX1262 RNG",
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

static command_t develop_radioconfig_command = {
  .execution_ptr = &develop_radioconfig_command_handler,
  .name = "config",
  .description = "Set Rx & Tx configuration on SX1262 according.",
  .prompt = "",
  .parameters = (parameter_t**) &develop_radioconfig_parameters,
  .parameter_count = PARAM_COUNT(develop_radioconfig_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t develop_linktestmode_command = {
  .execution_ptr = &develop_linktestmode_command_handler,
  .name = "linktestmode",
  .description = "Linktest mode: receive and bounce back message.",
  .prompt = "",
  .parameters = (parameter_t**) &develop_linktestmode_parameters,
  .parameter_count = PARAM_COUNT(develop_linktestmode_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t develop_testlink_command = {
  .execution_ptr = &develop_testlink_command_handler,
  .name = "testlink",
  .description = "Send a string via the SX1262's current configuration",
  .prompt = "",
  .parameters = (parameter_t**) develop_testlink_parameters,
  .parameter_count = PARAM_COUNT(develop_testlink_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t develop_colibrienable_command = {
  .execution_ptr = &develop_colibrienable_command_handler,
  .name = "colibrienable",
  .description = "Set the Colibri_EN (COM_GPIO2) pin",
  .prompt = "",
  .parameters = (parameter_t**) develop_colibrienable_parameters,
  .parameter_count = PARAM_COUNT(develop_colibrienable_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t develop_colibriwake_command = {
  .execution_ptr = &develop_colibriwake_command_handler,
  .name = "colibriwake",
  .description = "Wake up colibri after a specified time (i.e. toggle the DPP_Colibri_WAKE pin (COM_GPIO1))",
  .prompt = "",
  .parameters = (parameter_t**) develop_colibriwake_parameters,
  .parameter_count = PARAM_COUNT(develop_colibriwake_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

command_t* develop_subcommands[] = {
    &develop_random_number_command,
    &develop_radioconfig_command,
    &develop_linktestmode_command,
    &develop_testlink_command,
    &develop_colibrienable_command,
    &develop_colibriwake_command,
};

static command_t develop_command = {
  .execution_ptr = NULL,
  .name = "develop",
  .description = "Commands for development",
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

command_t* develop_commands[] = {&develop_command};

/******************************************************************************
 * REGISTER COMMANDS
 ******************************************************************************/

void develop_register_commands()
{
  command_register(&develop_command, develop_subcommands, COMMAND_COUNT(develop_subcommands));
  command_register(NULL, develop_commands, 1);

  return;
}

/******************************************************************************
 * COMMAND HANDLER
 ******************************************************************************/

command_return_t develop_random_number_command_handler(command_execution_t execution) {
  int32_t random = SX126xGetRandom();
  char buf[32];
  snprintf((char*) buf, 32, "%i", (int) random);
  cli_println(buf);
  return CMD_RET_SUCCESS;
}

command_return_t develop_radioconfig_command_handler(command_execution_t execution) {
  uint8_t count = command_get_unnamed_parameter_count(&execution);
  if (count >= 3) {
    uint8_t modulation = 0;
    uint32_t freq = 867300000; // TODO: use global defaults define in a config file
    int32_t datarate = -1;
    int8_t power = 0;
    int32_t bandwidth = -1;
    int preamble_length = 8;
    bool implicit = false;
    uint8_t payload_length = 0;
    bool crc = true;

    uint8_t buf[256];

    // 3 mandatory arguments: mod, freq, datarate
    if (count >= 3) {
      modulation = (uint8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10);
      freq = (uint32_t) strtol(command_get_unnamed_parameter_at_index(&execution, 1)->value, NULL, 10);
      datarate = (int32_t) strtol(command_get_unnamed_parameter_at_index(&execution, 2)->value, NULL, 10);

      if (cli_interactive_mode)
      {
        snprintf((char*) buf, sizeof(buf), "mod: %i\r\nfreq: %lu Hz\r\ndatarate: %ld", modulation, freq, datarate);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    value_t* power_param = command_get_parameter(&execution, 't');
    if (power_param != NULL) {
      power = (int8_t) strtol(power_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "power: %i", power);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    // set defaults which depend on modulation
    if (bandwidth == -1) {
      if (modulation == 0) {
        bandwidth = 0; // corresponds to 125000 Hz
      }
      else {
        bandwidth = 234300;
      }
    }

    // optional arguments with tags

    value_t* bandwidth_param = command_get_parameter(&execution, 'w');
    if (bandwidth_param != NULL) {
      bandwidth = (int32_t) strtol(bandwidth_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "bandwidth: %ld", bandwidth);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    value_t* implicit_param = command_get_parameter(&execution, 'i');
    if (implicit_param != NULL) {
      implicit = true;
      payload_length = strtol(implicit_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "payload: %i", payload_length);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    value_t* preamble_param = command_get_parameter(&execution, 'p');
    if (preamble_param != NULL) {
      preamble_length = strtol(preamble_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "preamble: %i", preamble_length);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    value_t* crc_param = command_get_parameter(&execution, 'c');
    if (crc_param != NULL) {
      crc = command_cast_parameter_to_bool(crc_param);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "crc: %d", crc);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    if (command_get_parameter_from_name(&execution, "irq", 3)) {
      radio_set_irq_direct(true);
    }
    else {
      radio_set_irq_direct(false);
    }

    // TODO: use config for coding rate (instead of hard-coded value)
    radio_set_config_rxtx(modulation, freq, datarate, power, bandwidth, preamble_length, 1, 0, implicit, payload_length, crc);


    if (!cli_interactive_mode) {
        bool failure = true;

        cJSON* config = cJSON_CreateObject();
        if (config == NULL) {
          goto end;
        }

        if (cJSON_AddStringToObject(config, "type", "radio_cfg") == NULL) {
          goto end;
        }

        if (cJSON_AddNumberToObject(config, "modulation", modulation) == NULL) {
          goto end;
        }

        if (cJSON_AddNumberToObject(config, "freq", freq) == NULL) {
          goto end;
        }

        if (count >= 3)
        {
          if (cJSON_AddNumberToObject(config, "power", power) == NULL) {
            goto end;
          }
        }

        if (preamble_length != -1) {
          if (cJSON_AddNumberToObject(config, "preamble", preamble_length) == NULL) {
            goto end;
          }
        }

        failure = false;
        cli_log_json(config, "radio", CLI_LOG_LEVEL_DEBUG);

    end:
        if (failure) {
          cJSON_Delete(config);
        }
    }

    return CMD_RET_SUCCESS;
  }
  else {
    command_print_description(&develop_radioconfig_command);
    return CMD_RET_FAILURE;
  }
}

command_return_t develop_linktestmode_command_handler(command_execution_t execution) {
  current_command = CMD_LINKTESTMODE;
  uint32_t timeout = 0;

  // prepare for rx
  radio_set_irq_mode(IRQ_MODE_RX);
  radio_set_rx_callback(&develop_radio_rx_callback);

  // start rx
  radio_receive(timeout);
  if (cli_interactive_mode) {
    cli_log_inline("Listening for linktest messages.", CLI_LOG_LEVEL_DEBUG, true, true, true);
  }
  else {
    cli_log("continuous rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
  }

  return CMD_RET_PENDING;
}

command_return_t develop_testlink_command_handler(command_execution_t execution) {
  current_command = CMD_TESTLINK;

  // prepare for tx
  radio_set_tx_callback(&develop_radio_tx_callback);

  uint8_t count = command_get_unnamed_parameter_count(&execution);

  if (count) {
    char* message = command_get_unnamed_parameter_at_index(&execution, 0)->value;

    uint16_t length = strlen(message) + 1;
    length = (length > 255) ? 255 : length;

    if (cli_interactive_mode) {
      char buf[256];
      snprintf(buf, 256, "payload_size:\t%i", length);
      cli_log_inline(buf, (length > 255) ? CLI_LOG_LEVEL_WARNING : CLI_LOG_LEVEL_DEFAULT, true, false, true);
    }
    else {
      bool failure = true;

      cJSON* tx = cJSON_CreateObject();
      if (tx == NULL) {
        goto end;
      }

      if (cJSON_AddStringToObject(tx, "type", "radio_tx") == NULL) {
        goto end;
      }

      if (cJSON_AddNumberToObject(tx, "payload", length) == NULL) {
        goto end;
      }

      failure = false;
      cli_log_json(tx, "radio", CLI_LOG_LEVEL_DEBUG);

end:
      if (failure) {
        cJSON_Delete(tx);
      }
    }
    radio_transmit((uint8_t*) message, length);
    return CMD_RET_SUCCESS;
  }
  else {
    radio_transmit(NULL, 0);    //FIXME
    return CMD_RET_SUCCESS;
  }
}

command_return_t develop_colibrienable_command_handler(command_execution_t execution) {
  uint8_t count = command_get_unnamed_parameter_count(&execution);
  char buf[32];

  if (count >= 1) {
    uint8_t pin_value = (uint8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10);

    if (cli_interactive_mode)
    {
      snprintf((char*) buf, sizeof(buf), "Colibri_EN: %d", pin_value);
      cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
    }

    if (pin_value) {
      PIN_SET(COM_GPIO2);
    } else {
      PIN_CLR(COM_GPIO2);
    }

    return CMD_RET_SUCCESS;
  }
  else {
    command_print_description(&develop_radioconfig_command);
    return CMD_RET_FAILURE;
  }
}

command_return_t develop_colibriwake_command_handler(command_execution_t execution) {
  uint8_t count = command_get_unnamed_parameter_count(&execution);
  char buf[50];

  if (count >= 1) {
    uint8_t wait_time = (uint8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10);

    if (cli_interactive_mode)
    {
      snprintf((char*) buf, sizeof(buf), "Wait for %d s and then toggle DPP_Colibri_WAKE", wait_time);
      cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
    }

    colibriwake_set = false;
    lptimer_set(lptimer_now() + LPTIMER_SECOND*wait_time, develop_colibriwake_callback);

    return CMD_RET_SUCCESS;
  }
  else {
    command_print_description(&develop_radioconfig_command);
    return CMD_RET_FAILURE;
  }
}

static void develop_colibriwake_callback(void) {
  if (colibriwake_set) {
    PIN_CLR(COM_GPIO1);
    colibriwake_set = false;
  } else {
    PIN_SET(COM_GPIO1);
    colibriwake_set = true;
    lptimer_set(lptimer_now() + LPTIMER_SECOND/10, develop_colibriwake_callback);
  }
}



/******************************************************************************
 * CALLBACK FUNCTIONS
 ******************************************************************************/


static void develop_radio_rx_callback(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error) {
  if (current_command == CMD_TESTLINK) {
    testlink_rx_callback(payload, size);
  } else if (current_command == CMD_LINKTESTMODE) {
    linktestmode_rx_callback(payload, size);
  }
}

static void develop_radio_tx_callback() {
  if (current_command == CMD_TESTLINK) {
    testlink_tx_callback();
  } else if (current_command == CMD_LINKTESTMODE) {
    linktestmode_tx_callback();
  }
}

static void linktestmode_rx_callback(uint8_t* payload, uint8_t size) {
  // TODO: only bounce back messages with predefined content (define content with config macro)

  // artificial delay
  rtc_delay(200);

  // prepare for tx
  radio_set_tx_callback(&develop_radio_tx_callback);

  // start tx
  radio_transmit((uint8_t*) payload, size, false);
}

static void testlink_rx_callback(uint8_t* payload, uint8_t size) {
  current_command = CMD_NONE;
}


static void linktestmode_tx_callback() {
  uint32_t timeout = 0;
  // TODO: check for correct state

  // prepare for rx
  radio_set_irq_mode(IRQ_MODE_RX);
  radio_set_rx_callback(&develop_radio_rx_callback);

  // start rx
  radio_receive(timeout);
  if (cli_interactive_mode) {
    cli_log_inline("Listening for linktest messages AGAIN.", CLI_LOG_LEVEL_DEBUG, true, true, true);
  }
  else {
    cli_log("continuous rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
  }
}

static void testlink_tx_callback() {
  uint32_t timeout = 0;

  // prepare for rx
  radio_set_irq_mode(IRQ_MODE_RX);
  radio_set_rx_callback(&develop_radio_rx_callback);

  // start rx
  radio_receive(timeout);
  if (cli_interactive_mode) {
    cli_log_inline("Listening for linktest reply...", CLI_LOG_LEVEL_DEBUG, true, true, true);
  }
  else {
    cli_log("continuous rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
  }
}

#endif /* CLI_ENABLE */
