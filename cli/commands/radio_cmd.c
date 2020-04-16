/*
 * radio_cmd.c
 *
 *  Created on: May 7, 2018
 *      Author: marku
 */

#include "cli/cli.h"

#if CLI_ENABLE

/*!
 * \brief Sets the whitening mode.
 *
 * \param [IN] whitening  Whitening mode [0: no whitening, 1: whitening enabled]
 */
void RadioSetGfskWhitening( uint8_t whitening );

extern bool cli_interactive_mode;

static command_return_t radio_random_number_command_handler(command_execution_t execution);
static command_return_t radio_send_command_handler(command_execution_t execution);
static command_return_t radio_payload_command_handler(command_execution_t execution);
static command_return_t radio_receive_command_handler(command_execution_t execution);
static command_return_t radio_cad_command_handler(command_execution_t execution);
static command_return_t radio_free_command_handler(command_execution_t execution);
static command_return_t radio_whitening_command_handler(command_execution_t execution);
static command_return_t radio_preamble_command_handler(command_execution_t execution);
static command_return_t radio_execute_command_handler(command_execution_t execution);
static command_return_t radio_cw_command_handler(command_execution_t execution);
static command_return_t radio_config_command_handler(command_execution_t execution);
static command_return_t radio_syncword_command_handler(command_execution_t execution);
static command_return_t radio_standby_command_handler(command_execution_t execution);
static command_return_t radio_sleep_command_handler(command_execution_t execution);
static command_return_t radio_wakeup_command_handler(command_execution_t execution);
static command_return_t radio_reset_command_handler(command_execution_t execution);
static command_return_t radio_status_command_handler(command_execution_t execution);
static command_return_t radio_register_command_handler(command_execution_t execution);

static command_t radio_random_number_command = {
  .execution_ptr = &radio_random_number_command_handler,
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

static parameter_t radio_send_parameter_last = {
    .name = "last",
    .description = "Send payload from buffer again. Reuse payload length configured in radio",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'l',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_send_parameter_schedule = {
    .name = "schedule",
    .description = "Wait with execution of Tx command for precise timings.",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 's',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t* radio_send_parameters[] = {
    &radio_send_parameter_last,
    &radio_send_parameter_schedule,
};

static command_t radio_send_command = {
  .execution_ptr = &radio_send_command_handler,
  .name = "send",
  .description = "Send a string via the SX1262's current configuration",
  .prompt = "",
  .parameters = (parameter_t**) radio_send_parameters,
  .parameter_count = sizeof(radio_send_parameters) / sizeof(parameter_t*),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static parameter_t radio_payload_parameter_data = {
    .name = "data",
    .description = "Set the payload to none or given string",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'd',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_payload_parameter_simultaneous = {
    .name = "simultaneous",
    .description = "Setting payload simultaneous to current operation",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'i',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t* radio_payload_parameters[] = {
    &radio_payload_parameter_data,
    &radio_payload_parameter_simultaneous,
};

static command_t radio_payload_command = {
  .execution_ptr = &radio_payload_command_handler,
  .name = "payload",
  .description = "Sets or gets the payload from the radio's buffer. Even when in TX or RX!",
  .prompt = "",
  .parameters = (parameter_t**) radio_payload_parameters,
  .parameter_count = PARAM_COUNT(radio_payload_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};


static parameter_t radio_receive_parameter_rx = {
    .name = "rx",
    .description = "Puts radio into sniff mode. Represents Rx period in milli seconds [ms].",

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


static parameter_t radio_receive_parameter_sleep = {
    .name = "sleep",
    .description = "Puts radio into sniff mode. Represents sleep period in milli seconds [ms].",

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

static parameter_t radio_receive_parameter_schedule = {
    .name = "schedule",
    .description = "Wait with execution of Rx command for precise timings.",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 's',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_preamble = {
    .name = "preamble",
    .description = "Enables preamble detect interrupt",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'p',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_rx_irq = {
    .name = "rx_irq",
    .description = "Enables RX_DONE interrupt only",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'r',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_sync_irq = {
    .name = "sync_irq",
    .description = "Enables SYNC/HEADER_VALID interrupt only",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'v',

    .optional = true,
    .has_flag = true,
    .has_value = false,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_boost = {
    .name = "boost",
    .description = "Enables RX_DONE interrupt only",

    .type = CMD_PARAMETER_BOOL,

    .options = NULL,
    .option_count = 0,

    .flag = 'b',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_timeout = {
    .name = "timeout",
    .description = "Manually set MCU timeout",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 't',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_execute = {
    .name = "execute",
    .description = "Directly execute Rx with given execution schedule offset. Allows to enable internal RxTimeout.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'e',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_receive_parameter_compare = {
    .name = "compare",
    .description = "If we directly execute (-e), use timebase from compare register.",

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



static parameter_t* radio_receive_parameters[] = {
    &radio_receive_parameter_rx,
    &radio_receive_parameter_sleep,
    &radio_receive_parameter_schedule,
    &radio_receive_parameter_preamble,
    &radio_receive_parameter_rx_irq,
    &radio_receive_parameter_sync_irq,
    &radio_receive_parameter_boost,
    &radio_receive_parameter_timeout,
    &radio_receive_parameter_execute,
    &radio_receive_parameter_compare,
};

static command_t radio_receive_command = {
  .execution_ptr = &radio_receive_command_handler,
  .name = "receive",
  .description = "Receive a packet via the SX1262's current configuration",
  .prompt = "",
  .parameters = (parameter_t**) &radio_receive_parameters,
  .parameter_count = PARAM_COUNT(radio_receive_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t radio_cad_command = {
  .execution_ptr = &radio_cad_command_handler,
  .name = "cad",
  .description = "Runs a CAD",
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

static command_t radio_free_command = {
  .execution_ptr = &radio_free_command_handler,
  .name = "free",
  .description = "Checks if channel is free",
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

static parameter_t radio_cw_parameter_frequency = {
    .name = "freq",
    .description = "Sets the center carrier frequency of the continuous-wave emission.",

    .type = CMD_PARAMETER_FREQUENCY,

    .options = NULL,
    .option_count = 0,

    .flag = 'f',

    .optional = false,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t radio_cw_parameter_power = {
    .name = "power",
    .description = "Sets the power in dBm of the continuous-wave emission.",

    .type = CMD_PARAMETER_POWER,

    .options = NULL,
    .option_count = 0,

    .flag = 'p',

    .optional = false,
    .has_flag = false,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t radio_cw_parameter_duration = {
    .name = "duration",
    .description = "Sets the duration of the continuous-wave emission in us (int32_t). (Default = inf)",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'd',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_cw_parameter_repetitions = {
    .name = "repetitions",
    .description = "Sets the number of repetitions for the continuous-wave emissions. (Default: 1)",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'n',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_cw_parameter_interval = {
    .name = "interval",
    .description = "Sets the interval between multiple continuous-wave pulses in us (int32_t). (Default = 1000000)",

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

static parameter_t radio_cw_parameter_random = {
    .name = "random",
    .description = "Sets the maximum deviation due to the randomization of the interval between multiple continuous-wave pulses in us (int32_t). (Default = 0)",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'r',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t* radio_cw_parameters[] = {
    &radio_cw_parameter_frequency,
    &radio_cw_parameter_power,
    &radio_cw_parameter_duration,
    &radio_cw_parameter_repetitions,
    &radio_cw_parameter_interval,
    &radio_cw_parameter_random,
};

static command_t radio_cw_command = {
  .execution_ptr = &radio_cw_command_handler,
  .name = "cw",
  .description = "Set continuous-wave (CW) mode on SX126x. Be aware that without proper ",
  .prompt = "",
  .parameters = (parameter_t**) radio_cw_parameters,
  .parameter_count = PARAM_COUNT(radio_cw_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static parameter_t radio_execute_parameter_current = {
    .name = "current",
    .description = "Sets HS timer counter compare value to absolute HS timer counter value and schedules execution.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 't',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_execute_parameter_compare = {
    .name = "compare",
    .description = "Sets HS timer counter compare value to the current HS timer compare counter value plus an offset and schedules execution.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'd',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_execute_parameter_capture = {
    .name = "capture",
    .description = "Sets HS timer counter compare value to the current HS timer capture counter value plus an offset and schedules execution.",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'c',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t* radio_execute_parameters[] = {
    &radio_execute_parameter_current,
    &radio_execute_parameter_compare,
    &radio_execute_parameter_capture,
};

static command_t radio_execute_command = {
  .execution_ptr = &radio_execute_command_handler,
  .name = "execute",
  .description = "Execute scheduled command manually",
  .prompt = "",
  .parameters = (parameter_t**) radio_execute_parameters,
  .parameter_count = PARAM_COUNT(radio_execute_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t radio_preamble_command = {
  .execution_ptr = &radio_preamble_command_handler,
  .name = "preamble",
  .description = "Set continuous preamble on SX1262",
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

static parameter_t radio_config_parameter_mod = {
    .name = "mod",
    .description = "Modulation index from table",

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

static parameter_t radio_config_parameter_band = {
    .name = "band",
    .description = "Band index from table.",

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

static parameter_t radio_config_parameter_power = {
    .name = "power",
    .description = "Transmit power from -9 up to 22 dBm. If empty, configure radio in Rx mode, else in Tx mode.",

    .type = CMD_PARAMETER_POWER,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = true,
    .has_value = true,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t radio_config_parameter_bandwidth = {
    .name = "bandwidth",
    .description =  "Bandwidth for modulation:\r\n\r\n"
            "\tLoRa:\r\n"

            "\t\t0: 125kHz\r\n"
            "\t\t1: 250kHz\r\n"
            "\t\t2: 500kHz\r\n"


            "\tFSK:\r\n"

            "\t\t234.3kHz\r\n"
            "\t\t312.0kHz\r\n"
            "\t\t373.6kHz\r\n"
            "\t\t467.0kHz\r\n"
            "\t\t500.0kHz\r\n",

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

static parameter_t radio_config_parameter_fsk_bitrate = {
    .name = "fsk_bitrate",
    .description =  "The FSK modulations's bitrate in Hz. Frequency deviation gets calculated automatically.",

    .type = CMD_PARAMETER_FREQUENCY,

    .options = NULL,
    .option_count = 0,

    .flag = 'f',

    .optional = true,
    .has_value = true,
    .has_flag = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t radio_config_parameter_implicit = {
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

static parameter_t radio_config_parameter_preamble = {
    .name = "preamble_len",
    .description = "Sets the preamble length.",

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

static parameter_t radio_config_parameter_crc = {
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

static parameter_t radio_config_parameter_irq = {
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


static parameter_t* radio_config_parameters[] = {
    &radio_config_parameter_mod,
    &radio_config_parameter_band,
    &radio_config_parameter_power,
    &radio_config_parameter_bandwidth,
    &radio_config_parameter_fsk_bitrate,
    &radio_config_parameter_implicit,
    &radio_config_parameter_preamble,
    &radio_config_parameter_crc,
    &radio_config_parameter_irq,
};

static command_t radio_config_command = {
  .execution_ptr = &radio_config_command_handler,
  .name = "config",
  .description = "Set or get Rx & Tx configuration on SX1262 according to the band indexes defined in 'radio_constants.h'.",
  .prompt = "",
  .parameters = (parameter_t**) &radio_config_parameters,
  .parameter_count = PARAM_COUNT(radio_config_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t radio_syncword_command = {
  .execution_ptr = &radio_syncword_command_handler,
  .name = "syncword",
  .description = "Set or get the LoRa syncword",
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

static command_t radio_standby_command = {
  .execution_ptr = &radio_standby_command_handler,
  .name = "standby",
  .description = "Put radio into standby mode",
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

static parameter_t radio_sleep_parameter_cold = {
    .name = "cold_sleep",
    .description = "Set radio in cold sleep instead of warm (default)",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'c',

    .optional = true,
    .has_value = false,
    .has_flag = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


static parameter_t* radio_sleep_parameters[] = {
    &radio_sleep_parameter_cold,
};

static command_t radio_sleep_command = {
  .execution_ptr = &radio_sleep_command_handler,
  .name = "sleep",
  .description = "Put radio into sleep mode",
  .prompt = "",
  .parameters = (parameter_t**) &radio_sleep_parameters,
  .parameter_count = PARAM_COUNT(radio_sleep_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t radio_wakeup_command = {
  .execution_ptr = &radio_wakeup_command_handler,
  .name = "wakeup",
  .description = "Wakeup radio",
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

static parameter_t radio_whitening_parameter_value = {
  .name = "value",
  .description = "Sets GFSK whitening accordingly.",

  .type = CMD_PARAMETER_BOOL,

  .options = NULL,
  .option_count = 0,

  .flag = '\0',

  .optional = false,
  .has_flag = false,
  .has_value = true,
  .has_countable_options = false,
  .requires_flag_or_name = false,
};

static parameter_t* radio_whitening_parameters[] = {
    &radio_whitening_parameter_value,
};

static command_t radio_whitening_command = {
  .execution_ptr = &radio_whitening_command_handler,
  .name = "whitening",
  .description = "Enable / disable whitening for GFSK (default: enabled).",
  .prompt = "",
  .parameters = (parameter_t**) radio_whitening_parameters,
  .parameter_count = PARAM_COUNT(radio_whitening_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

static command_t radio_reset_command = {
  .execution_ptr = &radio_reset_command_handler,
  .name = "reset",
  .description = "Reset SX126x",
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


static command_t radio_status_command = {
  .execution_ptr = &radio_status_command_handler,
  .name = "status",
  .description = "Get status on SX126x",
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


static parameter_t radio_register_parameter_address = {
    .name = "addr",
    .description = "Reads or sets register at given address",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = true,
    .has_value = false,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};

static parameter_t radio_register_parameter_value = {
    .name = "value",
    .description = "Sets value for address.",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = '\0',

    .optional = true,
    .has_value = false,
    .has_flag = false,
    .has_countable_options = false,
    .requires_flag_or_name = false,
};


static parameter_t* radio_register_parameters[] = {
    &radio_register_parameter_address,
    &radio_register_parameter_value,
};

static command_t radio_register_command = {
  .execution_ptr = &radio_register_command_handler,
  .name = "register",
  .description = "Dump registers or get register content.",
  .prompt = "",
  .parameters = (parameter_t**) &radio_register_parameters,
  .parameter_count = PARAM_COUNT(radio_register_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};

command_t* radio_subcommands[] = {
    &radio_random_number_command,
    &radio_send_command,
    &radio_receive_command,
    &radio_payload_command,
    &radio_cad_command,
    &radio_free_command,
    &radio_whitening_command,
    &radio_preamble_command,
    &radio_execute_command,
    &radio_cw_command,
    &radio_config_command,
    &radio_syncword_command,
    &radio_standby_command,
    &radio_sleep_command,
    &radio_wakeup_command,
    &radio_reset_command,
    &radio_status_command,
    &radio_register_command,
};

static command_t radio_command = {
  .execution_ptr = NULL,
  .name = "radio",
  .description = "Commands for manipulating the radio chip directly",
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

command_t* radio_commands[] = {&radio_command};

void radio_register_commands()
{
  command_register(&radio_command, radio_subcommands, COMMAND_COUNT(radio_subcommands));
  command_register(NULL, radio_commands, 1);

  return;
}



command_return_t radio_random_number_command_handler(command_execution_t execution) {
  int32_t random = SX126xGetRandom();
  char buf[32];
  snprintf((char*) buf, 32, "%i", (int) random);
  cli_println(buf);
  return CMD_RET_SUCCESS;
}

command_return_t radio_send_command_handler(command_execution_t execution) {
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

    if (command_get_parameter(&execution, 's')) {
      radio_transmit((uint8_t*) message, length, true);
    }
    else {
//      write_test_registers();
      radio_transmit((uint8_t*) message, length, false);
    }

    return CMD_RET_SUCCESS;
  }
  else {
    if (command_get_parameter(&execution, 'l')) {
      if (command_get_parameter(&execution, 's')) {
        SX126xSetTxWithoutExecute(0);
        if (cli_interactive_mode) {
          cli_log_inline("Operation is scheduled.", CLI_LOG_LEVEL_DEBUG, true, true, true);
        }
      }
      else {
        SX126xSetTx(0);
      }
    }
    else {
      if (command_get_parameter(&execution, 's')) {
        radio_transmit(NULL, 0, true);
        if (cli_interactive_mode) {
          cli_log_inline("Operation is scheduled.", CLI_LOG_LEVEL_DEBUG, true, true, true);
        }
      }
      else {
        radio_transmit(NULL, 0, false);
      }
    }

    return CMD_RET_SUCCESS;
  }
}


command_return_t radio_receive_command_handler(command_execution_t execution) {
  //lora_set_standby();

  uint8_t count = command_get_unnamed_parameter_count(&execution);

  value_t* preamble_irq = command_get_parameter(&execution, 'p');
  value_t* rx_irq = command_get_parameter(&execution, 'r');
  value_t* sync_irq = command_get_parameter(&execution, 'v');

  if (preamble_irq != NULL) {
    radio_set_irq_mode(IRQ_MODE_RX_PREAMBLE);
    cli_log_inline("Enable PREAMBLE_DETECTED interrupt.", CLI_LOG_LEVEL_DEBUG, true, true, true);
  }
  else if (rx_irq != NULL) {
    radio_set_irq_mode(IRQ_MODE_RX_ONLY);
  }
  else if (sync_irq != NULL) {
    radio_set_irq_mode(IRQ_MODE_SYNC_RX_VALID);
  }
  else {
//    radio_set_irq_mode(IRQ_MODE_RX);
    radio_set_irq_mode(IRQ_MODE_RX_CRC);
  }

  bool boost = true;
  value_t* boost_param = command_get_parameter(&execution, 'b');
  if (boost_param) {
    boost = command_cast_parameter_to_bool(boost_param);
  }

  uint32_t timeout = 0;
  value_t* timeout_param = command_get_parameter(&execution, 't');
  if (timeout_param) {
    timeout = strtol(timeout_param->value, NULL, 10);
  }

  uint32_t execute_offset = 0;
  value_t* execute_param = command_get_parameter(&execution, 'e');
  if (execute_param) {
    execute_offset = strtol(execute_param->value, NULL, 10);
  }

  bool compare = false;
  value_t* compare_param = command_get_parameter(&execution, 'c');
  if (compare_param) {
    compare = command_cast_parameter_to_bool(compare_param);
  }

  if (!count) {
    if (execute_offset) {
      if (compare) {
        radio_receive_and_execute(boost, hs_timer_get_compare_timestamp() + execute_offset);
      }
      else {
        radio_receive_and_execute(boost, hs_timer_get_current_timestamp() + execute_offset);
      }

      if (cli_interactive_mode) {
        cli_log_inline("Listening for one message until radio is set into another mode (e.g. standby, sleep or Tx), RxTimeout or MCU timeout gets triggered after execution", CLI_LOG_LEVEL_DEBUG, true, true, true);
      }
      else {
        cli_log("single rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
      }
    }
    else if (command_get_parameter(&execution, 's')) {
      radio_receive(true, boost, timeout, 0);
      if (cli_interactive_mode) {
        cli_log_inline("Listening for one message until radio is set into another mode (e.g. standby, sleep or Tx) or MCU timeout gets triggered after execution", CLI_LOG_LEVEL_DEBUG, true, true, true);
        cli_log_inline("Operation is scheduled.", CLI_LOG_LEVEL_DEBUG, true, true, true);
      }
      else {
        cli_log("scheduled single rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
      }
    }
    else {
      radio_receive(false, boost, timeout, 0);

      if (cli_interactive_mode) {
        cli_log_inline("Listening for messages repeatedly until radio is set into another mode (e.g. standby, sleep or Tx)", CLI_LOG_LEVEL_DEBUG, true, true, true);
      }
      else {
        cli_log("continuous rx_mode", "radio", CLI_LOG_LEVEL_DEBUG);
      }
    }
  }

  else if (count == 2) {
    uint32_t rx_period = (uint32_t) strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10); // us
    uint32_t sleep_period = (uint32_t) strtol(command_get_unnamed_parameter_at_index(&execution, 1)->value, NULL, 10); // us

    if (cli_interactive_mode) {
      cli_log_inline("Listening for message in listen/sniff mode.", CLI_LOG_LEVEL_DEBUG, true, true, true);
    }
    else {
      cli_log("sniff_mode", "radio", CLI_LOG_LEVEL_DEBUG);
    }

    bool schedule = false;

    if (command_get_parameter(&execution, 's')) {
      if (cli_interactive_mode) {
        cli_log_inline("Operation is scheduled.", CLI_LOG_LEVEL_DEBUG, true, true, true);
      }
      schedule = true;
    }

    radio_receive_duty_cycle(rx_period, sleep_period, schedule);
  }

  return CMD_RET_PENDING;
}


command_return_t radio_payload_command_handler(command_execution_t execution) {

  value_t* data = command_get_parameter(&execution, 'd');

  if (data == NULL)
  {
    uint8_t payload[255];
    char buf[8];
    uint8_t offset;
    uint8_t size;

    radio_get_payload(payload, &offset, &size);

    char status_buf[64];
    snprintf(status_buf, sizeof(status_buf), "offset: %i\r\nsize: %i", offset, size);
    cli_log_inline(status_buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);


    int j;
    for (j = 0; j < size; j++) {
      snprintf(buf, sizeof(buf), "%02x", payload[j]);
      if(j % 8 == 7) {
        strcat(buf, "\r\n");
      }
      else {
        strcat(buf, " ");
      }

      cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, false, false, false);
    }

    cli_log_inline("", CLI_LOG_LEVEL_DEFAULT, true, true, true);

    return CMD_RET_SUCCESS;
  }
  else {
    if (command_get_parameter(&execution, 'i')) {
      radio_set_payload_while_transmit((uint8_t*) data->value, 0, (uint8_t) strlen(data->value) + 1);
      return CMD_RET_SUCCESS;
    }
    else {
      radio_set_payload((uint8_t*) data->value, 0, (uint8_t) strlen(data->value) + 1);
      return CMD_RET_SUCCESS;
    }
  }
}


command_return_t radio_cad_command_handler(command_execution_t execution) {
  if(execution.value_count >= 1) {
    radio_set_irq_mode(IRQ_MODE_CAD_RX);
    radio_set_cad_params(true, true);
    radio_set_cad();
  }
  else {
    radio_set_irq_mode(IRQ_MODE_CAD);
    radio_set_cad_params(false, false);
    radio_set_cad();
  }

  return CMD_RET_PENDING;
}


command_return_t radio_free_command_handler(command_execution_t execution) {
  bool free = Radio.IsChannelFree(MODEM_LORA, 869400000, -20, 100);

  /*
  int8_t rssi = SX126xGetRssiInst();
  char buf[64];
  snprintf(buf, sizeof(buf), "CAD RSSI: %d", rssi);
  DBG(buf, CLI_MSG_INFO);
  */

  if (free)
  {
    cli_log_inline("Channel is free!", CLI_LOG_LEVEL_DEFAULT, true, true, true);
  }
  else {
    cli_log_inline("Channel is taken!", CLI_LOG_LEVEL_DEFAULT, true, true, true);
  }

  return CMD_RET_SUCCESS;
}


command_return_t radio_whitening_command_handler(command_execution_t execution) {
  if (execution.value_count == 1)
  {
    bool whitening = command_cast_parameter_to_bool(command_get_unnamed_parameter_at_index(&execution, 0));
    RadioSetGfskWhitening(whitening);
  }
  else
  {
    return CMD_RET_FAILURE;
  }

  return CMD_RET_SUCCESS;
}


command_return_t radio_preamble_command_handler(command_execution_t execution) {
    radio_set_continuous_preamble();

    return CMD_RET_SUCCESS;
}

command_return_t radio_execute_command_handler(command_execution_t execution) {
  value_t* current_param = command_get_parameter(&execution, 't');
  value_t* compare_param = command_get_parameter(&execution, 'd');
  value_t* capture_param = command_get_parameter(&execution, 'c');

  if (current_param) {
    uint32_t timer = (uint32_t) strtol(current_param->value, NULL, 10);
    radio_execute_manually(timer);
  }
  else if (compare_param) {
    uint32_t timer = hs_timer_get_compare_timestamp() + (uint32_t) strtol(compare_param->value, NULL, 10);
    radio_execute_manually(timer);
  }
  else if (capture_param) {
    uint32_t offset = (uint32_t) strtol(capture_param->value, NULL, 10);
    uint32_t capture_time = hs_timer_get_capture_timestamp();
    uint32_t timer = capture_time + offset;
    radio_execute_manually(timer);
  }
  else {
    radio_execute_manually(-1);
  }

  return CMD_RET_SUCCESS;
}

command_return_t radio_cw_command_handler(command_execution_t execution) {
  if (execution.value_count >= 2) {
    uint32_t frequency = (uint32_t) strtol(execution.values[0].value, NULL, 10);
    int8_t power = (int8_t) strtol(execution.values[1].value, NULL, 10);

    int32_t duration = -1;
    value_t* duration_param = command_get_parameter(&execution, 'd');
    if (duration_param != NULL) {
      duration = (int32_t) strtol(duration_param->value, NULL, 10);
    }

    int32_t repetitions = 1;
    value_t* repetitions_param = command_get_parameter(&execution, 'n');
    if (repetitions_param != NULL) {
      repetitions = (int32_t) strtol(repetitions_param->value, NULL, 10);
    }

    int32_t interval = 1000000;
    value_t* interval_param = command_get_parameter(&execution, 'i');
    if (interval_param != NULL) {
      interval = (int32_t) strtol(interval_param->value, NULL, 10);
    }

    int32_t random = 0;
    value_t* random_param = command_get_parameter(&execution, 'r');
    if (random_param != NULL) {
      random = (int32_t) strtol(random_param->value, NULL, 10);
    }

    uint8_t buf[256];
    snprintf((char*) buf, 256, "Frequency: %i; Power: %i, Duration: %i, Repetitions: %i, Interval: %i, Random: %i",
      (int) frequency,
      (int) power,
      (int) duration,
      (int) repetitions,
      (int) interval,
      (int) random
    );
    cli_println((char*) buf);

    if (duration < 0) {
      Radio.SetTxContinuousWave(frequency, power, 0);
    }
    else {
      uint64_t endTs = 0;
      int64_t randDev = 0;

      // seed random generator
      if (random != 0) {
        srand(SX126xGetRandom());
      }

      uint64_t startTs = hs_timer_get_current_timestamp(); // reference time for all cw pulses generated by the cw command

      for (int i=0; i<repetitions; i++) {
          // start cw pulse
          Radio.SetTxContinuousWave(frequency, power, 0);

          if (duration != 0) {
            // delay for Tx (busy wait)
            endTs = startTs + (i*interval + duration + + randDev + 200)*HS_TIMER_FREQUENCY_US; // times 8 (HS_TIMER_FREQUENCY_US) convert us to clock ticks (8 MHz clock speed), randDev necessary since start AND stop needs to be shifted, plus const to compensate overhead
            while (hs_timer_get_current_timestamp() < endTs) {};
          }

          // stop cw pulse
          Radio.Standby();

          if (i+1 == repetitions) {
            break;
          }

          // determine random deviation of interval
          if (random != 0) {
            int64_t low_num = -random;
            int64_t hi_num = random;
            randDev = (rand() % (hi_num - low_num)) + low_num;
          }

          // delay for wait time between cw pulses
          endTs = startTs + ((i+1)*interval + randDev)*HS_TIMER_FREQUENCY_US; // times 8 (HS_TIMER_FREQUENCY_US) convert us to clock ticks (8 MHz clock speed)
          while (hs_timer_get_current_timestamp() < endTs) {};
      }
    }
    return CMD_RET_SUCCESS;
  } else {
    cli_log_inline("Usage: See 'help radio cw'\r\n", CLI_LOG_LEVEL_DEFAULT, true, true, true);
    return CMD_RET_FAILURE;
  }
}

command_return_t radio_config_command_handler(command_execution_t execution) {
  uint8_t count = command_get_unnamed_parameter_count(&execution);
  if (count >= 2) {

    uint8_t modulation = 0;
    uint8_t band = 48;
    int8_t power = 0;

    uint8_t buf[256];

    if (count >= 2) {
      modulation = (uint8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10);
      band = (uint8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 1)->value, NULL, 10);

      if (cli_interactive_mode)
      {
        snprintf((char*) buf, sizeof(buf), "mod: %i\r\nband: %i", modulation, band);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }

      if (count >= 3) {
        power = (int8_t) strtol(command_get_unnamed_parameter_at_index(&execution, 2)->value, NULL, 10);
        if (cli_interactive_mode) {
          snprintf((char*) buf, sizeof(buf), "power: %i", power);
          cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
        }
      }
    }


    int32_t bandwidth = -1;
    int32_t fsk_bitrate = -1;

    value_t* bandwidth_param = command_get_parameter(&execution, 'w');
    if (bandwidth_param != NULL) {
      bandwidth = (int32_t) strtol(bandwidth_param->value, NULL, 10);

      value_t* fsk_bitrate_param = command_get_parameter(&execution, 'f');
      if (fsk_bitrate_param != NULL) {
        fsk_bitrate = (int32_t) strtol(fsk_bitrate_param->value, NULL, 10);
      }
    }


    bool implicit = false;
    uint8_t payload_length = 0;

    value_t* implicit_param = command_get_parameter(&execution, 'i');
    if (implicit_param != NULL) {
      implicit = true;
      payload_length = strtol(implicit_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "payload: %i", payload_length);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    int preamble_length = -1;

    value_t* preamble_param = command_get_parameter(&execution, 'p');
    if (preamble_param != NULL) {
      preamble_length = strtol(preamble_param->value, NULL, 10);
      if (cli_interactive_mode) {
        snprintf((char*) buf, sizeof(buf), "preamble: %i", preamble_length);
        cli_log_inline((char*) buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
      }
    }

    bool crc = true;

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


    if (count == 2) {
      radio_set_config_rx(modulation, band, bandwidth, fsk_bitrate, preamble_length, 0, implicit, payload_length, crc, true);
    }
    else {
      radio_set_config_tx(modulation, band, power, bandwidth, fsk_bitrate, preamble_length, implicit, crc);
    }


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

        if (cJSON_AddNumberToObject(config, "band", band) == NULL) {
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
    command_print_description(&radio_config_command);
    return CMD_RET_FAILURE;
  }
}

command_return_t radio_syncword_command_handler(command_execution_t execution) {
  if (execution.value_count == 1) {
    if (strcasecmp(execution.values[0].value, "private") == 0) {
      radio_set_lora_syncword(LORA_SYNCWORD_PRIVATE);
      cli_println("Set private sync word.");
      return CMD_RET_SUCCESS;
    } else if (strcasecmp(execution.values[0].value, "public") == 0) {
      radio_set_lora_syncword(LORA_SYNCWORD_PUBLIC);
      cli_println("Set public sync word.");
      return CMD_RET_SUCCESS;
    } else if (strcasecmp(execution.values[0].value, "glossy") == 0) {
      radio_set_lora_syncword(LORA_SYNCWORD_PERMASENSE);
      cli_println("Set glossy sync word.");
      return CMD_RET_SUCCESS;
    }
    else
    {
      cli_println("Usage: radio syncword <mode>\r\n\r\n"
            "\t<mode>:\r\n"
            "\t\tpublic\r\n"
            "\t\tprivate\r\n"
            "\t\tglossy\r\n");
      return CMD_RET_FAILURE;
    }

  }

  uint16_t syncword = radio_get_syncword();

  char buf[32];
  snprintf(buf, 32, "LoRa syncword: %#06x", syncword);
  cli_println((char*) buf);

  return CMD_RET_SUCCESS;
}

command_return_t radio_standby_command_handler(command_execution_t execution) {
  Radio.Standby();
  return CMD_RET_SUCCESS;
}

command_return_t radio_sleep_command_handler(command_execution_t execution) {
  if (command_get_parameter(&execution, 'c')) {
    radio_sleep(false);
  }
  else {
    radio_sleep(true);
  }
  return CMD_RET_SUCCESS;
}

command_return_t radio_wakeup_command_handler(command_execution_t execution) {
  radio_wakeup();
  return CMD_RET_SUCCESS;
}

command_return_t radio_reset_command_handler(command_execution_t execution) {
  radio_init();
  return CMD_RET_SUCCESS;
}

command_return_t radio_status_command_handler(command_execution_t execution) {
  PacketStatus_t pkt_status;

  SX126xGetPacketStatus(&pkt_status);

  char buf[255] = "Operating_Mode: ";

  RadioOperatingModes_t operating_mode = SX126xGetOperatingMode();
  switch (operating_mode) {
    case MODE_SLEEP:
      strcat(buf,"sleep");
      break;
    case MODE_STDBY_RC:
      strcat(buf,"rc");
      break;
    case MODE_STDBY_XOSC:
      strcat(buf,"xosc");
      break;
    case MODE_FS:
      strcat(buf,"fs");
      break;
    case MODE_TX:
      strcat(buf,"tx");
      break;
    case MODE_RX:
      strcat(buf,"rx");
      break;
    case MODE_RX_DC:
      strcat(buf,"dc");
      break;
    case MODE_CAD:
      strcat(buf,"cad");
      break;
    default:
      strcat(buf,"none");
      break;
  }

  cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);


  cli_log_inline("Packet Status:", CLI_LOG_LEVEL_DEFAULT, true, false, true);

  switch (pkt_status.packetType) {
  case PACKET_TYPE_GFSK:
    snprintf(buf,sizeof(buf),
         "pkt_type: FSK\r\n"
         "freq_error: %i\r\n"
         "rssi_avg: %i\r\n"
         "rssi_sync: %i\r\n"
         "rx_status: %i",
         (int) pkt_status.Params.Gfsk.FreqError,
         pkt_status.Params.Gfsk.RssiAvg,
         pkt_status.Params.Gfsk.RssiSync,
         pkt_status.Params.Gfsk.RxStatus
    );
    break;

  case PACKET_TYPE_LORA:
    snprintf(buf,sizeof(buf),
         "pkt_type: LoRa\r\n"
         "freq_error: %i\r\n"
         "rssi_pkt: %i\r\n"
         "signal_rssi_pkt: %i\r\n"
         "snr_pkt: %i",
         (int) pkt_status.Params.LoRa.FreqError,
         pkt_status.Params.LoRa.RssiPkt,
         pkt_status.Params.LoRa.SignalRssiPkt,
         pkt_status.Params.LoRa.SnrPkt
    );
    break;

  case PACKET_TYPE_NONE:
    break;

  default:
    break;
  }

  cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);

  RadioStatus_t status = SX126xGetStatus();

  snprintf(buf,sizeof(buf),
     "Radio Status:\r\n"
     "cmd_status: %#04x\r\n"
     "chip_mode: %#04x\r\n"
     "cpu_busy: %#04x",
     status.Fields.CmdStatus,
     status.Fields.ChipMode,
     status.Fields.CpuBusy
  );
  cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);

  uint16_t irq_status = SX126xGetIrqStatus();
  snprintf(buf,sizeof(buf),
     "IRQ_Status: %#06x",
     irq_status
  );
  cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, true);

  RadioError_t errors = SX126xGetDeviceErrors();
  snprintf(buf,sizeof(buf),
     "Errors: %#06x",
     errors.Value
  );

  cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);

  return CMD_RET_SUCCESS;
}

command_return_t radio_register_command_handler(command_execution_t execution) {
  uint8_t count = command_get_unnamed_parameter_count(&execution);

  if (count > 0) {
    uint16_t address = strtol(command_get_unnamed_parameter_at_index(&execution, 0)->value, NULL, 10);

    if (count == 2) {
      uint8_t value = strtol(command_get_unnamed_parameter_at_index(&execution, 1)->value, NULL, 10);
      SX126xWriteRegister(address, value);
    }
    else {
      uint8_t value = SX126xReadRegister(address);
      char buf[24];
      snprintf(buf, sizeof(buf), "value: %#04x", value);
      cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
    }
  }
  else {
    int i;
    for (i = 0; i < UINT16_MAX; i += 16) {
      uint8_t value_buf[16];
      SX126xReadRegisters(i, value_buf, 16);

      char buf[128] = "";

      int j;
      for (j = 0; j < 16; j++) {
        char mini_buf[3];
        snprintf(mini_buf, sizeof(mini_buf), "%02x", value_buf[j]);
        strcat(buf, mini_buf);

        if (j != 15) {
          strcat(buf, " ");
        }
      }

      cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, true, true);
    }
  }

  return CMD_RET_SUCCESS;
}

#endif /* CLI_ENABLE */
