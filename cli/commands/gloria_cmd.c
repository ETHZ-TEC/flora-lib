/*
 * gloria_cmd.c
 *
 *  Created on: Aug 7, 2018
 *      Author: marku
 */

#include "flora_lib.h"

#if CLI_ENABLE

extern uint64_t gloria_last_sync;
extern bool cli_interactive_mode;
uint32_t tx_period;
uint32_t guard_time;
uint32_t iterations;

gloria_message_t message;
gloria_flood_t flood;

gloria_message_t print_message;
gloria_flood_t print_flood;

bool gloria_flood_to_be_printed = false;

static void sync_callback();
static void gloria_finish_callback();
static void gloria_cont_callback();

static command_return_t gloria_rx_command_handler(command_execution_t execution);
static command_return_t gloria_tx_command_handler(command_execution_t execution);

/*
 * General gloria parameters
 */
static parameter_t gloria_parameter_ack = {
    .name = "ack_mode",
    .description = "set ack mode",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'a',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_parameter_idx = {
    .name = "idx",
    .description = "set flood index",

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

static parameter_t gloria_parameter_macks = {
    .name = "max_acks",
    .description = "set max number of acks sent",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'k',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_parameter_slots = {
    .name = "slots",
    .description = "set number of data slots",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'l',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


static parameter_t gloria_parameter_mod = {
    .name = "mod",
    .description = "Sets the modulation",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'm',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_parameter_pow = {
    .name = "power",
    .description = "set power",

    .type = CMD_PARAMETER_POWER,

    .options = NULL,
    .option_count = 0,

    .flag = 'o',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_parameter_retr = {
    .name = "retransmissions",
    .description = "set max retransmissions",

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


static parameter_t gloria_parameter_marker = {
    .name = "marker_delta",
    .description = "Set the flood marker relative to the last sync time. If not given will be immediate.",

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


static parameter_t gloria_parameter_continuous = {
    .name = "cont_meas",
    .description = "Continue with rx or tx after flood is finished",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'u',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


static parameter_t gloria_parameter_tx_period = {
    .name = "tx_period",
    .description = "Period of flood transmissions",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'x',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


/*
 * Specific parameters for gloria rx
 */
static parameter_t gloria_rx_parameter_cad = {
    .name = "cad",
    .description = "Sets rx timeout in ms",

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

static parameter_t gloria_rx_parameter_guard = {
    .name = "guard_time",
    .description = "set the guard time",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 'g',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_rx_parameter_sync = {
    .name = "sync_timer",
    .description = "sync the timer to the initiator",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 's',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


/*
 * Specific parameters for gloria tx
 */
static parameter_t gloria_tx_parameter_iterations = {
    .name = "iters",
    .description = "Set the number of floods for continuous mode.",

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

static parameter_t gloria_tx_parameter_destination = {
    .name = "dst",
    .description = "Set the flood destination. Will be broadcast if no destination given.",

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

static parameter_t gloria_tx_parameter_payload = {
    .name = "payload",
    .description = "Payload to transmit. If not given will be empty payload.",

    .type = CMD_PARAMETER_NONE,

    .options = NULL,
    .option_count = 0,

    .flag = 'p',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};

static parameter_t gloria_tx_parameter_sync = {
    .name = "sync_flood",
    .description = "send sync timestamp with flood",

    .type = CMD_PARAMETER_INTEGER,

    .options = NULL,
    .option_count = 0,

    .flag = 's',

    .optional = true,
    .has_flag = true,
    .has_value = true,
    .has_countable_options = false,
    .requires_flag_or_name = true,
};


/*
 * Parameter assignments
 */
static parameter_t* gloria_rx_parameters[] = {
  &gloria_parameter_ack,
  &gloria_parameter_idx,
  &gloria_parameter_slots,
  &gloria_parameter_macks,
  &gloria_parameter_mod,
  &gloria_parameter_retr,
  &gloria_parameter_pow,
  &gloria_parameter_marker,
  &gloria_parameter_continuous,
  &gloria_parameter_tx_period,
  &gloria_rx_parameter_cad,
  &gloria_rx_parameter_guard,
  &gloria_rx_parameter_sync,
};

static command_t gloria_rx_command = {
  .execution_ptr = &gloria_rx_command_handler,
  .name = "rx",
  .description = "Puts node into gloria rx mode.",
  .prompt = "",
  .parameters = (parameter_t**) gloria_rx_parameters,
  .parameter_count = PARAM_COUNT(gloria_rx_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};


static parameter_t* gloria_tx_parameters[] = {
  &gloria_parameter_ack,
  &gloria_parameter_idx,
  &gloria_parameter_macks,
  &gloria_parameter_slots,
  &gloria_parameter_mod,
  &gloria_parameter_retr,
  &gloria_parameter_pow,
  &gloria_parameter_marker,
  &gloria_parameter_continuous,
  &gloria_parameter_tx_period,
  &gloria_tx_parameter_iterations,
  &gloria_tx_parameter_destination,
  &gloria_tx_parameter_payload,
  &gloria_tx_parameter_sync,
};

static command_t gloria_tx_command = {
  .execution_ptr = &gloria_tx_command_handler,
  .name = "tx",
  .description = "Initiates a flood.",
  .prompt = "",
  .parameters = (parameter_t**) gloria_tx_parameters,
  .parameter_count = PARAM_COUNT(gloria_tx_parameters),
  .children = NULL,
  .children_count = 0,
  .executable = true,
  .built_in = false,
  .has_prompt = false,
  .hidden = false,
  .children_only_in_prompt_invokable = false
};


static command_t* gloria_subcommands[] = {&gloria_rx_command, &gloria_tx_command};

static command_t gloria_command = {
  .execution_ptr = NULL,
  .name = "gloria",
  .description = "",
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

static command_t* gloria_commands[] = {&gloria_command};

void gloria_register_commands() {
  command_register(&gloria_command, gloria_subcommands, COMMAND_COUNT(gloria_subcommands));
  command_register(NULL, gloria_commands, 1);
}


/*
 * Gloria command functions
 */
static command_return_t gloria_rx_command_handler(command_execution_t execution) {
  bool lp_listening = false;
  uint64_t marker = 0;
    value_t* marker_delta_param = command_get_parameter(&execution, 't');
  if (marker_delta_param != NULL) {
    marker = gloria_last_sync + strtol(marker_delta_param->value, NULL, 10);
    lp_listening = true;
    if (marker - hs_timer_get_current_timestamp() > INT64_MAX)
    {
      if (cli_interactive_mode) {
      }
      cli_log_inline("Delta marker is in the past!", CLI_LOG_LEVEL_ERROR, true, true, true);

      return CMD_RET_FAILURE;
    }
  }

  uint8_t ack_mode = 0;
  value_t* ack_param = command_get_parameter(&execution, 'a');
  if (ack_param != NULL) {
    ack_mode = strtol(ack_param->value, NULL, 10);
  }

  uint16_t idx = 0;
  value_t* idx_param = command_get_parameter(&execution, 'i');
  if (idx_param != NULL) {
    idx = strtol(idx_param->value, NULL, 10);
  }

  uint8_t mod = gloria_modulations[0];
    value_t* mod_param = command_get_parameter(&execution, 'm');
  if (mod_param != NULL) {
    mod = strtol(mod_param->value, NULL, 10);
  }

  int8_t pwr = gloria_powers[gloria_default_power_levels[mod]];
  value_t* pwr_param = command_get_parameter(&execution, 'o');
  if (pwr_param != NULL) {
    pwr = strtol(pwr_param->value, NULL, 10);
  }

  uint32_t cad = 0;
    value_t* cad_param = command_get_parameter(&execution, 'c');
  if (cad_param != NULL) {
    cad = strtol(cad_param->value, NULL, 10);
  }

  uint8_t retr = gloria_default_retransmissions[mod];
  value_t* retr_param = command_get_parameter(&execution, 'r');
  if (retr_param != NULL) {
    retr = strtol(retr_param->value, NULL, 10);
  }

  uint8_t slots = gloria_default_data_slots[mod];
  value_t* slots_param = command_get_parameter(&execution, 'l');
  if (slots_param != NULL) {
    slots = strtol(slots_param->value, NULL, 10);
  }

  uint8_t macks = gloria_default_acks[mod];
  value_t* macks_param = command_get_parameter(&execution, 'k');
  if (macks_param != NULL) {
    macks = strtol(macks_param->value, NULL, 10);
  }

  uint8_t sync = 0;
  value_t* sync_param = command_get_parameter(&execution, 's');
  if (sync_param != NULL) {
    sync = strtol(sync_param->value, NULL, 10);
  }

  uint8_t cont = 0;
  value_t* cont_param = command_get_parameter(&execution, 'u');
  if (cont_param != NULL) {
    cont = strtol(cont_param->value, NULL, 10);
  }

  uint32_t txper = 0;
  value_t* txper_param = command_get_parameter(&execution, 'x');
  if (txper_param != NULL) {
    txper = strtol(txper_param->value, NULL, 10);
  }

  uint32_t guard = 0;
  value_t* guard_param = command_get_parameter(&execution, 'g');
  if (guard_param != NULL) {
    guard = strtol(guard_param->value, NULL, 10);
  }

#ifdef RADIO_LOG
  set_radio_log(true);
#else
  set_radio_log(false);
#endif

  message.header.sync = sync;

  flood.marker = marker;
  flood.modulation = mod;
  flood.power = pwr;
  flood.band = RADIO_DEFAULT_BAND;
  flood.flood_idx = idx;
  flood.max_retransmissions = retr;
  flood.ack_mode = ack_mode;
  flood.max_acks = macks;
  flood.data_slots = slots;
  flood.message = &message;
  flood.initial = false;
  flood.rx_timeout = cad * HS_TIMER_FREQUENCY_MS;
  flood.sync_timer = (bool) sync;
  flood.lp_listening = lp_listening;
  flood.guard_time = guard * HS_TIMER_FREQUENCY_US;

  if (cont) {
    tx_period = txper;
    guard_time = guard;
    gloria_run_flood(&flood, &gloria_cont_callback);
  }
  else {
    gloria_run_flood(&flood, &gloria_finish_callback);
  }

  return CMD_RET_SUCCESS;
}

static command_return_t gloria_tx_command_handler(command_execution_t execution) {
  uint8_t payload_size = 0;
  uint8_t* payload = NULL;
  value_t* payload_param = command_get_parameter(&execution, 'p');
  if (payload_param != NULL) {
    payload_size = payload_param->length + 1;
    payload = (uint8_t*) payload_param->value;
  }

  uint64_t marker = ((hs_timer_get_current_timestamp() + GLORIA_RADIO_WAKEUP_TIME + HS_TIMER_FREQUENCY / 32 + (GLORIA_SCHEDULE_GRANULARITY - 1))) / GLORIA_SCHEDULE_GRANULARITY * GLORIA_SCHEDULE_GRANULARITY;
  value_t* marker_delta_param = command_get_parameter(&execution, 't');
  if (marker_delta_param != NULL) {
    marker = gloria_last_sync + strtol(marker_delta_param->value, NULL, 10);
    if (marker - hs_timer_get_current_timestamp() > INT64_MAX)
    {
      if (cli_interactive_mode) {
      }
      cli_log_inline("Delta marker is in the past!", CLI_LOG_LEVEL_ERROR, true, true, true);

      return CMD_RET_FAILURE;
    }
  }

  uint8_t dst = 0;
  value_t* dst_param = command_get_parameter(&execution, 'd');
  if (dst_param != NULL) {
    dst = strtol(dst_param->value, NULL, 10);
  }

  uint8_t mod = gloria_modulations[0];
  value_t* mod_param = command_get_parameter(&execution, 'm');
  if (mod_param != NULL) {
    mod = strtol(mod_param->value, NULL, 10);
  }

  uint16_t idx = 0;
  value_t* idx_param = command_get_parameter(&execution, 'i');
  if (idx_param != NULL) {
    idx = strtol(idx_param->value, NULL, 10);
  }

  uint8_t ack_mode = 0;
  value_t* ack_param = command_get_parameter(&execution, 'a');
  if (ack_param != NULL) {
    ack_mode = strtol(ack_param->value, NULL, 10);
  }

  int8_t pwr = gloria_powers[gloria_default_power_levels[mod]];
  value_t* pwr_param = command_get_parameter(&execution, 'o');
  if (pwr_param != NULL) {
    pwr = strtol(pwr_param->value, NULL, 10);
  }

  uint8_t retr = gloria_default_retransmissions[mod];
  value_t* retr_param = command_get_parameter(&execution, 'r');
  if (retr_param != NULL) {
    retr = strtol(retr_param->value, NULL, 10);
  }

  uint8_t slots = gloria_default_data_slots[mod];
  value_t* slots_param = command_get_parameter(&execution, 'l');
  if (slots_param != NULL) {
    slots = strtol(slots_param->value, NULL, 10);
  }

  uint8_t macks = gloria_default_acks[mod];
  value_t* macks_param = command_get_parameter(&execution, 'k');
  if (macks_param != NULL) {
    macks = strtol(macks_param->value, NULL, 10);
  }

  uint8_t sync = 0;
  value_t* sync_param = command_get_parameter(&execution, 's');
  if (sync_param != NULL) {
    sync = strtol(sync_param->value, NULL, 10);
  }

  uint8_t cont = 0;
  value_t* cont_param = command_get_parameter(&execution, 'u');
  if (cont_param != NULL) {
    cont = strtol(cont_param->value, NULL, 10);
  }

  uint32_t txper = 0;
  value_t* txper_param = command_get_parameter(&execution, 'x');
  if (txper_param != NULL) {
    txper = strtol(txper_param->value, NULL, 10);
  }

  uint32_t iters = 0;
  value_t* iters_param = command_get_parameter(&execution, 'n');
  if (iters_param != NULL) {
    iters = strtol(iters_param->value, NULL, 10);
  }

#ifdef RADIO_LOG
  set_radio_log(true);
#else
  set_radio_log(false);
#endif

  message.header.type = GLORIA_MESSAGE;
  message.header.dst = dst;
  message.header.sync = sync;

  memcpy(message.payload, payload, payload_size);

  flood.marker = marker;
  flood.modulation = mod;
  flood.power = pwr;
  flood.band = RADIO_DEFAULT_BAND;
  flood.flood_idx = idx;
  flood.payload_size = payload_size;
  flood.ack_mode = ack_mode;
  flood.max_retransmissions = retr;
  flood.max_acks = macks;
  flood.data_slots = slots;
  flood.message = &message;
  flood.initial = true;

  if (cont) {
    tx_period = txper;
    iterations = iters;
    gloria_run_flood(&flood, &gloria_cont_callback);
  }
  else {
    gloria_run_flood(&flood, &gloria_finish_callback);
  }

  return CMD_RET_SUCCESS;
}

static void sync_callback() {
  FLOCKLAB_PIN_SET(FLOCKLAB_INT1);
  FLOCKLAB_PIN_CLR(FLOCKLAB_INT1);
}

static void gloria_cont_callback() {
  if (flood.initial) {
    gloria_last_sync = flood.marker;
  }
  if (flood.msg_received) {
    hs_timer_generic(flood.reconstructed_marker + gloria_calculate_flood_time(flood.payload_size, flood.modulation, flood.data_slots, flood.message->header.sync, flood.ack_mode) + 50*HS_TIMER_FREQUENCY_MS, &sync_callback);
  }

  memcpy(&print_message, &message, sizeof(gloria_message_t));
  memcpy(&print_flood, &flood, sizeof(gloria_flood_t));
  print_flood.message = &print_message;
  gloria_print_flood(&print_flood);

  flood.marker = flood.reconstructed_marker + tx_period*HS_TIMER_FREQUENCY_MS;
  flood.guard_time = guard_time * HS_TIMER_FREQUENCY_US;
  flood.flood_idx++;

  if (!flood.initial || flood.flood_idx < iterations) {
    gloria_run_flood(&flood, &gloria_cont_callback);
  }
}

static void gloria_finish_callback() {
  if (flood.msg_received) {
    hs_timer_generic(flood.reconstructed_marker + gloria_calculate_flood_time(flood.payload_size, flood.modulation, flood.data_slots, flood.message->header.sync, flood.ack_mode) + 50*HS_TIMER_FREQUENCY_MS, &sync_callback);
  }
  if (flood.initial) {
    gloria_last_sync = flood.marker;
  }

  radio_sleep(true);

  memcpy(&print_message, &message, sizeof(gloria_message_t));
  flood.message = &print_message;
  memcpy(&print_flood, &flood, sizeof(gloria_flood_t));
  gloria_flood_to_be_printed = true;
  //set_radio_log(true);
}


/*
 * Print the flood stored in print_flood if it has changed
 * Useful to call periodically (e.g. from system_update())
 */
void gloria_print_flood_periodic() {
  if (gloria_flood_to_be_printed) {
    gloria_print_flood(&print_flood);
    gloria_flood_to_be_printed = false;
  }
}

/*
 * Print the flood passed as reference
 */

void gloria_print_flood(gloria_flood_t *print_flood) {
  if (cli_interactive_mode) {
    cli_log_inline("",CLI_LOG_LEVEL_DEBUG, true, false, true);
  }

  bool failure = true;

  cJSON_Hooks hooks = {
        .malloc_fn = pvPortMalloc,
        .free_fn = vPortFree,
  };
  cJSON_InitHooks(&hooks);

  cJSON* flood_result = cJSON_CreateObject();
  if (flood_result == NULL) {
    goto end;
  }

  if (cJSON_AddStringToObject(flood_result, "type", "gloria_flood") == NULL) {
    goto end;
  }

  if (cJSON_AddBoolToObject(flood_result, "msg_received", print_flood->msg_received) == NULL) {
    goto end;
  }

  if (cJSON_AddBoolToObject(flood_result, "initial", print_flood->initial) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "fld_idx", print_flood->flood_idx) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "mod", print_flood->modulation) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "slots", print_flood->data_slots) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "retrs", print_flood->max_retransmissions) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "ack_mode", print_flood->ack_mode) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "acks", print_flood->max_acks) == NULL) {
    goto end;
  }

  if (cJSON_AddBoolToObject(flood_result, "crc_error", print_flood->crc_error) == NULL) {
    goto end;
  }

  if (cJSON_AddBoolToObject(flood_result, "crc_timeout", print_flood->crc_timeout) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "msgs_sent", print_flood->max_retransmissions - print_flood->remaining_retransmissions) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "acks_sent", print_flood->ack_counter) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "last_active_idx", print_flood->slot_index-1) == NULL) {
    goto end;
  }

  if (cJSON_AddNumberToObject(flood_result, "msg_pow", print_flood->power) == NULL) {
    goto end;
  }

  if (print_flood->msg_received) {

    if (cJSON_AddNumberToObject(flood_result, "msg_size", print_flood->message_size) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(flood_result, "fld_size", print_flood->payload_size) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(flood_result, "first_rx_index", print_flood->first_rx_index) == NULL) {
      goto end;
    }

    if (cJSON_AddBoolToObject(flood_result, "acked", print_flood->acked) == NULL) {
      goto end;
    }

    if (print_flood->acked) {
      if (cJSON_AddNumberToObject(flood_result, "ack_dst", print_flood->ack_message.dst) == NULL) {
        goto end;
      }
    }

    if (print_flood->message_size > GLORIA_HEADER_LENGTH && cli_string_is_printable((char*) print_flood->message->payload, print_flood->message_size - GLORIA_HEADER_LENGTH))
    {
      if (cJSON_AddStringToObject(flood_result, "msg", (char*) print_flood->message->payload) == NULL) {
        goto end;
      }
    }

    if (cJSON_AddNumberToObject(flood_result, "msg_type", print_flood->message->header.type) == NULL) {
      goto end;
    }

    if (cJSON_AddBoolToObject(flood_result, "msg_sync", print_flood->message->header.sync) == NULL) {
      goto end;
    }

//      if (cJSON_AddNumberToObject(flood_result, "tx_marker", print_flood->current_tx_marker) == NULL) {
//        goto end;
//      }

    if (cJSON_AddNumberToObject(flood_result, "msg_dst", print_flood->message->header.dst) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(flood_result, "msg_src", print_flood->message->header.src) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(flood_result, "rssi", print_flood->rssi) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(flood_result, "snr", print_flood->snr) == NULL) {
      goto end;
    }
  }

  failure = false;
  cli_log_json(flood_result, "gloria", CLI_LOG_LEVEL_DEBUG);

  if (cli_interactive_mode) {
    cli_log_inline("",CLI_LOG_LEVEL_DEBUG, true, true, true);
  }

end:
  if (failure) {
    cJSON_Delete(flood_result);
  }

  return;
}

#endif /* CLI_ENABLE */
