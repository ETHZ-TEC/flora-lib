/*
 * gloria_interface.c
 *
 *  Created on: 04.12.2019
 *      Author: Roman Trub
 */

#include "flora_lib.h"

#if GLORIA_ENABLE

#ifndef GLORIA_START_IND
  #define GLORIA_START_IND()
  #define GLORIA_STOP_IND()
#endif /* GLORIA_START_IND */


/*******************************************************************************
 * BEGIN: GLORIA INTERFACE
 ******************************************************************************/

extern radio_sleeping_t radio_sleeping;

/* internal state */
static gloria_flood_t   flood;                                   // flood struct which (serves as input, state, and output to/from gloria_run_flood)
static gloria_message_t message;                                 // buffer for the message (static to avoid allocation on the stack)
static bool     flood_running;                                   // indicates whether flood is onging or not
static uint8_t  lastrun_msg_received = 0;                        // number of times a message has been received during the last gloria run
static uint8_t  lastrun_payload_len = 0;                         // length of the payload received during the last gloria run (0 if no payload has been received)
static uint8_t  lastrun_rx_index = 0;                            // slot index (slot in which the message was received (corresponds to relay counter in Glossy terminology)
static uint8_t  lastrun_n_rx_started = 0;                        // number of rx started events during the last Gloria run
static bool     lastrun_t_ref_updated = false;                   // indicates whether last_t_ref has been updated during the last flood
static uint64_t last_t_ref = 0;                                  // reference time (updated if gloria_start is called with sync_slot=true)
static int8_t   internal_power = GLORIA_INTERFACE_POWER;         // internal state for power (can be adapted from the GMW layer)
static uint8_t  internal_modulation = GLORIA_INTERFACE_MODULATION;  // internal state for the radio modulation (can be adapted from the GMW layer)
static uint8_t  internal_band = GLORIA_INTERFACE_RF_BAND;        // internal state for the frequency band (can be adapted from the GMW layer)
static bool     internal_enable_flood_printing = false;          // enable printing of finished (i.e. completely received/transmitted) floods
static gloria_cb_func_t flood_cb = 0;                            // user-defined callback; only called if flood participation terminates before gloria_stop() is called

/* variables to store gloria_start arguments */
static uint16_t arg_initiator_id = 0;               // ID of the inititator
static uint8_t *arg_payload_ptr = NULL;             // pointer to payload of currently ongoing flood
static uint8_t  arg_payload_len = 0;                // length of the memory of the payload variable provided as arg to gloria_start (can be truncated if provided payload_len exceeds GLORIA_MAX_PAYLOAD_LENGTH)
static bool     arg_sync_slot;                      // holds state whether current flood is used to update last_t_ref or not


/* Private Function Prototypes */
static void gloria_flood_callback(void);
static void copy_payload(void);
static void update_t_ref(void);

/* INTERFACE FUNCTIONS ********************************************************/
void gloria_start(uint16_t initiator_id,
                  uint8_t *payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  uint8_t sync_slot)
{
  if (!payload) {
    LOG_WARNING("invalid parameters");
    return;
  }

  /* radio must be woken from sleep mode! */
  if (radio_sleeping) {
    LOG_WARNING("radio is in sleep mode");
    return;
  }

  /* argument checks */
  if (payload_len > GLORIA_INTERFACE_MAX_PAYLOAD_LEN) {
    if (initiator_id == NODE_ID) {
      LOG_WARNING("payload_len passed to gloria_start as initiator exceeds limit (GLORIA_MAX_PAYLOAD_LENGTH)! Payload will be truncated before sending!");
    } else {
      LOG_WARNING("payload_len passed to gloria_start as receiver exceeds limit (GLORIA_MAX_PAYLOAD_LENGTH)! Payload will be truncated before returning!");
    }
    arg_payload_len = GLORIA_INTERFACE_MAX_PAYLOAD_LEN;
  }
  else {
    arg_payload_len = payload_len;
  }

  GLORIA_START_IND();

  /* store arguments for further use */
  arg_initiator_id = initiator_id;
  arg_payload_ptr = payload;
  // arg_payload_len: stored during argument check
  arg_sync_slot = sync_slot;

  /* initialize internal state */
  flood_running = true;  // keep ordering: first internal state variable to update here
  lastrun_msg_received = 0;
  lastrun_payload_len = 0;
  lastrun_rx_index = 0;
  lastrun_n_rx_started = 0;
  lastrun_t_ref_updated = false;
  // last_t_ref: not initialized here since old values of previous floods are still valid and useful if current flood does not update the value

  /* prepare flood struct */
  flood.marker = 0;
  flood.modulation = internal_modulation;
  flood.power = internal_power;
  flood.band = internal_band;
  flood.flood_idx = 0;
  flood.max_retransmissions = n_tx_max;
  flood.ack_mode = 0;
  flood.max_acks = 0;
  flood.data_slots = GLORIA_INTERFACE_MAX_SLOTS;
  flood.sync_timer = 0;
  flood.lp_listening = false;
  flood.radio_no_sleep = true;

  message.header.type = 0;
  message.header.sync = 0;  // no sync flood (i.e. timestamp for absolute sync to initiator is not included in to payload)
  message.header.slot_index = 0;
  // flood.reconstructed_marker: initialization not necessary -> initialized in gloria_run_flood()

  if (initiator_id == NODE_ID) {
    // send flood
    uint64_t marker = ((hs_timer_get_current_timestamp() + (GLORIA_SCHEDULE_GRANULARITY - 1))) / GLORIA_SCHEDULE_GRANULARITY * GLORIA_SCHEDULE_GRANULARITY;
    flood.marker = marker;    // marker (timestamp when flood shall start) must be set on the initiator
    message.header.dst = 0;   // 0 means broadcast
    flood.initial = true;     // this node is the initator
    memcpy(message.payload, payload, arg_payload_len);
    flood.message = &message;
    flood.payload_size = arg_payload_len;
  }
  else {
    // receive flood
    flood.message = &message;
    flood.marker = 0;
    flood.rx_timeout = 0;
    flood.guard_time = 0;   //10000 * HS_TIMER_FREQUENCY_US;
    flood.initial = false;
  }

  radio_reset_preamble_counter();
  radio_reset_sync_counter();

  gloria_run_flood(&flood, &gloria_flood_callback);
}

uint8_t gloria_stop(void)
{
  // only stop if flood is not terminated yet
  if (flood_running) {
    if (arg_initiator_id == NODE_ID) {
      // If this node is initiator, we can detect if flood did not terminate and warn the user
      LOG_WARNING("Stopping glossy while flood sending is still ongoing!");
    }
    else {
      //LOG_VERBOSE("Stopping glossy before finishing to participate in the flood!");
    }

    // put radio in standby mode
    radio_set_standby();

    // Stop gloria timers
    hs_timer_schedule_stop();
    // FIXME: clear corresponding interrupt flags (if any)
    // FIXME: reset/cleanup state of current flood -> not necessary? All state is contained in flood variable which is passed by reference to gloria_run_flood()

    // Clear interrupt flags (which potentially could prevent the MCU from sleeping or cause a higher sleep current)
    __HAL_GPIO_EXTI_CLEAR_IT(RADIO_DIO1_WAKEUP_Pin);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

    flood_running = false; // keep ordering: last internal state variable updated here
  }

  /* Set internal state for the case nothing has been received */
  lastrun_msg_received = 0;
  lastrun_payload_len = 0;
  lastrun_rx_index = 0;
  lastrun_t_ref_updated = false;

  /* Overwrite defaults if at least parts of the flood have been received */
  if (flood.msg_received && !flood.initial) {
    // node received at least one message => update received values
    lastrun_msg_received = flood.msg_received;
    lastrun_payload_len = flood.message_size - GLORIA_HEADER_LENGTH;
    lastrun_rx_index = flood.first_rx_index;
    copy_payload();
  }
  if (flood.msg_received && arg_sync_slot) {
    update_t_ref();
  }

  if (radio_modulations[flood.modulation].modem == MODEM_LORA) {
    lastrun_n_rx_started = radio_get_preamble_counter();
  } else {
    lastrun_n_rx_started = radio_get_sync_counter();
  }

  // DEBUG: print flood struct
  if (internal_enable_flood_printing) {
#if CLI_ENABLE
    gloria_print_flood(&flood);
#endif /* CLI_ENABLE */
  }

  /* clear arg variables */
  arg_initiator_id = 0;
  arg_payload_ptr = NULL;
  arg_payload_len = 0;
  arg_sync_slot = false;

  GLORIA_STOP_IND();

  return lastrun_msg_received;
}

uint8_t gloria_get_rx_cnt(void) {
  return lastrun_msg_received;
}

uint8_t gloria_get_payload_len(void) {
  return lastrun_payload_len;
}

uint8_t gloria_get_rx_index(void) {
  return lastrun_rx_index;
}

uint8_t gloria_get_rx_started_cnt(void) {
  return lastrun_n_rx_started;
}

uint8_t gloria_is_t_ref_updated(void){
  return lastrun_t_ref_updated;
}

uint64_t gloria_get_t_ref(void){
  return last_t_ref;
}

uint32_t gloria_get_flood_time(void)
{
  return gloria_calculate_flood_time(&flood);
}

/* EXTENDED INTERFACE *********************************************************/

void gloria_set_tx_power(int8_t power) {
  internal_power = power;
}

void gloria_set_modulation(uint8_t modulation) {
  internal_modulation = modulation;
}

void gloria_set_band(uint8_t band) {
  internal_band = band;
}

uint32_t gloria_get_time_on_air(uint8_t payload_len) {
  // TODO: arg check
  uint32_t toa = 10000000UL; // default initialization

  uint8_t phyPl = payload_len + GLORIA_HEADER_LENGTH;

  if (radio_modulations[internal_modulation].modem == MODEM_LORA) {
    // LoRa modulation
    uint8_t sf = radio_modulations[internal_modulation].datarate;
    double syncSyms = sf <= 6 ? 6.25 : 4.25;
    uint8_t nPreambleSyms = radio_modulations[internal_modulation].preambleLen;
    uint8_t coderate = radio_modulations[internal_modulation].coderate;

    int32_t arg1 = 8*phyPl + 16 - 4*sf + 20;
    int32_t ceilPart = ceil(MAX(arg1, 0)/(4*sf));
    double nSymbol = nPreambleSyms + syncSyms + 8 + ceilPart*(coderate + 4);
    toa = pow(2, sf)*nSymbol * 1000000UL / 125000;
  } else if (radio_modulations[internal_modulation].modem == MODEM_FSK) {
    // FSK modulation
    uint32_t bitrate = radio_modulations[internal_modulation].datarate;
    uint8_t nPreambleBits = 8*radio_modulations[internal_modulation].preambleLen;
    // NOTE: addr is disabled -> length is assumed to be 0 bytes
    // syncword=3, length=1, address=0, crc=2 (in bytes)
    uint32_t nBits = nPreambleBits + 3*8 + 1*8 + 0*8 + phyPl*8 + 2*8;
    toa = nBits * 1000000UL / bitrate;
  } else {
    LOG_WARNING("Unknown modulation!");
  }
  return toa;
}

void gloria_enable_flood_printing(bool enable) {
  internal_enable_flood_printing = enable;
}

void gloria_register_flood_callback(gloria_cb_func_t cb) {
  flood_cb = cb;
}



/* PRIVATE FUNCTIONS **********************************************************/

static void gloria_flood_callback(void) {
  // unsetting flood_running indicates to gloria_stop that the flood completed
  flood_running = false;

  // call callback function registered via the gloria_interface
  if (flood_cb) {
    flood_cb();
  }
}


/*
 * Updates lastrun_msg_received, lastrun_payload_len, lastrun_rx_index, and the
 * buffer to which arg_payload_ptr points with the values from the global flood
 * struct.
 */
static void copy_payload(void) {
  /* copy payload to provided memory space */
  // check if provided payload buffer space is large enough
  if (lastrun_payload_len <= GLORIA_INTERFACE_MAX_PAYLOAD_LEN) {
    memcpy(arg_payload_ptr, flood.message->payload, lastrun_payload_len);
  }
  else {
    LOG_WARNING("Payload length in received message is larger than payload length of Gloria interface (GLORIA_MAX_PAYLOAD_LENGTH)! Payload has been truncated!");
    memcpy(arg_payload_ptr, flood.message->payload, GLORIA_INTERFACE_MAX_PAYLOAD_LEN);
    lastrun_payload_len = GLORIA_INTERFACE_MAX_PAYLOAD_LEN;
  }
}

/*
 * Updates the last_t_ref and lastrun_t_ref_updated based on the flood.reconstructed_marker
 */
static void update_t_ref(void) {
  lastrun_t_ref_updated = false;
  if (flood.reconstructed_marker == 0) {
    LOG_WARNING("Tried to update t_ref with flood.reconstructed_marker==0!");
    return;
  }

  bool sync_point_acquired = false;
  uint64_t hs_sync_point = 0;
  uint64_t lp_sync_point = 0;
  int64_t hs_sync_point_check = 0;

  // get sync point
  while (!sync_point_acquired) {
    hs_sync_point = hs_timer_get_current_timestamp();
    lp_sync_point = lptimer_now();

    // Read timer values again to make sure that timers were not updated while reading
    uint64_t hs_tmp = hs_timer_get_current_timestamp();

    hs_sync_point_check = hs_tmp - hs_sync_point;

    if (hs_sync_point_check >= 0 &&
        hs_sync_point_check < HS_TIMER_FREQUENCY_MS) {
      sync_point_acquired = true;
    }
  }

  // determine time difference between reconstructed_marker and sync point
  uint64_t lp_time_diff = (hs_sync_point - flood.reconstructed_marker) / HS_TIMER_FREQUENCY_US * LPTIMER_SECOND / 1000000;

  // determine the the reconstructed_marker time in lptimer ticks
  /* update t_ref related internal state */
  last_t_ref = lp_sync_point - lp_time_diff;
  lastrun_t_ref_updated = true;
}

/*******************************************************************************
 * END: GLORIA INTERFACE
 ******************************************************************************/

#endif /* GLORIA_ENABLE */
