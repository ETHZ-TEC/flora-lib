/*
 * Copyright (c) 2018 - 2022, ETH Zurich, Computer Engineering Group (TEC)
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

#include "flora_lib.h"

#if GLORIA_ENABLE

#ifndef GLORIA_START_IND
  #define GLORIA_START_IND()
  #define GLORIA_STOP_IND()
#endif /* GLORIA_START_IND */

/* internal state */
static gloria_flood_t flood;                                              // flood struct which (serves as input, state, and output to/from gloria_run_flood)
static uint8_t        gloria_payload[GLORIA_INTERFACE_MAX_PAYLOAD_LEN];   // buffer for the message
static bool           flood_running;                                      // indicates whether flood is onging or not
static bool           flood_completed;                                    // indicates whether the flood completed (N_TX reached)
static uint8_t        lastrun_n_rx_started;                               // number of rx started events during the last Gloria run
static bool           lastrun_t_ref_updated;                              // indicates whether last_t_ref has been updated during the last flood
static uint64_t       last_t_ref = 0;                                     // reference time (updated if gloria_start is called with sync_slot=true)
static uint64_t       last_t_ref_hs = 0;                                  // last reference time in hs timer ticks
static int8_t         internal_power = GLORIA_INTERFACE_POWER;            // internal state for power (can be adapted from the upper layer)
static uint8_t        internal_modulation = GLORIA_INTERFACE_MODULATION;  // internal state for the radio modulation (can be adapted from the upper layer)
static uint8_t        internal_band = GLORIA_INTERFACE_RF_BAND;           // internal state for the frequency band (can be adapted from the upper layer)
static bool           internal_enable_flood_printing = false;             // enable printing of finished (i.e. completely received/transmitted) floods
static bool           internal_append_timestamp = GLORIA_INTERFACE_APPEND_TIMESTAMP;

/* variables to store gloria_start arguments */
static uint8_t*       arg_payload_ptr = NULL;                             // pointer to payload of currently ongoing flood
static bool           arg_sync_slot;                                      // holds state whether current flood is used to update last_t_ref or not

/* Private Function Prototypes */
static void flood_callback(void);
static void copy_payload(void);
static void update_t_ref(void);


/* INTERFACE FUNCTIONS ********************************************************/
void gloria_start(bool is_initiator,
                  uint8_t* payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  uint8_t sync_slot)
{
  if (!payload) {
    LOG_WARNING("invalid parameters");
    return;
  }
  if (flood_running) {
    LOG_WARNING("flood is still running");
    return;
  }

  // radio must be woken from sleep mode!
  if (radio_wakeup()) {
    LOG_WARNING("radio was in sleep mode");
  }

  // argument checks
  if (payload_len > GLORIA_INTERFACE_MAX_PAYLOAD_LEN) {
    if (is_initiator) {
      LOG_WARNING("payload_len passed to gloria_start as initiator exceeds limit! payload will be truncated");
    } else {
      LOG_WARNING("payload_len passed to gloria_start as receiver exceeds limit! payload will be truncated");
    }
    payload_len = GLORIA_INTERFACE_MAX_PAYLOAD_LEN;
  }

  GLORIA_START_IND();

  // store arguments for further use
  arg_payload_ptr = payload;
  arg_sync_slot   = sync_slot;

  // initialize internal state
  flood_running         = true;  // keep ordering: first internal state variable to update here
  flood_completed       = false;
  lastrun_n_rx_started  = 0;
  lastrun_t_ref_updated = false;
  // last_t_ref: not initialized here since old values of previous floods are still valid and useful if current flood does not update the value

  flood.ack_mode              = 0;
  flood.band                  = internal_band;
  flood.data_slots            = GLORIA_INTERFACE_MAX_SLOTS;
  flood.guard_time            = 0;
  flood.header.protocol_id    = PROTOCOL_ID_GLORIA;
  flood.header.type           = 0;
  flood.header.sync           = (internal_append_timestamp != 0);   // no sync flood (i.e. timestamp for absolute sync to initiator is not included in to payload)
  flood.header.slot_index     = 0;
  flood.initiator             = is_initiator;
  flood.lp_listening          = false;
  flood.max_retransmissions   = n_tx_max;
  flood.modulation            = internal_modulation;
  flood.payload               = gloria_payload;
  flood.payload_size          = payload_len;
  flood.power                 = internal_power;
  flood.radio_no_sleep        = true;
  flood.rem_retransmissions   = n_tx_max;
  flood.rssi                  = 0;
  flood.rx_timeout            = 0;
  flood.snr                   = 0;
  flood.sync_timer            = false;    // do not automatically adjust the hs timer offset

  // unused fields or internal state of Gloria (initialized in gloria_run_flood)
  flood.acked                 = false;
  flood.ack_counter           = 0;
  flood.crc_error             = false;
  flood.crc_timeout           = false;
  flood.current_tx_marker     = 0;
  flood.first_rx_index        = 0;
  flood.flood_idx             = 0;
  flood.header_size           = 0;
  flood.last_active_slot      = 0;
  flood.max_acks              = 0;
  flood.msg_received          = false;
  flood.node_id               = 0;
  flood.received_marker       = 0;
  flood.reconstructed_marker  = 0;
  flood.stop                  = false;
  memset(&flood.ack_message, 0, sizeof(gloria_ack_msg_t));

  // don't change the following fields (they are reset in gloria_stop and can be modified by the user prior to the call to gloria_start)
  //flood.flood_cb            = NULL;
  //flood.filter_cb           = NULL;
  //flood.rx_cb               = NULL;
  //flood.tx_delay_slots      = 0;

  if (is_initiator) {
    // send flood
    // set the TX marker (timestamp when flood shall start), must be set on the initiator
    if (flood.marker < hs_timer_get_current_timestamp()) {
      // use current timestamp
      flood.marker = ((hs_timer_get_current_timestamp() + (GLORIA_SCHEDULE_GRANULARITY - 1))) / GLORIA_SCHEDULE_GRANULARITY * GLORIA_SCHEDULE_GRANULARITY;
    }
    memcpy(gloria_payload, payload, payload_len);
  }
  else {
    // receive flood
    flood.marker = 0;
    // clear the receive buffer
    memset(payload, 0, payload_len);
  }

  radio_reset_preamble_counter();
  radio_reset_sync_counter();

#if GLORIA_INTERFACE_DISABLE_INTERRUPTS
  HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  HAL_SuspendTick();
  SUSPEND_SYSTICK();
#endif /* GLORIA_INTERFACE_DISABLE_INTERRUPTS */

  gloria_run_flood(&flood, &flood_callback);
}


uint8_t gloria_stop(void)
{
  // only stop if flood is not terminated yet
  if (flood_running) {

    if (!flood_completed && flood.initiator) {
      // if this node is initiator, we can detect if flood did not terminate and warn the user
      LOG_WARNING("Stopping glossy while flood sending is still ongoing!");
    }

#if GLORIA_INTERFACE_WAIT_TX_FINISHED
    // if a transmission is ongoing, wait for it to complete
    flood.stop = true;
    uint64_t timeout = hs_timer_get_current_timestamp() + gloria_calculate_slot_time(internal_modulation, 0, 0, GLORIA_INTERFACE_MAX_PAYLOAD_LEN);
    while ((radio_get_status() == RF_TX_RUNNING) && !flood_completed && (hs_timer_get_current_timestamp() < timeout));
#endif /* GLORIA_INTERFACE_WAIT_TX_FINISHED */

    // stop gloria timers
    hs_timer_schedule_stop();
    hs_timer_timeout_stop();

    // reset radio callbacks
    radio_set_timeout_callback(NULL);
    radio_set_rx_callback(NULL);

    // put radio in standby mode (use critical section to make sure no interrupt can abort this command)
    ENTER_CRITICAL_SECTION();
    radio_standby();
    LEAVE_CRITICAL_SECTION();

    // set internal state for the case nothing has been received
    lastrun_t_ref_updated = false;

    // overwrite defaults if at least parts of the flood have been received
    if (flood.msg_received && !flood.initiator) {
      // node received at least one message => update received values
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

    // clear arg variables
    arg_payload_ptr      = NULL;
    arg_sync_slot        = false;
    flood.flood_cb       = NULL;
    flood.filter_cb      = NULL;
    flood.rx_cb          = NULL;
    flood.tx_delay_slots = 0;
    flood.marker         = 0;

  #if GLORIA_INTERFACE_DISABLE_INTERRUPTS
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_ResumeTick();
    RESUME_SYSTICK();
  #endif /* GLORIA_INTERFACE_DISABLE_INTERRUPTS */

    flood_running = false; // keep ordering: last internal state variable updated here

    GLORIA_STOP_IND();
  }

  return flood.msg_received;
}


uint8_t gloria_get_rx_cnt(void)
{
  return flood.msg_received;
}


uint8_t gloria_get_payload_len(void)
{
  return flood.payload_size;
}


uint8_t gloria_get_rx_index(void)
{
  return flood.first_rx_index;
}


uint8_t gloria_get_rx_started_cnt(void)
{
  return lastrun_n_rx_started;
}


uint8_t gloria_is_t_ref_updated(void)
{
  return lastrun_t_ref_updated;
}


uint64_t gloria_get_t_ref(void)
{
  return last_t_ref;
}


uint64_t gloria_get_t_ref_hs(void)
{
  return last_t_ref_hs;
}


int32_t gloria_get_rssi(void)
{
  return flood.rssi;
}


int32_t gloria_get_snr(void)
{
  return flood.snr;
}


/* EXTENDED INTERFACE *********************************************************/

void gloria_set_tx_power(int8_t power)
{
  internal_power = power;
}


void gloria_set_modulation(uint8_t modulation)
{
  if (modulation < RADIO_NUM_MODULATIONS) {
    internal_modulation = modulation;
  }
}


void gloria_set_band(uint8_t band)
{
  internal_band = band;
}


uint32_t gloria_get_toa(uint8_t payload_len)
{
  return gloria_get_toa_sl(payload_len, internal_modulation);
}


uint32_t gloria_get_toa_sl(uint8_t payload_len, uint8_t modulation)
{
  return radio_get_toa(payload_len + GLORIA_HEADER_LENGTH_MIN, modulation);
}


uint32_t gloria_get_flood_time(uint8_t payload_len, uint8_t num_slots)
{
  return gloria_get_flood_time_sl(payload_len, internal_modulation, num_slots);
}


uint32_t gloria_get_flood_time_sl(uint8_t payload_len, uint8_t modulation, uint8_t num_slots)
{
  uint64_t ret = gloria_calculate_flood_time(
    payload_len,
    modulation,
    num_slots,
    false,        // sync
    0             // ack_mode
  );
  return HS_TIMER_TICKS_TO_US(ret);
}


void gloria_enable_flood_printing(bool enable)
{
  internal_enable_flood_printing = enable;
}


void gloria_register_flood_callback(gloria_flood_cb_t flood_cb)
{
  flood.flood_cb = flood_cb;
}


void gloria_set_pkt_filter(gloria_filter_cb_t filter_cb)
{
  flood.filter_cb = filter_cb;
}


void gloria_register_rx_callback(gloria_rx_cb_t rx_cb)
{
  flood.rx_cb = rx_cb;
}


void gloria_set_tx_marker(uint64_t timestamp_hs)
{
  flood.marker = timestamp_hs;
}


void gloria_set_tx_delay(uint8_t delay_slots)
{
  flood.tx_delay_slots = delay_slots;
}


void gloria_enable_append_timestamp(bool enable)
{
  internal_append_timestamp = enable;
}


bool gloria_get_received_timestamp(uint8_t* out_timestamp)
{
  if (out_timestamp && flood.header.sync) {
    memcpy(out_timestamp, flood.header.min.timestamp, GLORIA_TIMESTAMP_LENGTH);
    return true;
  }
  return false;
}



/* PRIVATE FUNCTIONS **********************************************************/

static void flood_callback(void)
{
  gloria_flood_cb_t cb = flood.flood_cb;

  // flood completed
  flood_completed = true;
  gloria_stop();

  // call callback function registered via the gloria_interface
  if (cb) {
    cb();
  }
}


/*
 * Updates the buffer to which arg_payload_ptr points with the values from the global flood
 * struct.
 */
static void copy_payload(void)
{
  if (arg_payload_ptr) {
    // copy payload to provided memory space
    if (flood.payload_size <= GLORIA_INTERFACE_MAX_PAYLOAD_LEN) {
      memcpy(arg_payload_ptr, flood.payload, flood.payload_size);
    }
    else {
      LOG_WARNING("Payload length in received message is larger than payload length of Gloria interface (GLORIA_INTERFACE_MAX_PAYLOAD_LEN)! Payload has been truncated!");
      memcpy(arg_payload_ptr, flood.payload, GLORIA_INTERFACE_MAX_PAYLOAD_LEN);
      flood.payload_size = GLORIA_INTERFACE_MAX_PAYLOAD_LEN;
    }
  }
}


/*
 * Updates the last_t_ref and lastrun_t_ref_updated based on the flood.reconstructed_marker
 */
static void update_t_ref(void)
{
  lastrun_t_ref_updated = false;
  if (flood.reconstructed_marker == 0) {
    LOG_WARNING("Tried to update t_ref with flood.reconstructed_marker==0!");
    return;
  }

  // get sync point
  uint64_t hs_sync_point = 0;
  uint64_t lp_sync_point = 0;
  if (!lptimer_now_synced(&lp_sync_point,  &hs_sync_point)) {
    LOG_ERROR("failed to get sync point");
    return;
  }

  // determine time difference between reconstructed_marker and sync point
  uint64_t lp_time_diff = (hs_sync_point - flood.reconstructed_marker) / HS_TIMER_FREQUENCY_US * LPTIMER_SECOND / 1000000;

  // determine the the reconstructed_marker time in lptimer ticks
  /* update t_ref related internal state */
  last_t_ref            = lp_sync_point - lp_time_diff;
  last_t_ref_hs         = flood.reconstructed_marker;
  lastrun_t_ref_updated = true;
}

#endif /* GLORIA_ENABLE */
