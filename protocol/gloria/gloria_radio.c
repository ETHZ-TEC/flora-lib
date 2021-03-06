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

static uint64_t             rx_timeout_ts = 0;
static gloria_radio_state_t state;
static gloria_flood_t*      current_flood = NULL;

static void (*callback)(void)                              = NULL;
static void (*rx_callback)(uint8_t* payload, uint8_t size) = NULL;

static uint64_t gloria_try_to_sleep(uint64_t future_timestamp);
static void     gloria_radio_setup_callback(void);
static void     gloria_radio_tx_callback(void);
static void     gloria_radio_tx_ack_callback(void);
static void     gloria_radio_rx_timeout_callback(bool crc_error);
static void     gloria_radio_rx_callback(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error);
static void     gloria_radio_continue_rx(void);

/*
 * calculate tx marker for data message and set timer for radio_setup
 */
void gloria_tx(gloria_flood_t* flood, void (*tx_callback)(void))
{
  state         = GLORIA_RADIO_TX;
  current_flood = flood;
  callback      = tx_callback;

  uint64_t setup_timestamp = gloria_calculate_tx_marker(flood);

  if (flood->radio_no_sleep) {
    gloria_radio_setup_callback();
  }
  else {
    setup_timestamp = setup_timestamp - GLORIA_TX_SETUP - GLORIA_TIME_BUFFER;
    setup_timestamp = gloria_try_to_sleep(setup_timestamp);

    switch (setup_timestamp) {
      case 1:
        // go directly to radio setup
        gloria_radio_setup_callback();
        break;
      case 0:
        // skip slot
        LOG_WARNING("Tx too late!");
        callback();
        break;
      default:
        // schedule radio wakeup for rx setup
        hs_timer_schedule_start(setup_timestamp, &gloria_radio_setup_callback);
        break;
    }
  }
}

/*
 * calculate tx marker for ack message and set timer for radio_setup
 */
void gloria_tx_ack(gloria_flood_t* flood, void (*tx_callback)(void))
{
  state         = GLORIA_RADIO_ACK_TX;
  current_flood = flood;
  callback      = tx_callback;

  uint64_t setup_timestamp = gloria_calculate_tx_marker(flood);

  if (flood->radio_no_sleep) {
    gloria_radio_setup_callback();
  }
  else {
    setup_timestamp = setup_timestamp - GLORIA_TX_SETUP - GLORIA_TIME_BUFFER;
    setup_timestamp = gloria_try_to_sleep(setup_timestamp);

    switch (setup_timestamp) {
      case 1:
        // go directly to radio setup
        gloria_radio_setup_callback();
        break;
      case 0:
        // skip slot
        LOG_WARNING("Tx ack too late!");
        callback();
        break;
      default:
        // schedule radio wakeup for rx setup
        hs_timer_schedule_start(setup_timestamp, &gloria_radio_setup_callback);
        break;
    }
  }
}

/*
 * calculate rx marker and set timer for radio_setup
 */
void gloria_rx(gloria_flood_t* flood, void (*callback)(uint8_t*, uint8_t))
{
  state         = GLORIA_RADIO_RX;
  current_flood = flood;
  rx_callback   = callback;

  if (current_flood->msg_received || current_flood->marker) {

    uint64_t setup_timestamp = gloria_calculate_rx_marker(flood);

    if (flood->radio_no_sleep) {
      gloria_radio_setup_callback();
    }
    else {
      setup_timestamp = setup_timestamp - GLORIA_RX_SETUP - GLORIA_TIME_BUFFER;
      setup_timestamp = gloria_try_to_sleep(setup_timestamp);

      switch (setup_timestamp) {
        case 1:
          // go directly to radio setup
          gloria_radio_setup_callback();
          break;
        case 0:
          if (!current_flood->msg_received && !current_flood->lp_listening) {
            // if this is the first rx, start listening anyhow
            gloria_radio_setup_callback();
          }
          else {
            // skip slot
            LOG_WARNING("Rx too late!");
            rx_callback(NULL, 0);
          }
          break;
        default:
          // schedule radio wakeup for rx setup
          hs_timer_schedule_start(setup_timestamp, &gloria_radio_setup_callback);
          break;
      }
    }
  }
  else {
    gloria_radio_setup_callback();
  }
}

/*
 * sets radio in sleep mode if the time until the future_timestamp is enough
 * returns the future_timesamp - the wake-up time of the radio, if time is enough to sleep
 * returns 1 if there is no time to sleep but the future_timestamp has not yet passed
 * returns 0 if the future_timesamp has already passed
 */
static uint64_t gloria_try_to_sleep(uint64_t future_timestamp)
{
  uint64_t current_timestamp = hs_timer_get_current_timestamp();
  uint64_t time_diff         = future_timestamp - current_timestamp;

  if (time_diff <= INT64_MAX) {
    if (time_diff > (GLORIA_RADIO_SLEEP_TIME_COLD + GLORIA_RADIO_WAKEUP_TIME_COLD)) {
      radio_sleep(false);
      return future_timestamp - GLORIA_RADIO_WAKEUP_TIME_COLD;
    }
    else if (time_diff > (GLORIA_RADIO_SLEEP_TIME + GLORIA_RADIO_WAKEUP_TIME)) {
      radio_sleep(true);
      return future_timestamp - GLORIA_RADIO_WAKEUP_TIME;
    }
    else {
      return 1;
    }
  }
  else {
    return 0;
  }
}

/*
 * radio setup and start of tx/rx
 */
static void gloria_radio_setup_callback(void)
{
  if (!current_flood->radio_no_sleep) {
    radio_wakeup();
  }

  switch (state) {
    case GLORIA_RADIO_TX:
      radio_set_irq_mode(IRQ_MODE_TX);
      // NOTE: TX config is set in gloria_run_flood()
      radio_set_payload_chunk((uint8_t*)&current_flood->header, 0, current_flood->header_size, false);
      radio_set_payload_chunk((uint8_t*)current_flood->payload, current_flood->header_size, current_flood->payload_size, true);
      radio_set_tx_callback(&gloria_radio_tx_callback);
      radio_transmit_scheduled(0, 0, current_flood->current_tx_marker);
      break;

    case GLORIA_RADIO_ACK_TX:
      radio_set_irq_mode(IRQ_MODE_TX);
      // NOTE: TX config is set in gloria_run_flood()
      radio_set_payload((uint8_t*) &current_flood->ack_message, GLORIA_ACK_LENGTH);
      radio_set_tx_callback(&gloria_radio_tx_ack_callback);
      radio_transmit_scheduled(0, 0, current_flood->current_tx_marker);
      break;

    case GLORIA_RADIO_RX:
      if (radio_modulations[current_flood->modulation].modem == MODEM_LORA) {
        // preamble detected events are used for the gloria_interface function 'gloria_get_rx_started_cnt'
        radio_set_irq_mode(IRQ_MODE_RX_CRC_PREAMBLE);
      } else {
        // many false positive preamble detected events for FSK => we don't use it and therefore disable them
        radio_set_irq_mode(IRQ_MODE_RX_CRC);
      }
      // NOTE: RX config is set in gloria_run_flood()
      radio_set_rx_callback(&gloria_radio_rx_callback);
      radio_set_timeout_callback(&gloria_radio_rx_timeout_callback);

      if (current_flood->msg_received || current_flood->lp_listening) {
        // if a msg has been received listen for predefined timeout
        uint32_t radio_rx_timeout = gloria_calculate_rx_timeout(current_flood);
        radio_receive_scheduled(current_flood->current_tx_marker - gloria_get_rx_ex_offset(current_flood), radio_rx_timeout);
      }
      else if (current_flood->marker) {
        // if the flood marker is not 0 start listening at the expected flood time
        uint64_t schedule_ts = current_flood->current_tx_marker - gloria_get_rx_ex_offset(current_flood);
        rx_timeout_ts = schedule_ts + current_flood->rx_timeout;
        radio_receive_scheduled(schedule_ts, (uint64_t) current_flood->rx_timeout);
      }
      else {
        // start receiving immediately
        rx_timeout_ts = hs_timer_get_current_timestamp() + current_flood->rx_timeout;
        radio_receive((uint64_t) current_flood->rx_timeout);
      }
      break;

    default:
      LOG_WARNING("invalid state");
      break;
  }
}

static void gloria_radio_tx_callback(void)
{
  current_flood->rem_retransmissions--;
  callback();
}


static void gloria_radio_tx_ack_callback(void)
{
  callback();
}

static void gloria_radio_rx_timeout_callback(bool crc_error)
{
  if (crc_error) {
    current_flood->crc_timeout = true;
    gloria_radio_continue_rx();
  }
  else {
    rx_callback(NULL, 0);
  }
}

static void gloria_radio_rx_callback(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error)
{
  // we expect at least GLORIA_HEADER_LENGTH_MIN bytes
  if (size < GLORIA_HEADER_LENGTH_MIN) {
    gloria_radio_continue_rx();
    return;
  }
  if (crc_error) {
    current_flood->crc_error = true;
    gloria_radio_continue_rx();
    return;
  }
  // update header size
  gloria_header_t* header    = (gloria_header_t*)payload;
  current_flood->header_size = (current_flood->ack_mode ? GLORIA_HEADER_LENGTH : GLORIA_HEADER_LENGTH_MIN) + header->sync * GLORIA_TIMESTAMP_LENGTH;

  // check packet type and apply user-defined packet filter
  if ((header->protocol_id != PROTOCOL_ID_GLORIA) ||
      (current_flood->filter_cb && (size > current_flood->header_size) && !current_flood->filter_cb(payload, current_flood->header_size, payload + current_flood->header_size, size - current_flood->header_size))) {
    gloria_radio_continue_rx();
  }
  else {
    current_flood->rssi = rssi;
    current_flood->snr  = snr;
    rx_callback(payload, size);
  }
}

/*
 * checks if the radio should continue receiving or the callback should be triggered
 * continues with rx if no rx timeout was specified or the timeout has not yet expired
 * always triggers the callback if a message has already been received (node is synchronized to the flood)
 */
static void gloria_radio_continue_rx(void)
{
  if (!current_flood->msg_received && !current_flood->lp_listening) {
    if (current_flood->rx_timeout) {
      uint64_t now = hs_timer_get_current_timestamp();

      if (rx_timeout_ts > (now + GLORIA_MIN_RX_TIME)) {
        radio_set_rx_callback(&gloria_radio_rx_callback);
        radio_set_timeout_callback(&gloria_radio_rx_timeout_callback);
        radio_receive(rx_timeout_ts - now);
      }
      else {
        rx_callback(NULL, 0);
      }
    }
    else {
      radio_set_rx_callback(&gloria_radio_rx_callback);
      radio_set_timeout_callback(&gloria_radio_rx_timeout_callback);
      radio_receive(0);
    }
  }
  else {
    rx_callback(NULL, 0);
  }
}

#endif /* GLORIA_ENABLE */
