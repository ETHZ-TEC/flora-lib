/*
 * gloria.c
 *
 *  Created on: 03.08.2018
 *      Author: marku
 */

#include "flora_lib.h"

#if GLORIA_ENABLE

extern volatile uint64_t gloria_last_sync;

gloria_flood_t* current_flood = NULL;

static void (*flood_callback)();

static void gloria_rx_callback(uint8_t* payload, uint8_t size);
static void gloria_process_rx(uint8_t* payload, uint8_t size);
static void gloria_tx_callback();

static void gloria_process_slot();
static void gloria_finish_slot();


void gloria_init() {
  gloria_load_id_and_role();
}

/*
 * initialize the necessary flood and message header parameters
 * start the flood
 */
void gloria_run_flood(gloria_flood_t* flood, void (*callback)()) {
  current_flood = flood;
  flood_callback = callback;

  // initialize flood parameters
  current_flood->first_rx_index = ((current_flood->ack_mode? -2 : -1));
  current_flood->slot_index = 0;
  current_flood->msg_received = current_flood->initial;
  current_flood->last_active_slot = gloria_calculate_last_active_slot(current_flood);

  current_flood->remaining_retransmissions = current_flood->max_retransmissions;

  current_flood->ack_message.dst = 0;
  current_flood->ack_counter = 0;
  current_flood->acked = 0;

  // calculate the message size; always needed for the initiator; other nodes need it for low power listening
  current_flood->message_size = GLORIA_HEADER_LENGTH + current_flood->payload_size + (current_flood->message->header.sync? GLORIA_TIMESTAMP_LENGTH:0);

  // calculate the message size for the initiator, initialize markers
  if (current_flood->initial) {
    current_flood->received_marker = flood->marker;
    current_flood->reconstructed_marker = flood->marker;

    // add timestamp for sync floods
    if (current_flood->message->header.sync) {
      uint64_t new_timestamp = current_flood->received_marker / GLORIA_SCHEDULE_GRANULARITY;
      memcpy(current_flood->message->payload + current_flood->payload_size, (uint8_t*) &new_timestamp, GLORIA_TIMESTAMP_LENGTH);
    }
  }
  else {
    current_flood->received_marker = 0;
    current_flood->reconstructed_marker = 0;

    // check if lp listening possible with the given guard time
    if (current_flood->lp_listening) {
      uint32_t slot_time;
      uint32_t rx2rx_trans = gloria_timings[flood->modulation].slotAckOverhead;
      if (current_flood->ack_mode) {
        slot_time = gloria_calculate_slot_time(current_flood, 1, GLORIA_ACK_LENGTH);
      }
      else {
        slot_time = gloria_calculate_slot_time(current_flood, 0, current_flood->message_size);
      }

      if (slot_time < rx2rx_trans + gloria_calculate_rx_timeout(current_flood)) {
        // deactivate lp listening and listen for the whole flood time
        current_flood->lp_listening = false;
        current_flood->rx_timeout = gloria_calculate_flood_time(current_flood) + 2*current_flood->guard_time;
      }
    }
    else if (current_flood->rx_timeout) {
      // increase rx_timeout by the guard time
      current_flood->rx_timeout += 2 * current_flood->guard_time;
    }
  }

  // initialize message header
  current_flood->message->header.src = (current_flood->initial? gloria_get_id() : 0);
  current_flood->message->header.slot_index = 0;

  // initialize error flags
  current_flood->crc_error = false;
  current_flood->crc_timeout = false;

  gloria_process_slot();
}

void gloria_update() {
  //gloria_print_flood_periodic();
}

void gloria_process_slot() {
  if (gloria_is_not_finished(current_flood)) {
    if (gloria_is_ack_slot(current_flood)) {
      if (current_flood->acked) {
        // flood ack received or node is flood destination -> send ack
        current_flood->ack_counter++;
        gloria_tx_ack(current_flood, &gloria_tx_callback);
      }
      else {
        // listen for ack
        gloria_rx(current_flood, &gloria_rx_callback);
      }
    }
    else if (gloria_valid_to_send(current_flood)) {
      current_flood->message->header.slot_index = current_flood->slot_index;
      gloria_tx(current_flood, &gloria_tx_callback);
    }
    else if (!current_flood->msg_received) {
      gloria_rx(current_flood, &gloria_rx_callback);
    }
    else {
      gloria_finish_slot();
    }
  }
  else {
    // finish flood
    flood_callback();
  }
}

static void gloria_rx_callback(uint8_t* payload, uint8_t size) {
  if (payload == NULL || size == 0) {
    gloria_finish_slot();
  }
  else if (size == GLORIA_ACK_LENGTH) {
    gloria_ack_message_t* ack_message = (gloria_ack_message_t*) payload;
    if (ack_message->dst) {
      current_flood->acked = true;
      current_flood->ack_message.dst = ack_message->dst;

      // prevent ack destination from retransmitting the ack message (ack_mode 2)
      // prevent nodes that have not yet sent the message from retransmitting acks (ack mode 1)
      if (!current_flood->msg_received || ((current_flood->ack_mode == 2) && ack_message->dst == gloria_get_id()) ||
          ((current_flood->ack_mode == 1) && current_flood->slot_index - current_flood->first_rx_index == 1)) {
        // increase slot index to be consistent with other flood ends
        current_flood->slot_index++;
        // finish flood
        flood_callback();
        return;
      }
    }
    gloria_finish_slot();
  }
  else if (size >= GLORIA_HEADER_LENGTH) {
    gloria_process_rx(payload, size);
  }
  else {
    gloria_finish_slot();
  }
}

static void gloria_process_rx(uint8_t* payload, uint8_t size) {
  gloria_message_t* message = (gloria_message_t*) payload;

    if (!current_flood->msg_received) {
      // get message
      current_flood->message_size = size;
      current_flood->msg_received = true;
      memcpy(current_flood->message, message, size);

      // the size of the actual payload is the message size minus the header length and for sync floods minus the timestamp length
      current_flood->payload_size = current_flood->message_size - GLORIA_HEADER_LENGTH - (message->header.sync? GLORIA_TIMESTAMP_LENGTH:0);

      current_flood->slot_index = message->header.slot_index;
      gloria_reconstruct_flood_marker(current_flood);

      // make sure the timer is only synced if the flood contains a timestamp
      current_flood->sync_timer = current_flood->sync_timer && message->header.sync;

      // sync timer
      if (current_flood->sync_timer) {
        gloria_sync_timer(current_flood);
      }

      current_flood->first_rx_index = current_flood->slot_index;
      current_flood->last_active_slot = gloria_calculate_last_active_slot(current_flood);

      // set guard time to 0 as node is now synced to this flood
      current_flood->guard_time = 0;

      // check if node is also the destination
      if (message->header.dst == gloria_get_id()) {
        if (current_flood->ack_mode) {
          // prepare ack if flood should be acked
          current_flood->acked = true;
          current_flood->ack_message.dst = message->header.src;
        }
        else {
          // finish flood
          current_flood->slot_index++;
          flood_callback();
          return;
        }
      }
    }

  gloria_finish_slot();
}

static void gloria_tx_callback() {
  gloria_finish_slot();
}

void gloria_finish_slot() {
  current_flood->slot_index++;
  gloria_process_slot();
}

#endif /* GLORIA_ENABLE */
