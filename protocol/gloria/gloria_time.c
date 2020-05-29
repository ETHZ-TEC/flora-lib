/*
 * gloria_time.c
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#include "flora_lib.h"

#if GLORIA_ENABLE

extern uint64_t radio_last_sync_timestamp;
extern bool hs_timer_recovered_by_rtc;
volatile uint64_t gloria_last_sync = (uint64_t) -1;


/*
 * calculate the tx marker based on the marker, slot_idx, modulation and message size
 */
inline uint64_t gloria_calculate_tx_marker(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  uint64_t offset = 0;

  if (flood->msg_received) {
    if (flood->sync_timer) {
      // take the received marker if the timer is synced to the initiator
      offset = flood->received_marker;
    }
    else {
      // else take the reconstructed marker
      offset = flood->reconstructed_marker;
    }
  }
  else {
    // if no message was received take the set marker
    offset = flood->marker;
  }

  offset += timings->floodInitOverhead;

  uint32_t slot_time_data = gloria_calculate_slot_time(flood, 0, flood->message_size);

  if (flood->ack_mode) {
    uint32_t slot_time_ack = gloria_calculate_slot_time(flood, 1, GLORIA_ACK_LENGTH);
    offset += (flood->slot_index + 1) / 2 * slot_time_data + (flood->slot_index) / 2 * slot_time_ack;
  }
  else {
    offset += flood->slot_index * slot_time_data;
  }

  flood->current_tx_marker = offset;

  return offset;
}


/*
 * calculate the rx marker based on the current tx marker
 */
inline uint64_t gloria_calculate_rx_marker(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);
  return gloria_calculate_tx_marker(flood) - timings->rxOffset - flood->guard_time;
}


/*
 * calculate slot time of current flood
 * flood: modulation and ack_mode need to be specified
 * index: 0 for data slot; 1 for ack slot
 * msg_size: size of the whole message
 * returns slot time in hs_timer ticks
 */
inline uint32_t gloria_calculate_slot_time(gloria_flood_t* flood, uint8_t index, uint8_t msg_size) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);
  uint32_t slot_time;
  if (flood->ack_mode && (index % 2)) {
    slot_time = timings->slotAckOverhead;
  }
  else {
    slot_time = timings->slotOverhead;
  }

  slot_time += radio_lookup_toa(flood->modulation, msg_size);

  return slot_time;
}


/*
 * calculate the duration of a flood
 * flood: modulation, data_slots, payload, ack_mode and header->sync must be specified in the flood struct
 * returns flood duration in hs_timer ticks
 */
inline uint32_t gloria_calculate_flood_time(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  uint32_t offset = timings->floodInitOverhead + GLORIA_FLOOD_FINISH_OVERHEAD;

  uint8_t slot_count = flood->data_slots;
  uint8_t msg_size = flood->payload_size + GLORIA_HEADER_LENGTH + (flood->message->header.sync? GLORIA_TIMESTAMP_LENGTH:0);
  uint32_t slot_time_data = gloria_calculate_slot_time(flood, 0, msg_size);

  if (flood->ack_mode) {
    uint32_t slot_time_ack = gloria_calculate_slot_time(flood, 1, GLORIA_ACK_LENGTH);
    offset += slot_count * slot_time_data + slot_count * slot_time_ack;
  }
  else {
    offset += slot_count * slot_time_data;
  }

  return offset;
}


inline int32_t gloria_get_rx_ex_offset(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  return timings->rxOffset + flood->guard_time + GLORIA_RX_TRIGGER_DELAY - GLORIA_TX_TRIGGER_DELAY;
}


/*
 * calculate rx timeout
 */
inline uint16_t gloria_calculate_rx_timeout(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  return (uint64_t) (2*timings->rxOffset + radio_lookup_toa(flood->modulation, 0) + 2*flood->guard_time) * RADIO_TIMER_FREQUENCY / HS_TIMER_FREQUENCY;
}


/*
 * reconstruct capture timestamp
 */
inline uint64_t gloria_get_capture_timestamp(uint8_t modulation) {
  const gloria_timings_t* timings = &(gloria_timings[modulation]);
  return radio_last_sync_timestamp - timings->txSync - GLORIA_BLACK_BOX_SYNC_DELAY;
}


/*
 * reconstruct flood marker from the message capture timestamp
 */
void gloria_reconstruct_flood_marker(gloria_flood_t* flood) {
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  if (flood->ack_mode) {
    flood->reconstructed_marker = gloria_get_capture_timestamp(flood->modulation)
        - (flood->slot_index / 2 * (gloria_calculate_slot_time(flood, 0, flood->message_size) + gloria_calculate_slot_time(flood, 1, GLORIA_ACK_LENGTH)) + timings->floodInitOverhead);
  }
  else {
    flood->reconstructed_marker = gloria_get_capture_timestamp(flood->modulation)
        - (flood->slot_index * gloria_calculate_slot_time(flood, 0, flood->message_size) + timings->floodInitOverhead);
  }

  if (flood->message->header.sync) {
    flood->received_marker = get_message_timestamp(flood);
  }
}


/*
 * synchronize hs_timer to the received timestamp
 */
void gloria_sync_timer(gloria_flood_t* flood) {
  gloria_last_sync = flood->received_marker;
  hs_timer_adapt_offset((double_t) flood->received_marker - flood->reconstructed_marker);
};


/*
 * reconstruct message timestamp
 */
uint64_t get_message_timestamp(gloria_flood_t* flood) {
  uint64_t payload_timestamp = 0;
  memcpy(&payload_timestamp, flood->message->payload + flood->payload_size, GLORIA_TIMESTAMP_LENGTH);
  uint64_t message_timestamp = payload_timestamp * GLORIA_SCHEDULE_GRANULARITY;
  return message_timestamp;
}

#endif /* GLORIA_ENABLE */
