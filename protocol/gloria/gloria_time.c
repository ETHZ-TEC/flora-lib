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

volatile uint64_t gloria_last_sync = (uint64_t) -1;


/*
 * calculate the tx marker based on the marker, slot_idx, modulation and message size
 */
uint64_t gloria_calculate_tx_marker(gloria_flood_t* flood)
{
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

  offset += timings->floodInitOverhead - GLORIA_HSTIMER_TRIGGER_DELAY;

  uint32_t slot_time_data = gloria_calculate_slot_time(flood->modulation, flood->ack_mode, 0, flood->payload_size + flood->header_size + (flood->header.sync ? GLORIA_TIMESTAMP_LENGTH : 0));

  if (flood->ack_mode) {
    uint32_t slot_time_ack = gloria_calculate_slot_time(flood->modulation, flood->ack_mode, 1, GLORIA_ACK_LENGTH);
    offset += (flood->header.slot_index + 1) / 2 * slot_time_data + (flood->header.slot_index) / 2 * slot_time_ack;
  }
  else {
    offset += flood->header.slot_index * slot_time_data;
  }

  flood->current_tx_marker = offset;

  return offset;
}


/*
 * calculate the rx marker based on the current tx marker
 */
inline uint64_t gloria_calculate_rx_marker(gloria_flood_t* flood)
{
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
uint32_t gloria_calculate_slot_time(uint8_t modulation, uint8_t ack_mode, uint8_t index, uint8_t msg_size)
{
  const gloria_timings_t* timings = &(gloria_timings[modulation]);
  uint32_t slot_time;
  if (ack_mode && (index % 2)) {
    slot_time = timings->slotAckOverhead;
  }
  else {
    slot_time = timings->slotOverhead;
  }

  slot_time += radio_get_toa_hs(msg_size, modulation);

  return slot_time;
}


/*
 * calculate the duration of a flood
 * flood: modulation, data_slots, payload, ack_mode and header->sync must be specified in the flood struct
 * returns flood duration in hs_timer ticks
 */
uint32_t gloria_calculate_flood_time(uint8_t payload_len, uint8_t modulation, uint8_t data_slots, uint8_t sync, uint8_t ack_mode)
{
  const gloria_timings_t* timings = &(gloria_timings[modulation]);

  uint8_t  msg_size       = payload_len + (ack_mode ? GLORIA_HEADER_LENGTH : GLORIA_HEADER_LENGTH_MIN) + (sync ? GLORIA_TIMESTAMP_LENGTH : 0);
  uint32_t slot_time_data = gloria_calculate_slot_time(modulation, ack_mode, 0, msg_size);
  uint32_t flood_time     = timings->floodInitOverhead + GLORIA_FLOOD_FINISH_OVERHEAD + data_slots * slot_time_data;

  if (ack_mode) {
    flood_time += data_slots * gloria_calculate_slot_time(modulation, ack_mode, 1, GLORIA_ACK_LENGTH);
  }

  return flood_time;
}


inline int32_t gloria_get_rx_ex_offset(gloria_flood_t* flood)
{
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  return timings->rxOffset + flood->guard_time + GLORIA_RX_TRIGGER_DELAY - GLORIA_TX_TRIGGER_DELAY;
}


/*
 * calculate rx timeout in hs timer ticks
 */
inline uint16_t gloria_calculate_rx_timeout(gloria_flood_t* flood)
{
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  return (uint64_t) (2*timings->rxOffset + radio_get_toa_hs(0, flood->modulation) + 2*flood->guard_time);
}


/*
 * reconstruct message timestamp
 */
inline uint64_t gloria_get_message_timestamp(gloria_flood_t* flood)
{
  uint64_t payload_timestamp = 0;
  memcpy(&payload_timestamp, flood->payload + flood->payload_size, GLORIA_TIMESTAMP_LENGTH);
  uint64_t message_timestamp = payload_timestamp * GLORIA_SCHEDULE_GRANULARITY;
  return message_timestamp;
}


/*
 * reconstruct flood marker from the message capture timestamp
 */
void gloria_reconstruct_flood_marker(gloria_flood_t* flood)
{
  const gloria_timings_t* timings = &(gloria_timings[flood->modulation]);

  uint32_t slot_time_sum;
  if (flood->ack_mode) {
    slot_time_sum = (flood->header.slot_index / 2 * ( gloria_calculate_slot_time(flood->modulation, flood->ack_mode, 0, flood->payload_size + flood->header_size + flood->header.sync * GLORIA_TIMESTAMP_LENGTH)
                      + gloria_calculate_slot_time(flood->modulation, flood->ack_mode, 1, GLORIA_ACK_LENGTH) ));
  } else {
    slot_time_sum = (flood->header.slot_index * gloria_calculate_slot_time(flood->modulation, flood->ack_mode, 0, flood->payload_size + flood->header_size + flood->header.sync * GLORIA_TIMESTAMP_LENGTH));
  }
  flood->reconstructed_marker = radio_get_last_sync_timestamp() -
                                timings->txSync -
                                slot_time_sum -
                                timings->floodInitOverhead;

  if (flood->header.sync) {
    flood->received_marker = gloria_get_message_timestamp(flood);
  }
}


/*
 * synchronize hs_timer to the received timestamp
 */
void gloria_sync_timer(gloria_flood_t* flood)
{
  gloria_last_sync = flood->received_marker;
  hs_timer_adapt_offset((double_t) flood->received_marker - flood->reconstructed_marker);
};


#endif /* GLORIA_ENABLE */
