/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
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

#ifndef PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_
#define PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_


typedef void (* gloria_flood_cb_t)(void);
typedef bool (* gloria_pkt_filter_cb_t)(uint8_t*, uint8_t);

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t protocol_id : 4;    // protocol ID
  uint8_t type : 3;           // msg type
  uint8_t sync: 1;            // 1: message includes ts for sync, 0: no ts
  uint8_t dst;
} gloria_ack_message_t;

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t protocol_id : 4;    // protocol ID
  uint8_t type : 3;           // msg type
  uint8_t sync: 1;            // 1: message includes ts for sync, 0: no ts
  int8_t  slot_index;         // current slot index (must be signed due to init values for ACK floods)
  uint8_t src;                // msg source
  uint8_t dst;                // msg destination
} gloria_header_t;

typedef struct {
  // parameters to specify before flood start
  gloria_flood_cb_t callback;             // callback function after flood finished
  gloria_header_t   header;               // header of the gloria message
  void*    payload;                       // pointer to the payload to send
  uint64_t marker;                        // set flood start timestamp; has to be a multiple of GLORIA_SCHEDULE_GRANULARITY for the initiator;
                                          // can be set to determine start of rx;
  uint32_t rx_timeout;                    // specific time (in hs_timer_ticks) to listen before returning if no flood was received
  uint8_t  band;                          // radio_band_t selection as defined in radio_constants.h [0-51]
  uint8_t  modulation;                    // radio_config_t selection as defined in radio_constants.h [0-9]
  int8_t   power;                         // power for this flood
  uint8_t  payload_size;                  // size of the raw payload (without header or timestamp)
                                          // max payload = 255 - GLORIA_HEADER_LENGTH (- GLORIA_TIMESTAMP_LENGTH (if sync flood))
  uint8_t  data_slots;                    // max number of data slots for this flood; for ack floods this is also the number of ack slots
  uint8_t  max_retransmissions;           // max number of retransmissions for this flood
  uint8_t  max_acks;                      // max number of acks to send
  uint8_t  ack_mode;                      // 2: initiator waits for ack, 1: send ack to save energy, 0: no ack requested
  bool     initial;                       // 1: initiator of the flood
  bool     sync_timer;                    // specify if gloria should adapt the timer offset to sync to the initiator clock
  bool     lp_listening;                  // don't listen continuously but only for a short time each slot until a message is received
  uint32_t guard_time;                    // guard time in hs timer ticks is the time that the nodes starts to listen earlier and ends later

  gloria_pkt_filter_cb_t pkt_filter;      // user-defined RX packet filter

  // parameters used during flood run
  bool     msg_received;                  // true: successfully received a message; false: no message received; set to 1 at flood start for the initiator
  bool     acked;                         // true: flood ack msg received

  uint8_t  remaining_retransmissions;     // remaining retransmissions for this node
  uint8_t  ack_counter;                   // number of acks sent

  int8_t   first_rx_index;                // used to save slot number of first receive
  uint8_t  last_active_slot;              // last slot the node should send / receive (different for nodes)

  uint64_t received_marker;               // flood marker that was received during the flood; equal to marker for the initiator
  uint64_t reconstructed_marker;          // reconstructed marker from a message receive; equal to marker for the initiator

  uint64_t current_tx_marker;             // save ts of current transmission

  uint8_t  header_size;                   // size of the message header (GLORIA_HEADER_LENGTH or GLORIA_HEADER_LENGTH_MIN)
  gloria_ack_message_t ack_message;       // contains the ack_message to send / that was received

  // parameters that contain additional information
  bool     crc_error;                     // true: crc_error occurred during flood (msg was received but the crc failed)
  bool     crc_timeout;                   // true: crc_timeout occurred during flood (a header_error interrupt occurred)
  int8_t   snr;                           // save snr of received data msg
  int8_t   rssi;                          // save rssi of received data msg
  uint16_t flood_idx;                     // allows to assign the flood an id

  bool     radio_no_sleep;                // true: don't put radio into sleep mode even if tx marker is far in the future
  bool     stop;                          // if set to true, the flood will be stopped at the end of the current slot

  uint16_t node_id;
  uint8_t  tx_delay_slots;                // TX delay after a reception, in number of slots (default: 0)

} gloria_flood_t;


_Static_assert(sizeof(gloria_ack_message_t) == GLORIA_ACK_LENGTH, "gloria_ack_message_t is not GLORIA_ACK_LENGTH bytes in size!");
_Static_assert(sizeof(gloria_header_t) == GLORIA_HEADER_LENGTH, "gloria_header_t is not GLORIA_HEADER_LENGTH bytes in size!");

#endif /* PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_ */
