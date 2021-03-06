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

#ifndef PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_
#define PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_


typedef void (* gloria_flood_cb_t)(void);
typedef bool (* gloria_filter_cb_t)(const uint8_t*, uint8_t, const uint8_t*, uint8_t);
typedef bool (* gloria_rx_cb_t)(void);

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t protocol_id : 4;    // protocol ID
  uint8_t type : 3;           // msg type
  uint8_t sync: 1;            // 1: message includes ts for sync, 0: no ts
  uint8_t dst;
} gloria_ack_msg_t;

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t protocol_id : 4;    // protocol ID
  uint8_t type : 3;           // msg type
  uint8_t sync: 1;            // 1: message includes ts for sync, 0: no ts
  int8_t  slot_index;         // current slot index (must be signed due to init values for ACK floods)
  union {
    struct {                  // extended header for ACK mode
      uint8_t src;            // msg source
      uint8_t dst;            // msg destination
      uint8_t timestamp[GLORIA_TIMESTAMP_LENGTH];   // optional timestamp
    } ext;
    struct {                  // minimal header
      uint8_t timestamp[GLORIA_TIMESTAMP_LENGTH];   // optional timestamp
    } min;
  };
} gloria_header_t;

typedef enum {
  GLORIA_PKT_TYPE_DATA = 0,
  GLORIA_PKT_TYPE_ACK  = 1,
  GLORIA_PKT_TYPE_LAST = 7,   // 3 bits only, 7 is the last valid value
} gloria_pkt_type_t;

typedef struct {
  // parameters to specify before flood start
  gloria_flood_cb_t   flood_cb;           // callback function after flood finished
  gloria_rx_cb_t      rx_cb;              // user-defined RX callback function (NOTE: execution time must be short / in the microsecond range, otherwise the Gloria slotOverhead must be adjusted)
  gloria_filter_cb_t  filter_cb;          // user-defined RX packet filter

  gloria_header_t     header;             // header of the gloria message
  gloria_ack_msg_t    ack_message;        // contains the ack_message to send / that was received

  void*    payload;                       // pointer to the payload to send

  uint64_t marker;                        // set flood start timestamp; has to be a multiple of GLORIA_SCHEDULE_GRANULARITY for the initiator; can be set to determine start of rx
  uint64_t received_marker;               // flood marker that was received during the flood; equal to marker for the initiator
  uint64_t reconstructed_marker;          // reconstructed marker from a message receive; equal to marker for the initiator
  uint64_t current_tx_marker;             // save ts of current transmission

  uint32_t rx_timeout;                    // specific time (in hs_timer_ticks) to listen before returning if no flood was received
  uint32_t guard_time;                    // guard time in hs timer ticks is the time that the nodes starts to listen earlier and ends later

  uint16_t flood_idx;                     // allows to assign the flood an id
  uint16_t node_id;

  uint8_t  header_size;                   // size of the message header (GLORIA_HEADER_LENGTH or GLORIA_HEADER_LENGTH_MIN)
  uint8_t  payload_size;                  // size of the payload buffer (for TX: # bytes to send, for RX: max. number of bytes to receive)
  uint8_t  data_slots;                    // max number of data slots for this flood; for ack floods this is also the number of ack slots
  uint8_t  last_active_slot;              // last slot the node should send / receive (different for nodes)
  uint8_t  rem_retransmissions;           // remaining retransmissions for this node
  uint8_t  max_retransmissions;           // max number of retransmissions for this flood
  uint8_t  tx_delay_slots;                // TX delay after a reception, in number of slots (default: 0)
  uint8_t  max_acks;                      // max number of acks to send
  uint8_t  ack_mode;                      // 2: initiator waits for ack, 1: send ack to save energy, 0: no ack requested
  uint8_t  ack_counter;                   // number of acks sent
  uint8_t  band;                          // radio_band_t selection as defined in radio_constants.h [0-51]
  uint8_t  modulation;                    // radio_config_t selection as defined in radio_constants.h [0-9]
  int8_t   power;                         // power for this flood
  int8_t   first_rx_index;                // used to save slot number of first receive
  int8_t   snr;                           // save snr of received data msg
  int8_t   rssi;                          // save rssi of received data msg

  bool     initiator;                     // initiator of the flood
  bool     sync_timer;                    // specify if gloria should adapt the timer offset to sync to the initiator clock
  bool     lp_listening;                  // don't listen continuously but only for a short time each slot until a message is received
  bool     msg_received;                  // true: successfully received a message; false: no message received; set to 1 at flood start for the initiator
  bool     acked;                         // true: flood ack msg received
  bool     crc_error;                     // true: crc_error occurred during flood (msg was received but the crc failed)
  bool     crc_timeout;                   // true: crc_timeout occurred during flood (a header_error interrupt occurred)
  bool     radio_no_sleep;                // true: don't put radio into sleep mode even if tx marker is far in the future
  bool     stop;                          // if set to true, the flood will be stopped at the end of the current slot

} gloria_flood_t;


_Static_assert(sizeof(gloria_ack_msg_t) == GLORIA_ACK_LENGTH, "invalid size of gloria_ack_msg_t");
_Static_assert(sizeof(gloria_header_t) == (GLORIA_HEADER_LENGTH + GLORIA_TIMESTAMP_LENGTH), "invalid size of gloria_header_t");

#endif /* PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_ */
