/*
 * gloria_time.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 *
 *      comments added by kelmicha
 */

#ifndef PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_
#define PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_


typedef struct __attribute__((__packed__)) {
  uint8_t dst;
} gloria_ack_message_t;

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t type: 7;        // msg type
  uint8_t sync: 1;        // 1: message includes ts for sync, 0: no ts
  uint8_t slot_index;        // current slot index
  uint8_t src;          // msg source
  uint8_t dst;          // msg destination
} gloria_header_t;

typedef struct {
  // parameters to specify before flood start
  void (*callback);            // callback function after flood finished
  gloria_header_t header;    // header of the gloria message
  void* payload;             // pointer to the payload to send
  uint64_t marker;            // set flood start timestamp; has to be a multiple of GLORIA_SCHEDULE_GRANULARITY for the initiator;
                      // can be set to determine start of rx;
  uint32_t rx_timeout;          // specific time (in hs_timer_ticks) to listen before returning if no flood was received
  uint8_t band;              // radio_band_t selection as defined in radio_constants.h [0-51]
  uint8_t modulation;            // radio_config_t selection as defined in radio_constants.h [0-9]
  int8_t power;              // power for this flood
  uint8_t payload_size;            // size of the raw payload (without header or timestamp)
                      // max payload = 255 - GLORIA_HEADER_LENGTH (- GLORIA_TIMESTAMP_LENGTH (if sync flood))
  uint8_t data_slots;            // max number of data slots for this flood; for ack floods this is also the number of ack slots
  uint8_t max_retransmissions;      // max number of retransmissions for this flood
  uint8_t max_acks;            // max number of acks to send
  uint8_t ack_mode;            // 2: initiator waits for ack, 1: send ack to save energy, 0: no ack requested
  bool initial;              // 1: initiator of the flood
  bool sync_timer;            // specify if gloria should adapt the timer offset to sync to the initiator clock
  bool lp_listening;            // don't listen continuously but only for a short time each slot until a message is received
  uint32_t guard_time;          // guard time in hs timer ticks is the time that the nodes starts to listen earlier and ends later

  // parameters used during flood run
  bool msg_received;            // true: successfully received a message; false: no message received; set to 1 at flood start for the initiator
  bool acked;                // true: flood ack msg received

  uint8_t remaining_retransmissions;  // remaining retransmissions for this node
  uint8_t ack_counter;          // number of acks sent

  int8_t first_rx_index;          // used to save slot number of first receive
  uint8_t last_active_slot;        // last slot the node should send / receive (different for nodes)

  uint64_t received_marker;        // flood marker that was received during the flood; equal to marker for the initiator
  uint64_t reconstructed_marker;      // reconstructed marker from a message receive; equal to marker for the initiator

  uint64_t current_tx_marker;        // save ts of current transmission

  uint8_t message_size;          // actual size of the transmitted message /(header + payload + (timestamp for sync floods))
  gloria_ack_message_t ack_message;    // contains the ack_message to send / that was received

  // parameters that contain additional information
  bool crc_error;              // true: crc_error occurred during flood (msg was received but the crc failed)
  bool crc_timeout;            // true: crc_timeout occurred during flood (a header_error interrupt occurred)
  int8_t snr;                // save snr of received data msg
  int8_t rssi;              // save rssi of received data msg
  uint16_t flood_idx;            // allows to assign the flood an id

  bool radio_no_sleep;      // true: don't put radio into sleep mode even if tx marker is far in the future

  uint16_t node_id;

} gloria_flood_t;

#endif /* PROTOCOL_GLORIA_GLORIA_STRUCTURES_H_ */
