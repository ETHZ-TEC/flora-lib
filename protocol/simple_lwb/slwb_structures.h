/*
 * slwb_structures.h
 *
 *  Created on: Oct 11, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_STRUCTURES_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_STRUCTURES_H_


enum slwb_messages {
  SLWB_ROUND_SCHEDULE,    // round schedule for a normal round without any long range information
  SLWB_LR_ROUND_SCHEDULE,    // round schedule for long range round (contains normal and long range information)
  SLWB_LR_SCHEDULE,      // long range schedule information sent to the long range nodes
  SLWB_STREAM_REQUEST,
  SLWB_STREAM_ACK,
  SLWB_DATA_MESSAGE,      // simple data message
  SLWB_DATA_PLUS_SR      // data message with appended stream request
};

typedef enum {
  NORMAL,
  LONG_RANGE
} round_type_t;

typedef enum {
  RELAY = 0,
  BASE = 1,
  SLWB_LR,
} slwb_role_t;

typedef struct {
  uint16_t uid;
  slwb_role_t role;
} slwb_config_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t round_period;              // round period to calculate next round start
  bool contention_slot: 1;            // current round has a contention slot
  uint8_t n_acks: 7;                // number of stream acks
  uint8_t n_data_slots;              // number of data slots in this round
} slwb_gen_schedule_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t lr_base;                // ID of the bridge node that handles the lr communication
  uint8_t lr_period;                // round period to calculate next round start for remote nodes
  bool lr_ack :1;                  // schedule contains a stream ack for a remote node or not
  bool lr_cont :1;                // round has a lr contantion slot allocated
  uint8_t n_lr_data :6;              // number of lr data slots in this round
} slwb_lr_schedule_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t node_id;
  uint8_t stream_id;
} slwb_stream_ack_t;

typedef struct __attribute__((packed, aligned(1))) {
  slwb_lr_schedule_t lr_schedule;
  slwb_gen_schedule_t gen_schedule;
  uint8_t slots[SLWB_MAX_DATA_SLOTS];        // array of node ID's, that defines which nodes should send (or receive for lr) when
  slwb_stream_ack_t stream_acks[SLWB_MAX_STREAM_ACKS];  // array of stream acks
} slwb_round_schedule_t;




typedef struct __attribute__((packed, aligned(1))) {
  round_type_t type;  // long range or normal
  slwb_round_schedule_t* round_schedule;
  uint64_t round_start;  // start timestamp for the round

  uint8_t modulation;  // modulation for normal floods
  uint8_t power_lvl;  // power lvl for

  uint8_t lr_mod;    // modulation for long range communication
  uint8_t lr_pwr;    // power for long range communication
} slwb_round_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t node_id;
  uint8_t stream_id;
  uint8_t size;  // in bytes
  uint8_t period; // in s
} slwb_stream_request_t;

typedef struct __attribute__((packed, aligned(1))) {
  uint8_t node_id;
  uint16_t packet_id;
  uint8_t payload[SLWB_MAX_DATA_PAYLOAD];
} slwb_data_message_t;

typedef struct {
  bool acked;
  int8_t backoff;
  slwb_stream_request_t stream_req;
} slwb_stream_t;


#endif /* PROTOCOL_SIMPLE_LWB_SLWB_STRUCTURES_H_ */
