/*
 * helpers.c
 *
 *  Created on: Oct 5, 2018
 *      Author: kelmicha
 */

#include "flora_lib.h"


protocol_config_t protocol_config;
bool lr_base = false;
bool lr_node = false;

/*
 * print the buffer if the priority is >= the PRINT_PRIO defined in helpers.h
 */
void print(uint8_t prio, char* buf) {
  if(prio >= PRINT_PRIO) {
    cli_println(buf);
  }
}


uint8_t array_find (uint8_t* array, uint8_t len, uint8_t n) {
  uint8_t* elem = array;

  for (int i = 0; i < len; ++i) {
    if (*elem == n) {
      return i;
    }
    else {
      elem++;
    }
  }

  return 0xFF;
}


uint64_t first_ts;
bool first = true;
uint64_t count = 0;

void blink_callback() {
#if FLOCKLAB
  FLOCKLAB_PIN_SET(FLOCKLAB_INT1);
  FLOCKLAB_PIN_CLR(FLOCKLAB_INT1);

  if (first) {
    first_ts = hs_timer_get_current_timestamp();
    first = false;
  }

  count++;
  hs_timer_schedule(first_ts + count*500*HS_TIMER_FREQUENCY_MS, &blink_callback);

#else /* FLOCKLAB */
  print_c_ts(1);
  slwb_round_finished();
#endif /* FLOCKLAB */
}


/*
 * serial print the current timestamp
 */
void print_c_ts(uint8_t prio) {
  uint64_t timestamp = hs_timer_get_current_timestamp();

  sprintf(char_buff, "Time:\t%llu", timestamp);
  print(prio, char_buff);
}


void slwb_print_schedule(uint8_t prio, slwb_round_t* round) {
  slwb_round_schedule_t* schedule = round->round_schedule;

  sprintf(char_buff, "round type: %d", round->type);
  print(prio, char_buff);
  print(prio, "schedule:");
  if (round->type == LONG_RANGE) {
    sprintf(char_buff, "\tlr_base: %d", schedule->lr_schedule.lr_base);
    print(prio, char_buff);
    sprintf(char_buff, "\tlr_period: %d", schedule->lr_schedule.lr_period);
    print(prio, char_buff);
    sprintf(char_buff, "\tlr_ack: %d", schedule->lr_schedule.lr_ack);
    print(prio, char_buff);
    sprintf(char_buff, "\tn_lr_data: %d", schedule->lr_schedule.n_lr_data);
    print(prio, char_buff);
  }

  sprintf(char_buff, "\tperiod: %d", schedule->gen_schedule.round_period);
  print(prio, char_buff);

  sprintf(char_buff, "\tcont: %d", schedule->gen_schedule.contention_slot);
  print(prio, char_buff);
  sprintf(char_buff, "\tn_data: %d", schedule->gen_schedule.n_data_slots);
  print(prio, char_buff);
  sprintf(char_buff, "\tn_acks: %d", schedule->gen_schedule.n_acks);
  print(prio, char_buff);

  uint8_t n_acks = schedule->lr_schedule.lr_ack + schedule->gen_schedule.n_acks;

  sprintf(char_buff, "\tstream acks %d:", n_acks);
  print(prio, char_buff);
  for (int i = 0; i < n_acks; ++i) {
    sprintf(char_buff, "\t\tnode: %d", schedule->stream_acks[i].node_id);
    print(prio, char_buff);
    sprintf(char_buff, "\t\tstream: %d", schedule->stream_acks[i].stream_id);
    print(prio, char_buff);
  }




  uint8_t slots = schedule->gen_schedule.n_data_slots + schedule->lr_schedule.n_lr_data;

  sprintf(char_buff, "\tslots %d:", slots);
  print(prio, char_buff);
  for (int i = 0; i < slots; ++i) {
    sprintf(char_buff, "\t\t%d", schedule->slots[i]);
    print(prio, char_buff);
  }
}

void slwb_print_stream(uint8_t prio, slwb_stream_request_t* sr) {
  sprintf(char_buff, "stream: %d, %d, %d", sr->node_id, sr->stream_id, sr->period);
  print(prio, char_buff);
}

void slwb_print_ack(slwb_stream_ack_t* ack, uint8_t dst) {
  print(1, "ack:");
  sprintf(char_buff, "\tnode: %d", dst);
  print(1, char_buff);
  sprintf(char_buff, "\tstream_id: %d", ack->stream_id);
  print(1, char_buff);
}

void slwb_print_data_msg(slwb_data_message_t* msg) {
  sprintf(char_buff, "data_msg: %d, %d", msg->node_id, msg->packet_id);
  print(10, char_buff);
}


/*
 * Protocol configurations
 */

inline uint16_t slwb_get_id() {
  return protocol_config.uid;
}

inline bool slwb_is_base() {
  return protocol_config.role == BASE;
}

void slwb_set_lr_base(bool lrb) {
  lr_base = lrb;
}

inline bool slwb_is_lr_base() {
  return lr_base;
}

void slwb_set_lr_node(bool lrn) {
  lr_node = lrn;
}

inline bool slwb_is_lr_node() {
  return lr_node;
}

inline bool slwb_is_lr_participant() {
  return slwb_is_lr_node() || slwb_is_lr_base();
}
