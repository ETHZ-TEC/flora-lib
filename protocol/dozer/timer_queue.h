/*
 * timer_queue.h
 *
 *  Created on: Jul 17, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_TIMER_QUEUE_H_
#define PROTOCOL_DOZER_TIMER_QUEUE_H_

typedef enum timer_name {
  BEACON_SEND_TIMER,
  RECEIVE_BEACON_TIMER,
  CON_REQ_TIMER,

  WATCHDOG_TIMER,
  UPLOAD_TIMER,
  DOWNLOAD_TIMER,
  OVERHERA_TIMEOUT_TIMER,
  OVERHEAR_TIMER,
  SEND_CRM_WAIT_TIMER,
  RANDOM_LISTEN_TIMER,
  PARENTS_UPDATE_TIMER,
  WAIT_TIMER,
  HANDSHAKE_TIMER,
  DATA_TIMER,

  GEN_TIMER,
  RX_WATCHDOG,


} timer_name_t;

typedef struct timer_queue_element {
  timer_name_t timer_name;
  uint64_t timestamp;
  void (*callback);
  struct timer_queue_element_t* next;
} timer_queue_element_t;

uint8_t tq_add_element(timer_name_t timer_name, uint64_t ts, void (*callback));
uint8_t tq_remove_element(timer_name_t timer_name);
void tq_print_queue();

void tq_start_timer(timer_name_t timer_name, uint64_t ts, void (*callback));
void tq_stop_timer(timer_name_t timer_name);

void tq_run_timer(bool fired);
void tq_timer_fired();

bool tq_is_running(timer_name_t timer_name);
uint64_t tq_get_fire_ts(timer_name_t timer_name);



#endif /* PROTOCOL_DOZER_TIMER_QUEUE_H_ */
