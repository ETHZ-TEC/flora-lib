/*
 * dozer_radio_admin.h
 *
 *  Created on: Jun 28, 2018
 *      Author: kelmicha
 */

#ifndef DOZER_RADIO_ADMIN_H_
#define DOZER_RADIO_ADMIN_H_

#include "dozer_topology.h"

enum {
  BROADCAST_ADDR = 0xFF,
};

enum {
  DOZER_TX = 1,
  DOZER_RX = 2,
};



// transmission or receive done and timeout functions
void dozer_tx_done(uint64_t tx_ts);
void dozer_rx_done(dozer_message_t* message, uint16_t size);
void dozer_rx_timeout();
void dozer_tx_timeout();


// send messages
uint8_t send_beacon(dozer_send_message_t* msg);
uint8_t send_activation_frame();
uint8_t send_connection_request(dozer_send_message_t * msg);
uint8_t send_handshake(dozer_send_message_t* msg);
uint8_t send_buffered_messages(uint8_t num, uint16_t id);


// receive messages
uint8_t listen_to_beacon(uint16_t address, uint32_t timeout, uint8_t activation);
void receive_messages_to_buffer(uint8_t num, uint16_t id, uint16_t * _lastSeqNr);


// overhearing functions
// overhear params have to be set before start_overhearing
void set_overhear_params(message_type_t oh_msg_type, uint32_t oh_time);
void start_overhearing(uint8_t continued) ;
void stop_overhearing();


// radio statistic functions
uint64_t compute_intervall();
void radio_stats_off();
void radio_stats_on();
void print_radio_stats();


// timer fired
void wait_timer_fired();
void data_timer_fired();
void connection_request_timer_fired();


// other
void radio_reset_state();
bool radio_is_idle();

void rssi_done(bool cannel_busy);
void handle_ack(uint8_t res, uint8_t ack_byte);

void assign_payload(dozer_message_t* msg, dozer_send_message_t* send_msg, uint16_t size);
uint8_t fill_buffer(uint8_t num_msgs_remaining);



#endif /* DOZER_RADIO_ADMIN_H_ */
