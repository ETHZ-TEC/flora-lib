/*
 * dozer_radio.h
 *
 *  Created on: Jul 23, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_DOZER_RADIO_H_
#define PROTOCOL_DOZER_DOZER_RADIO_H_


// rx and tx config functions
void tx_config();
void rx_config();

// send and receive functions
uint8_t dozer_send(dozer_send_message_t* msg, uint8_t size, uint32_t timeout);
uint8_t dozer_receive(uint32_t rx_timeout, int64_t rx_delay);

// radio callbacks
void dozer_tx_callback();
void dozer_rx_callback(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr, bool crc_error);
void dozer_timeout_callback(bool crc_error);

// acknowledgment related functions
uint8_t send_ack();
void set_ack_byte(uint8_t ab);
void set_await_ack(bool aw_ack);

// channel activity detection
void get_rssi(uint32_t max_sense_time);

// debug help functions
bool in_rec_fct();
void rx_to_fail();
uint16_t get_to_fails();
uint16_t get_to_count();

// shut down function
void dozer_radio_shutdown();

// allocate memory for received message buffer
void dozer_alloc_msg();


#endif /* PROTOCOL_DOZER_DOZER_RADIO_H_ */
