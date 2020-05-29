/*
 * helpers.h
 *
 *  Created on: Oct 5, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_HELPERS_H_
#define PROTOCOL_SIMPLE_LWB_HELPERS_H_


extern char slwb_print_buffer[100];
extern protocol_config_t slwb_protocol_config;

// define the print priority for the print function
#if FLOCKLAB
  #define PRINT_PRIO 9
#else /* FLOCKLAB */
  #define PRINT_PRIO 1
#endif /* FLOCKLAB */


void print(uint8_t prio, char* buf);

uint8_t array_find (uint8_t* array, uint8_t len, uint8_t n);

void blink_callback();

void print_c_ts(uint8_t prio);

void slwb_print_schedule(uint8_t prio, slwb_round_t* round);
void slwb_print_stream(uint8_t prio, slwb_stream_request_t* sr);
void slwb_print_ack(slwb_stream_ack_t* ack, uint8_t dst);
void slwb_print_data_msg(slwb_data_message_t* msg);


bool slwb_is_base();
uint16_t slwb_get_id();

void slwb_set_lr_base(bool lrb);
bool slwb_is_lr_base();
void slwb_set_lr_node(bool lrn);
bool slwb_is_lr_node();

bool slwb_is_lr_participant();

#endif /* PROTOCOL_SIMPLE_LWB_HELPERS_H_ */
