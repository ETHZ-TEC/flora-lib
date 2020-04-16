/*
 * helpers.h
 *
 *  Created on: Oct 5, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_LWB_TEST_HELPERS_H_
#define PROTOCOL_LWB_TEST_HELPERS_H_

#include <stdarg.h>
#include <stdint.h>

//#include "cli/cli_print.h"
#include "protocol/protocol.h"
#include "led/led.h"
#include "time/hs_timer.h"

#include "time/hs_timer.h"
#include "led/led.h"
#include "main.h"
#include "config/config.h"
#include "radio/semtech/sx126x/sx126x.h"
#include "radio/semtech/boards/sx126x-board.h"

#include "protocol/simple_lwb/slwb_structures.h"
#include "protocol/simple_lwb/slwb.h"
#include "flocklab/flocklab.h"


extern char char_buff[100];

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

#endif /* PROTOCOL_LWB_TEST_HELPERS_H_ */
