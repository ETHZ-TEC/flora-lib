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

#ifndef PROTOCOL_SIMPLE_LWB_HELPERS_H_
#define PROTOCOL_SIMPLE_LWB_HELPERS_H_


extern char slwb_print_buffer[100];
extern slwb_config_t slwb_config;

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
