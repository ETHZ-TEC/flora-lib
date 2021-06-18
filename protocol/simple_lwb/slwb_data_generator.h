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

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_DATA_GENERATOR_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_DATA_GENERATOR_H_


void slwb_data_generator_init(uint8_t period);

slwb_stream_t* slwb_data_generator_get_data_stream();

void slwb_data_generator_stream_acked(uint8_t node_id, uint8_t stream_id);

uint8_t slwb_data_generator_unallocated_streams();

void slwb_data_generator_new_stream(uint8_t id, uint8_t period);
void slwb_data_generator_add_stream(slwb_stream_request_t* sr);

void slwb_data_generator_generate_data();
void slwb_data_generator_add_data(slwb_data_message_t* msg);
slwb_data_message_t* slwb_data_generator_get_data();
void slwb_data_generator_msg_sent();

void slwb_data_generator_print_msgs();

void slwb_data_generator_set_backoff(uint8_t node_id, uint8_t stream_id, uint8_t backoff);
void slwb_data_generator_reduce_backoffs();

void slwb_data_generatior_print_stats();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_DATA_GENERATOR_H_ */
