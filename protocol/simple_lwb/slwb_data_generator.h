/*
 * slwb_data_generator.h
 *
 *  Created on: Dec 20, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_DATA_GENERATOR_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_DATA_GENERATOR_H_

#include "slwb.h"
#include "slwb_constants.h"

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
