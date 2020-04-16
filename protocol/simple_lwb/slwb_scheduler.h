/*
 * slwb_scheduler.h
 *
 *  Created on: Dec 7, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_SCHEDULER_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_SCHEDULER_H_


void slwb_initialize_scheduler();

void slwb_scheduler_get_schedule(slwb_round_schedule_t* schedule);

void slwb_scheduler_calculate_round_schedule(slwb_round_t* round);

void slwb_scheduler_add_stream_req(slwb_stream_request_t* sr, uint8_t sender, bool acked);

slwb_stream_t* slwb_scheduler_get_stream_to_ack();
slwb_stream_t* slwb_scheduler_get_lr_stream_to_ack();

uint8_t slwb_scheduler_number_data_slots(slwb_round_t* round, bool long_range);

void slwb_scheduler_update_streams();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_SCHEDULER_H_ */
