/*
 * slwb_manager.h
 *
 *  Created on: Dec 6, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_MANAGER_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_MANAGER_H_


void slwb_start_round(slwb_round_t* round);
void slwb_finish_round();

void slwb_schedule_slot();
void slwb_contention_slot();
void slwb_ack_slot();
void slwb_data_slot();
void slwb_lr_schedule_slot();
void slwb_lr_contention_slot();
void slwb_lr_data_slot();

void slwb_schedule_slot_callback();
void slwb_contention_slot_callback();
void slwb_ack_slot_callback();
void slwb_data_slot_callback();
void slwb_lr_schedule_slot_callback();
void slwb_lr_contention_slot_callback();
void slwb_lr_data_slot_callback();

void slwb_set_flood_defaults(uint8_t modulation, int8_t power, uint8_t slots, uint8_t retransmissions, uint8_t acks, uint8_t ack_mode);
void slwb_set_flood_tx_defaults();
void slwb_set_flood_rx_defaults();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_MANAGER_H_ */
