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
