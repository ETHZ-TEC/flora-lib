/*
 * Copyright (c) 2018 - 2022, ETH Zurich, Computer Engineering Group (TEC)
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

#ifndef PROTOCOL_GLORIA_GLORIA_TIME_H_
#define PROTOCOL_GLORIA_GLORIA_TIME_H_

uint32_t gloria_calculate_slot_time(uint8_t modulation, uint8_t ack_mode, uint8_t index, uint8_t msg_size);
uint32_t gloria_calculate_flood_time(uint8_t payload_len, uint8_t modulation, uint8_t data_slots, uint8_t sync, uint8_t ack_mode);
uint64_t gloria_calculate_tx_marker(gloria_flood_t* flood);
uint64_t gloria_calculate_rx_marker(gloria_flood_t* flood);
int32_t  gloria_get_rx_ex_offset(gloria_flood_t* flood);
uint16_t gloria_calculate_rx_timeout(gloria_flood_t* flood);
void     gloria_reconstruct_flood_marker(gloria_flood_t* flood);
uint64_t gloria_get_message_timestamp(gloria_flood_t* flood);
void     gloria_sync_timer(gloria_flood_t* flood);

#endif /* PROTOCOL_GLORIA_GLORIA_TIME_H_ */
