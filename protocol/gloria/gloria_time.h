/*
 * gloria_time.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef PROTOCOL_GLORIA_GLORIA_TIME_H_
#define PROTOCOL_GLORIA_GLORIA_TIME_H_

#define GLORIA_SYNC_SEARCH_SPACE 3

uint32_t gloria_calculate_slot_time(gloria_flood_t* flood, uint8_t index, uint8_t msg_size);
uint32_t gloria_calculate_flood_time(gloria_flood_t* flood);

uint64_t gloria_calculate_tx_marker(gloria_flood_t* flood);
uint64_t gloria_calculate_rx_marker(gloria_flood_t* flood);

int32_t gloria_get_rx_ex_offset(gloria_flood_t* flood);
uint16_t gloria_calculate_rx_timeout(gloria_flood_t* flood);
uint32_t gloria_calculate_mcu_timeout(uint8_t modulation);
uint32_t gloria_calculate_tx_ex_offset(uint8_t modulation);

uint64_t gloria_get_capture_timestamp(uint8_t modulation);
void gloria_reconstruct_flood_marker(gloria_flood_t* flood);
void gloria_sync_timer(gloria_flood_t* flood);
uint64_t get_message_timestamp(gloria_flood_t* flood);

#endif /* PROTOCOL_GLORIA_GLORIA_TIME_H_ */
