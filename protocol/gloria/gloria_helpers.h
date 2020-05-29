/*
 * gloria_helpers.h
 *
 *  Created on: 03.08.2018
 *      Author: marku
 */

#ifndef PROTOCOL_GLORIA_GLORIA_HELPERS_H_
#define PROTOCOL_GLORIA_GLORIA_HELPERS_H_


void gloria_load_id_and_role();
uint8_t gloria_calculate_last_active_slot(gloria_flood_t* flood);
bool gloria_is_not_finished(gloria_flood_t* flood);
bool gloria_valid_to_send(gloria_flood_t* flood);
bool gloria_is_ack_slot(gloria_flood_t* flood);
uint16_t gloria_get_id();
protocol_role_t gloria_get_role();

#endif /* PROTOCOL_GLORIA_GLORIA_HELPERS_H_ */
