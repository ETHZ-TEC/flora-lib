  /*
 * gloria_radio_helpers.h
 *
 *  Created on: 03.08.2018
 *      Author: marku
 */

#ifndef PROTOCOL_GLORIA_GLORIA_RADIO_H_
#define PROTOCOL_GLORIA_GLORIA_RADIO_H_

#include "gloria.h"

typedef enum {
  GLORIA_RADIO_TX,
  GLORIA_RADIO_ACK_TX,
  GLORIA_RADIO_RX,
} gloria_radio_state_t;

void gloria_tx(gloria_flood_t* flood, void (*tx_callback)());
void gloria_tx_ack(gloria_flood_t* flood, void (*tx_callback)());
void gloria_rx(gloria_flood_t* flood, void (*callback)(uint8_t*, uint8_t));

#endif /* PROTOCOL_GLORIA_GLORIA_RADIO_H_ */
