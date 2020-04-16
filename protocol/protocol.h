/*
 * protocol.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_


typedef enum {
  RELAY = 0,
  BASE = 1,
  SLWB_LR,
} protocol_role_t;

typedef struct {
  uint16_t uid;
  protocol_role_t role;
} protocol_config_t;


void protocol_init();
void protocol_update();
void protocol_run();

extern protocol_config_t protocol_config;

#endif /* PROTOCOL_PROTOCOL_H_ */
