/*
 * protocol.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_


#ifndef IS_HOST
#define IS_HOST         (HOST_ID == NODE_ID)
#endif /* HOST_ID */


typedef enum {
  RELAY = 0,
  BASE = 1,
  SLWB_LR,
} protocol_role_t;

typedef struct {
  uint16_t uid;
  protocol_role_t role;
} protocol_config_t;


#endif /* PROTOCOL_PROTOCOL_H_ */
