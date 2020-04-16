/*
 * gloria_helpers.c
 *
 *  Created on: 03.08.2018
 *      Author: marku
 */

#include "flora_lib.h"


#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif /* MIN */
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif /* MAX */

protocol_config_t protocol_config;


/*
 * calculate the last active slot (rx or tx) for this flood based on the first rx index
 * returns 0 if the node is not the initiator and no msg has been received and lp_listening is false
 * returns the max number of slots for low power listening
 */
inline uint8_t gloria_calculate_last_active_slot(gloria_flood_t* flood) {
  if (flood->msg_received || flood->initial) {
    uint8_t las;
    switch (flood->ack_mode) {
      case 0:
        las = flood->first_rx_index + flood->max_retransmissions;
        return MIN(las, flood->data_slots - 1);
        break;
      case 1:
        las = flood->first_rx_index + 2 * flood->max_retransmissions;
        return MIN(las, flood->data_slots * 2 - 2);
        break;
      case 2:
        return flood->data_slots * 2 - 1;
        break;
      default:
        return 0;
        break;
    }
  }
  else if (flood->lp_listening) {
    if (flood->ack_mode) {
      return flood->data_slots * 2 - 1;
    }
    else {
      return flood->data_slots - 1;
    }
  }
  else {
    return 0;
  }
}


/*
 * not finished if:
 *     slot_index <= last_active_slot  &&
 *     (!ack_mode || ack_counter < MAX_ACKS)
 */
inline bool gloria_is_not_finished(gloria_flood_t* flood) {
  if (flood->slot_index <= flood->last_active_slot
      && (!flood->ack_mode || flood->ack_counter < flood->max_acks)) {
    return true;
  }
  else {
    return false;
  }
}


/*
 * valid to send if:
 *     msg_received          &&
 *     remaining retransmissions > 0  &&
 *     flood was not acked
 */
inline bool gloria_valid_to_send(gloria_flood_t* flood) {
  if (
      flood->msg_received
          && flood->remaining_retransmissions
      && !flood->acked
  ) {
    return true;
  }
  else {
    return false;
  }
}


inline bool gloria_is_ack_slot(gloria_flood_t* flood) {
  return (flood->ack_mode && flood->slot_index % 2);
}


inline uint16_t gloria_get_id() {
  return protocol_config.uid;
}


inline protocol_role_t gloria_get_role() {
  return protocol_config.role;
}
