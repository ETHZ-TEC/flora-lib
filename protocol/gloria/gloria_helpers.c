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

#include "flora_lib.h"

#if GLORIA_ENABLE

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif /* MIN */
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif /* MAX */



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
        las = flood->first_rx_index + flood->tx_delay_slots + flood->max_retransmissions;
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
  if (flood->header.slot_index <= flood->last_active_slot
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
  return (flood->ack_mode && flood->header.slot_index % 2);
}


#endif /* GLORIA_ENABLE */
