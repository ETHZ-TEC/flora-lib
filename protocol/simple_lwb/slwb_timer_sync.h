/*
 * slwb_timer_sync.h
 *
 *  Created on: Feb 4, 2019
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_TIMER_SYNC_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_TIMER_SYNC_H_

#include "slwb.h"

typedef struct {
  bool valid;
  uint64_t global_marker;
  uint64_t local_marker;
} slwb_timer_sync_markers_t;

void slwb_save_markers(gloria_flood_t* flood);
void slwb_adapt_timer();

uint64_t slwb_get_latest_sync();
uint64_t slwb_reconstruct_marker(uint64_t marker);

void slwb_reset_timer();
bool slwb_timer_drift_comp_active();


#endif /* PROTOCOL_SIMPLE_LWB_SLWB_TIMER_SYNC_H_ */
