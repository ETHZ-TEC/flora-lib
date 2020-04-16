/*
 * slwb_timer_sync.c
 *
 *  Created on: Feb 4, 2019
 *      Author: kelmicha
 */

#include "flora_lib.h"

static slwb_timer_sync_markers_t timer_markers[NUMBER_SAVED_MARKERS] = {0};
static uint8_t latest_markers = NUMBER_SAVED_MARKERS-1;
uint8_t n_markers = 0;
static uint64_t slwb_latest_sync = 0;

/*
 * get and store the time diff from the given flood
 * flood: needs to be a successful sync flood
 */
void slwb_save_markers(gloria_flood_t* flood) {
  latest_markers = (latest_markers + 1) % NUMBER_SAVED_MARKERS;

  slwb_timer_sync_markers_t* markers = &timer_markers[latest_markers];

  markers->global_marker = flood->received_marker;
  markers->local_marker = slwb_reconstruct_marker(flood->reconstructed_marker);
  markers->valid = true;

  if (n_markers < NUMBER_SAVED_MARKERS) {
    n_markers++;
  }
}


/*
 * calculate timer offset and drift and adjust timer accordingly
 */
void slwb_adapt_timer() {
  double_t drift = 1;
  double_t offset = 0;
  if (n_markers >= SLWB_MIN_MARKERS_FOR_DRIFT_COMP) {
    uint64_t min_x;
    uint64_t min_y;
    double_t mean_x = 0;
    double_t mean_y = 0;
    double_t mean_xy = 0;
    double_t mean_x2 = 0;

    if (n_markers == NUMBER_SAVED_MARKERS) {
      min_x = timer_markers[(latest_markers + 1) % NUMBER_SAVED_MARKERS].local_marker;
      min_y = timer_markers[(latest_markers + 1) % NUMBER_SAVED_MARKERS].global_marker;
    }
    else {
      min_x = timer_markers[0].local_marker;
      min_y = timer_markers[0].global_marker;
    }

    for (int i = 0; i < n_markers; ++i) {
      double_t x = (double_t) (timer_markers[i].local_marker - min_x);
      double_t y = (double_t) (timer_markers[i].global_marker - min_y);

      mean_x += x / n_markers;
      mean_y += y / n_markers;
      mean_xy += x*y / n_markers;
      mean_x2 += x*x / n_markers;
    }

    drift = ((mean_x * mean_y) - mean_xy) / (mean_x*mean_x - mean_x2);
    offset = mean_y - drift*mean_x;
    offset = offset - drift*min_x + min_y;
  }
  else if (n_markers > 0) {
    drift = 1;
    offset = (double_t) timer_markers[latest_markers].global_marker - timer_markers[latest_markers].local_marker;
  }
  else {
    return;
  }

  if (drift < 0.5 || drift > 1.5) {
    drift = 1;
    offset = (double_t) timer_markers[latest_markers].global_marker - timer_markers[latest_markers].local_marker;
    print(2, "markers:");
    for (int i = 0; i < n_markers; ++i) {
      sprintf(char_buff, "%llu, %llu", timer_markers[i].local_marker, timer_markers[i].global_marker);
      print(2, char_buff);
    }
  }

  hs_timer_set_drift(drift);
  hs_timer_set_offset(offset);
  slwb_latest_sync = hs_timer_get_current_timestamp();

  sprintf(char_buff, "drift: %f, offset: %f", drift, offset);
  print(2, char_buff);
}

/*
 * reconstruct counter value from virtual timestamp
 */
inline uint64_t slwb_reconstruct_marker(uint64_t marker) {
  return (uint64_t) round(((double_t) marker - hs_timer_get_offset()) / hs_timer_get_drift());
}

uint64_t slwb_get_latest_sync() {
  return slwb_latest_sync;
}

/*
 * reset variables to start synchronization again
 */
void slwb_reset_timer() {
  n_markers = 0;
  latest_markers = NUMBER_SAVED_MARKERS-1;
  slwb_latest_sync = 0;
}

/*
 * returns true if enough markers have been saved to calculate the drift and compensate it
 */
bool slwb_timer_drift_comp_active() {
  return n_markers >= SLWB_MIN_MARKERS_FOR_DRIFT_COMP;
}
