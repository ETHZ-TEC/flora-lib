/*
 * slwb_constants.h
 *
 *  Created on: Dec 7, 2018
 *      Author: kelmicha
 */

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "protocol/gloria/gloria_structures.h"
#include "protocol/gloria/gloria_time.h"
#include "protocol/simple_lwb/slwb_structures.h"

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_CONSTANTS_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_CONSTANTS_H_

#define LR_SCHEDULE 1          // defines if lr nodes should be included or not
#define N_ROUNDS 1000          // number of rounds before the protocol is stopped

#define SLWB_SLOT_OVERHEAD 400000     // 50ms; overhead for each slot used for processing

#define SLWB_MAX_DATA_SLOTS 32      // max number of data slots that can be allocated in one schedule
#define SLWB_MAX_DATA_PAYLOAD 16    // payload size of the data messages
#define SLWB_MAX_STREAM_ACKS 4      // max number of acks appended to schedule

#define SLWB_DATA_QUEUE_SIZE 32      // max number of messages in the data queue
#define SLWB_BACKLOG_HIGHER_LIMIT 2    // if the data queue size is above this value a second stream gets requested
#define SLWB_BACKLOG_LOWER_LIMIT 2    // cancel the second stream if the queue size is again below this value

#define SLWB_BACKOFF 5          // SLWB_BACKOFF-1 is the max number of rounds that a node backs off for contention
#define SLWB_LR_BACKOFF 2        // SLWB_LR_BACKOFF-1 is the max number of rounds that a long range node backs off for contention
#define SLWB_LR_ROUND_MULT 3       // the period for long range nodes is SLWB_LR_ROUND_MULT times the round period
#define SLWB_MISSED_SCHEDULES 3      // node returns to bootstrap after SLWB_MISSED_SCHEDULES missed schedules in a row

#define NUMBER_SAVED_MARKERS 16        // max number of markers safed for timer synchronization
#define SLWB_MIN_MARKERS_FOR_DRIFT_COMP 5   // minimal number of sync markers needed before the clock drift is compensated
#define SLWB_UNSYNCED_DRIFT_PPM 30      // drift in ppm if the nodes has no clock drift compensation
#define SLWB_SYNCED_DRIFT_PPM 0.02      // drift in ppm with clock drift compensation


typedef struct {
  uint32_t schedule_slot_time;
  uint32_t contention_slot_time;
  uint32_t ack_slot_time;
  uint32_t data_slot_time;
  uint32_t lr_schedule_slot_time;
  uint32_t lr_cont_slot_time;
  uint32_t lr_data_slot_time;
} slwb_slot_times_t;

extern const slwb_slot_times_t slwb_slot_times[10];
extern const uint8_t slwb_max_flood_slots[];
extern const uint8_t slwb_default_power_levels[];

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_CONSTANTS_H_ */
