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
