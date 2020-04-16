/*
 * dozer_constants.h
 *
 *  Created on: Jun 29, 2018
 *      Author: user
 */

#ifndef PROTOCOL_DOZER_DOZER_CONSTANTS_H_
#define PROTOCOL_DOZER_DOZER_CONSTANTS_H_


// TODO: adapt timeouts and other time ralated constants to new platform
// TODO: check if no overflows occur

enum {
  JIFFIES_MULTIPLIER_MS = HS_TIMER_FREQUENCY_MS,
  JIFFIES_MULTIPLIER_US = HS_TIMER_FREQUENCY_US,

  INTERVAL_TIME_IN_JIFFIES = INTERVAL_TIME_IN_MS * JIFFIES_MULTIPLIER_MS,
  MAX_JITTER_IN_JIFFIES = MAX_JITTER_IN_MS * JIFFIES_MULTIPLIER_MS,

  SLOT_TIME_IN_MS = (uint32_t)INTERVAL_TIME_IN_MS/((uint32_t)MAX_CHILDREN+1+NUM_PROCESSING_SLOTS), // one slot for each potential child plus
                                                                                                   // one contention
                                                                                                   // and NUM_PROCESSING_SLOTS processing slot
  SLOT_TIME_IN_JIFFIES = SLOT_TIME_IN_MS * JIFFIES_MULTIPLIER_MS,

  CRM_BACKOFF_IN_MS = 20,
  CRM_BACKOFF_IN_JIFFIES = CRM_BACKOFF_IN_MS * JIFFIES_MULTIPLIER_MS,

  MAX_DRIFT_IN_PPM = 300,

  MIN_GUARD_TIME_FOR_BEACONS_IN_US = 300,
  MIN_GUARD_TIME_FOR_BEACONS_IN_JIFFIES = MIN_GUARD_TIME_FOR_BEACONS_IN_US * JIFFIES_MULTIPLIER_US,

  GRACE_PERIOD_IN_JIFFIES = ((uint32_t)INTERVAL_TIME_IN_JIFFIES*MAX_DRIFT_IN_PPM/1000000),

  BEACON_TRANSMISSION_TIME_IN_US = 9375,
  BEACON_TRANSMISSION_TIME_IN_JIFFIES = BEACON_TRANSMISSION_TIME_IN_US * JIFFIES_MULTIPLIER_US,

  DOWNLOAD_GUARD_TIME_IN_MS = 1,
  DOWNLOAD_GUARD_TIME_IN_JIFFIES = DOWNLOAD_GUARD_TIME_IN_MS * JIFFIES_MULTIPLIER_MS,

  MAX_SENSOR_MISSES = 3,     //number of beacon slot misses that lead to a child disconnect

  DATA_TRANSMISSION_TIME_IN_MS = 8,
  DATA_TRANSMISSION_TIME_IN_JIFFIES = DATA_TRANSMISSION_TIME_IN_MS * JIFFIES_MULTIPLIER_MS,

  MAX_CONNECTION_ATTEMPTS = 2,   // max number of attempted connection setups to a potential parent
  MAX_TRANSMISSION_FAILURES = 2,  // number of consecutive transmission problems till reconnect (upload failures and missed beacons)

  OVERHEAR_TIMEOUT_DECREASE = ((uint32_t)INTERVAL_TIME_IN_MS/10),
  MIN_OVERHEAR_TIME = 100,

  RANDOM_LISTEN_TIME = 1000,      // random listen for beacons for one second
  RANDOM_LISTEN_BACKOFF = 14400000UL, // every 4 hours

  MAX_UPDATE_CNT = 5,     // The maximum number of potential parents to update.
  FORCED_UPDATE_CNT = 5,  // The first FORCED_UPDATE_CNT potential parents are updated for sure. The remaining MAX_UPDATE_CNT-FORCED_UPDATE_CNT are chosen randomly
  PARENTS_UPDATE_INTERVAL_TIME_IN_MS = 900000UL, // interval between two update cycles in which MAX_UPDATE_CNT potential parents are updated // 15 minutes

  WAKEUP_TIME_IN_MS = 8,
  WAKEUP_TIME_IN_JIFFIES = WAKEUP_TIME_IN_MS * JIFFIES_MULTIPLIER_MS,

  CONNECTION_REQUEST_GUARD_IN_MS = 15,
  CONNECTION_REQUEST_GUARD_IN_JIFFIES = CONNECTION_REQUEST_GUARD_IN_MS * JIFFIES_MULTIPLIER_MS,

  CONNECTION_REQ_TIMEOUT_IN_MS = CRM_BACKOFF_IN_MS + 5,
  CONNECTION_REQ_TIMEOUT_IN_JIFFIES = CONNECTION_REQ_TIMEOUT_IN_MS * JIFFIES_MULTIPLIER_MS,

  HANDSHAKE_TIMEOUT = 10,

  DATA_TIMEOUT_IN_MS  = 30,
  DATA_TIMEOUT_IN_JIFFIES = DATA_TIMEOUT_IN_MS * JIFFIES_MULTIPLIER_MS,

  RSSI_THRESHOLD = -80,
  RSSI_COMM_THRESHOLD = -40,              // beacons below this RSSI value cause a higher rating

  NUM_ACK_TIMEOUT = 1, // Number of data message retransmission attempts

  DATA_MSG_HEADER_SIZE = 8,

  WAIT_BEFORE_RSSI_SNIFF = 1,

  SENSOR_SAMPLING_INTERVAL_IN_JIFFIES = SENSOR_SAMPLING_INTERVAL_IN_MS * JIFFIES_MULTIPLIER_MS,

  OBJECT_DATA_TIMEOUT_IN_JIFFIES = 1280L,

  RSSI_LISTEN_TIME_IN_US = 3000,
  RSSI_LISTEN_TIME_IN_JIFFIES = RSSI_LISTEN_TIME_IN_US * JIFFIES_MULTIPLIER_US,

  ACK_RECEIVE_TIMEOUT_IN_MS = 5,

  // TODO: check if this gets applied correctly
  CQUEUEH_BUFFER_SIZE = 100, // size of the circular queue for data storage
};


enum {
  NUM_PARENTS = 0,
  PARENT_ID = 1,
  NUM_DIS = 2,
  LOAD = 3,
  HOP_COUNT = 4,
  CONNECTING = 5,
  RA_STATE = 6,
  MIS_INTER = 7,
  DELTA_LOW = 8,
  DELTA_HIGH = 9,
  CT_LOW = 10,
  CT_HIGH =11,
  NUM_CONN_ATTEMPTS = 12,
};


#endif /* PROTOCOL_DOZER_DOZER_CONSTANTS_H_ */
