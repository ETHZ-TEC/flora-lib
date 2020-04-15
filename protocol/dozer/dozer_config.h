/*
 * dozer_config.h
 *
 *  Created on: Jun 22, 2018
 *      Author: kelmicha
 */

#ifndef DOZER_CONFIG_H_
#define DOZER_CONFIG_H_

enum {
  INTERVAL_TIME_IN_MS = 10000,
  MAX_JITTER_IN_MS = 300,
  MAX_CHILDREN = 10,
  NUM_PROCESSING_SLOTS = 4,                // currently just used for random data generation
  MAX_HOPS = 13,
  MAX_PARENTS = 5,
#ifdef SHORT_SAMPLING_INT
  SENSOR_SAMPLING_INTERVAL_IN_MS = INTERVAL_TIME_IN_MS,
#else
  SENSOR_SAMPLING_INTERVAL_IN_MS = 120000UL,
#endif

  MODULATION_INDEX = 8,
  BAND_INDEX = 33,
  TX_POWER = 14,

  DOZER_REMOVE_DUPLICATES = 1,


#ifndef DEVKIT
  SINK_ID = 49,
#else
  SINK_ID = 47,
#endif

};

#endif /* DOZER_DOZER_CONFIG_H_ */
