/*
 * gloria.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef PROTOCOL_GLORIA_GLORIA_H_
#define PROTOCOL_GLORIA_GLORIA_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "cli/command.h"
#include "time/hs_timer.h"
#include "protocol/protocol.h"
#include "radio/radio_helpers.h"
#include "system/system.h"

#include "flocklab/flocklab.h"

#include "protocol/gloria/gloria_constants.h"
#include "protocol/gloria/gloria_structures.h"

#include "protocol/gloria/gloria_helpers.h"
#include "protocol/gloria/gloria_radio.h"
#include "protocol/gloria/gloria_time.h"


void gloria_run_flood(gloria_flood_t* flood, void (*callback)());
void gloria_update();

#endif /* PROTOCOL_GLORIA_GLORIA_H_ */
