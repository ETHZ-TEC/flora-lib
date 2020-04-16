/*
 * gloria.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef PROTOCOL_GLORIA_GLORIA_H_
#define PROTOCOL_GLORIA_GLORIA_H_


/* include all gloria related files */
#include "protocol/gloria/gloria.h"
#include "protocol/gloria/gloria_constants.h"
#include "protocol/gloria/gloria_structures.h"
#include "protocol/gloria/gloria_helpers.h"
#include "protocol/gloria/gloria_radio.h"
#include "protocol/gloria/gloria_time.h"
#include "protocol/gloria/gloria_interface.h"


void gloria_run_flood(gloria_flood_t* flood, void (*callback)());
void gloria_update();

#endif /* PROTOCOL_GLORIA_GLORIA_H_ */
