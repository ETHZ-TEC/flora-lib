/*
 * swlb.h
 *
 *  Created on: Oct 12, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_H_


#ifndef SLWB_ENABLE
#define SLWB_ENABLE       0
#endif /* SLWB_ENABLE */

/* include all sLWB related files */
#include "protocol/simple_lwb/slwb_constants.h"
#include "protocol/simple_lwb/slwb_structures.h"
#include "protocol/simple_lwb/slwb_scheduler.h"
#include "protocol/simple_lwb/slwb_manager.h"
#include "protocol/simple_lwb/slwb_data_generator.h"
#include "protocol/simple_lwb/slwb_timer_sync.h"
#include "protocol/simple_lwb/slwb_network.h"

#include "protocol/simple_lwb/helpers.h"
#include "protocol/simple_lwb/linked_list.h"


extern uint16_t slwb_round_period;
extern uint16_t slwb_round_idx;


void slwb_start(uint8_t lr_modulation, uint8_t modulation, uint8_t lr_power, uint8_t power, uint16_t round_period);
void slwb_round_finished();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_H_ */
