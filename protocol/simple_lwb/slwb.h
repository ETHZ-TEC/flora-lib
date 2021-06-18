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


void slwb_start(uint8_t lr_modulation, uint8_t modulation, uint8_t lr_power, uint8_t power, uint16_t round_period, uint16_t node_id, slwb_role_t role);
void slwb_round_finished();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_H_ */
