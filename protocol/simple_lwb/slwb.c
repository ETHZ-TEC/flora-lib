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

#include "flora_lib.h"

#if SLWB_ENABLE

slwb_config_t slwb_config;
slwb_round_t slwb_round = {0};
slwb_round_schedule_t round_schedule = {0};

uint16_t slwb_round_period;
uint16_t slwb_round_idx;

uint8_t data_period;
uint8_t data_counter = 0;

void slwb_init(uint16_t uid, slwb_role_t role) {

  slwb_round.round_schedule = &round_schedule;
  slwb_round_idx = 0;

  slwb_config.role = role;
  slwb_config.uid = uid;

  slwb_set_lr_node(slwb_config.role == SLWB_LR);

  slwb_network_init();

  if (slwb_is_lr_node()) {
    data_period = slwb_round_period * SLWB_LR_ROUND_MULT;
  }
  else {
    data_period = (rand() % (5*slwb_round_period) + slwb_round_period);
  }
  sprintf(slwb_print_buffer, "data_period: %d", data_period);
  print(2, slwb_print_buffer);

  slwb_data_generator_init(data_period);

  if (slwb_is_base()) {
    slwb_initialize_scheduler();
  }
}

void slwb_start(uint8_t lr_modulation, uint8_t modulation, uint8_t lr_power, uint8_t power, uint16_t round_period, uint16_t node_id, slwb_role_t role) {
  slwb_round_period = round_period;
  slwb_init(node_id, role);
  sprintf(slwb_print_buffer, "Node: %d, base: %d, lr: %d", slwb_get_id(), slwb_is_base(), slwb_is_lr_node());
  print(1, slwb_print_buffer);

  // initialize the round struct
  slwb_round.round_start = ((hs_timer_get_current_timestamp() + 1*HS_TIMER_FREQUENCY) / GLORIA_SCHEDULE_GRANULARITY) * GLORIA_SCHEDULE_GRANULARITY;
  slwb_round.lr_mod = lr_modulation;
  slwb_round.modulation = modulation;
  slwb_round.lr_pwr = lr_power;
  slwb_round.power_lvl = power;


  if (slwb_is_base()) {
    slwb_scheduler_calculate_round_schedule(&slwb_round);
    slwb_print_schedule(2, &slwb_round);
  }

  slwb_start_round(&slwb_round);
}

void slwb_round_finished() {
  slwb_round_idx += (slwb_is_lr_node()? SLWB_LR_ROUND_MULT : 1);

  if (slwb_is_lr_node()) {
    slwb_round.round_start += ((round_schedule.lr_schedule.lr_period * HS_TIMER_FREQUENCY) / GLORIA_SCHEDULE_GRANULARITY) * GLORIA_SCHEDULE_GRANULARITY;
  }
  else {
    slwb_round.round_start += ((round_schedule.gen_schedule.round_period * HS_TIMER_FREQUENCY) / GLORIA_SCHEDULE_GRANULARITY) * GLORIA_SCHEDULE_GRANULARITY;
  }

  // reset long range schedule
  round_schedule.lr_schedule.n_lr_data = 0;
  round_schedule.lr_schedule.lr_base = 0;
  round_schedule.lr_schedule.lr_ack = 0;

  if (slwb_round_idx < N_ROUNDS) {
    if (slwb_is_base()) {
      slwb_scheduler_calculate_round_schedule(&slwb_round);
      slwb_print_schedule(2, &slwb_round);
    }
    slwb_start_round(&slwb_round);
  }
}

#endif /* SLWB_ENABLE */
