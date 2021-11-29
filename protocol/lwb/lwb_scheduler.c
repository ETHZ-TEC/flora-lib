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

#if LWB_ENABLE

/**
 * @brief struct to store information about active nodes on the host
 */
typedef struct lwb_node_info {
  struct lwb_node_info* next;
  uint16_t              id;         /* node ID */
  uint16_t              ipi;        /* bandwidth demand in number of packets */
  /* note: use 16 bits due to alignment */
  uint32_t              t_last_req; /* time of the last request, in seconds */
} lwb_node_list_t;

static lwb_time_t         lwb_time;                                               /* global time in microseconds */
static uint32_t           lwb_time_ofs;                                           /* time offset */
static uint32_t           base_period = LWB_SCHED_PERIOD * LWB_TIMER_FREQUENCY;   /* base (idle) period in ticks */
static uint32_t           min_period;
static uint32_t           n_nodes;                                                /* # active nodes */
static lwb_node_list_t    node_list[LWB_MAX_NUM_NODES];                           /* actual list */
static lwb_node_list_t*   head;                                                   /* head of the linked list */

#if LWB_USE_TX_DELAY
static uint8_t            delay_mask[LWB_TX_DELAY_MASK_SIZE];
static bool               delay_mask_set;
#endif /* LWB_USE_TX_DELAY */

/* returns true if a stream with the requested IPI can be added to the schedule */
bool lwb_sched_check_utilization(uint16_t requested_ipi)
{
  /* calculate the current bandwidth utilization */
  if (requested_ipi > 0) {
    lwb_node_list_t* curr_node = head;
    float pkt_rate = 1.0 / requested_ipi;
    while (curr_node != 0) {
      pkt_rate += 1.0 / curr_node->ipi;
      curr_node = curr_node->next;
    }
    uint32_t required_period = LWB_TIMER_FREQUENCY * (LWB_MAX_DATA_SLOTS / pkt_rate);
    if (required_period < min_period) {
      return false;
    }
  }
  return true;
}


bool lwb_sched_add_node(uint16_t node_id, uint16_t ipi)
{
  if (n_nodes >= LWB_MAX_NUM_NODES) {
    LOG_WARNING("request ignored (max #nodes reached)");
    return false;
  }
  if (node_id == DPP_DEVICE_ID_SINK      ||
      node_id == DPP_DEVICE_ID_BROADCAST ||
      node_id == LWB_NODE_ID             ||
      node_id < LWB_MIN_NODE_ID          ||
      node_id > LWB_MAX_NODE_ID) {
    LOG_WARNING("invalid node ID %u", node_id);
    return false;
  }
  /* check utilization */
  if (!lwb_sched_check_utilization(ipi)) {
    LOG_WARNING("request rejected (traffic demand exceeds capacity limit)");
    return false;
  }
  lwb_node_list_t* node = 0;
  uint32_t i;
  /* find a free spot */
  for (i = 0; i < LWB_MAX_NUM_NODES; i++) {
    if (node_list[i].id == 0) {
      node = &node_list[i];   /* use this spot */
      break;
    }
  }
  if (node == 0) {
    /* this will never happen (if there's no bug in the code) */
    LOG_ERROR("out of memory: request dropped");
    return false;
  }
  node->id         = node_id;
  node->ipi        = ipi;
  node->t_last_req = lwb_time / 1000000;
  /* insert the node into the list, ordered by node id */
  if (!head || node_id < head->id) {
    node->next = head;
    head = node;
  } else {
    lwb_node_list_t* prev;
    for (prev = head; prev != 0; prev = prev->next) {
      if ((node_id >= prev->id) && ((prev->next == 0) || (node_id < prev->next->id))) {
        break;  /* this is the spot */
      }
    }
    node->next = prev->next;
    prev->next = node;
  }
  n_nodes++;
  LOG_INFO("node %u registered (IPI %u)", node_id, ipi);

  return true;
}


void lwb_sched_remove_node(lwb_node_list_t* node)
{
  if (node == 0 || n_nodes == 0) {
    LOG_ERROR("invalid argument for remove_node");
    return;
  }
  LOG_INFO("node %u removed", node->id);
  node->id = 0;   /* mark as 'unused' by setting the ID to zero */
  n_nodes--;
  if (node == head) {
    head = node->next;
  } else {
    /* find predecessor node in linked list */
    lwb_node_list_t* prev;
    for (prev = head; prev != 0; prev = prev->next) {
      if (prev->next == node) {
        prev->next = node->next;
        break;
      }
    }
  }
}


bool lwb_sched_process_req(uint16_t id,
                           uint16_t ipi)
{
  lwb_node_list_t* node = 0;
  /* check if node already exists */
  for (node = head; node != 0; node = node->next) {
    if (id == node->id) {
      /* if the requested IPI is larger than the current, check if it still fits within the bus capacity */
      if ((ipi > node->ipi) && !lwb_sched_check_utilization(ipi - node->ipi)) {
        LOG_WARNING("request rejected (traffic demand exceeds capacity limit)");
        return false;
      }
      node->ipi        = ipi;
      node->t_last_req = lwb_time / 1000000;    /* set to current time */
      LOG_VERBOSE("IPI of node %u adjusted to %us", id, node->ipi);
      return true;
    }
  }
  /* does not exist: add the node */
  return lwb_sched_add_node(id, ipi);
}


uint32_t lwb_sched_compute(lwb_schedule_t* const sched,
                           uint32_t reserve_slots_host)
{
  lwb_node_list_t* curr_node = head;

  /* clear schedule content */
  memset(sched->slot, 0, sizeof(sched->slot));
  sched->n_slots = 0;

  /* determine the required period (for this we need to know the total traffic demand per second) */
  curr_node = head;
  float pkt_rate = 0;
  while (curr_node != 0) {
    if (curr_node->ipi) {
      pkt_rate += 1.0 / curr_node->ipi;
    }
    curr_node = curr_node->next;
  }
  uint32_t curr_period = LWB_TIMER_FREQUENCY * (LWB_MAX_DATA_SLOTS / pkt_rate);
  if (curr_period > base_period) {
    curr_period = base_period;
  } else if (curr_period < min_period) {
    curr_period = min_period;
  }

  /* increment time */
  lwb_time += LWB_TICKS_TO_US(sched->period);
  uint32_t curr_time = lwb_time / 1000000;      /* current time in seconds */

  /* assign slots to the host */
  while (reserve_slots_host && sched->n_slots < LWB_MAX_SLOTS_HOST) {
    sched->slot[sched->n_slots++] = LWB_NODE_ID;
    reserve_slots_host--;
  }

  /* go through the list of nodes and assign the requested slots */
  curr_node = head;
  while (curr_node != 0 && (sched->n_slots < LWB_MAX_DATA_SLOTS)) {
    if (curr_node->ipi) {
      uint32_t n_slots = (curr_time - curr_node->t_last_req) / curr_node->ipi;
      while (n_slots && (sched->n_slots < LWB_MAX_DATA_SLOTS)) {
        sched->slot[sched->n_slots++] = curr_node->id;
        curr_node->t_last_req += curr_node->ipi;
        n_slots--;
      }
    }
    curr_node = curr_node->next;
  }

  /* complete the schedule */
  sched->period        = curr_period;
  sched->time          = lwb_time + lwb_time_ofs;
  sched->host_id       = LWB_NODE_ID;
  sched->header.net_id = LWB_NETWORK_ID;    /* set the network ID */
  sched->header.type   = 1;                 /* packet type 'schedule' */

  uint16_t sched_len = LWB_SCHED_HDR_LEN + sched->n_slots * 2;

#if LWB_USE_TX_DELAY
  if (delay_mask_set) {
    memcpy(&sched->slot[sched->n_slots], delay_mask, LWB_TX_DELAY_MASK_SIZE);
    sched->has_delay_mask = 1;
    sched_len += LWB_TX_DELAY_MASK_SIZE;
  }
#endif /* LWB_USE_TX_DELAY */

#if LWB_SCHED_ADD_CRC
  uint16_t crc = crc16((uint8_t*)sched, sched_len, 0);
  sched->rxbuffer[sched_len - LWB_SCHED_HDR_LEN] = crc & 0xff;
  sched->rxbuffer[sched_len - LWB_SCHED_HDR_LEN + 1] = (crc >> 8) & 0xff;
  sched_len += LWB_SCHED_CRC_LEN;
#endif /* LWB_SCHED_ADD_CRC */

  /* log the parameters of the new schedule */
  LOG_VERBOSE("schedule updated (s=%lu T=%lums n=%u l=%u)", n_nodes, (uint32_t)LWB_TICKS_TO_MS(sched->period), sched->n_slots, sched_len);

  return sched_len;
}


uint32_t lwb_sched_init(lwb_schedule_t* sched)
{
  uint32_t t_round_max = lwb_get_max_round_duration(0, 0, 0);

  LOG_INFO("rounds [ms]: T=%lu000ms round_max=%lums",
           LWB_TICKS_TO_S(base_period),
           (uint32_t)LWB_TICKS_TO_MS(t_round_max));

  /* make sure the minimal round period is not smaller than the max. round duration! */
  if (base_period <= t_round_max) {
    LOG_ERROR("invalid parameters");
    return 0;
  }
  /* initialize node list */
  memset(node_list, 0, sizeof(lwb_node_list_t) * LWB_MAX_NUM_NODES);
  head       = 0;
  n_nodes    = 0;
  min_period = t_round_max + LWB_TIMER_FREQUENCY / 10;    /* add some slack time */

#if LWB_USE_TX_DELAY
  memset(delay_mask, 0, LWB_TX_DELAY_MASK_SIZE);
  delay_mask_set = false;
#endif /* LWB_USE_TX_DELAY */

#ifdef LWB_SCHED_NODE_LIST
  const uint16_t node_ids[] = { LWB_SCHED_NODE_LIST };
  uint32_t i;
  for (i = 0; i < sizeof(node_ids) / 2; i++) {
    lwb_sched_add_node(node_ids[i], LWB_TICKS_TO_S(base_period));         /* add one slot per node and round by default */
  }
#endif /* LWB_SCHED_NODE_LIST */

  return lwb_sched_compute(sched, 0);
}


bool lwb_sched_check_params(uint32_t period_secs, uint32_t sched_slot, uint32_t cont_slot, uint32_t data_slot)
{
  if (period_secs == 0) {
    period_secs = LWB_TICKS_TO_S(base_period);
  }
  if ((period_secs > LWB_TICKS_TO_S(lwb_get_max_round_duration(sched_slot, cont_slot, data_slot))) && (period_secs <= LWB_SCHED_PERIOD_MAX_S)) {
    return true;
  }
  return false;
}


uint32_t lwb_sched_get_period(void)
{
  /* period in seconds */
  return LWB_TICKS_TO_S(base_period);
}


bool lwb_sched_set_period(uint32_t period_secs)
{
  if (!lwb_sched_check_params(period_secs, 0, 0, 0)) {
    return false;
  }
  base_period = LWB_S_TO_TICKS(period_secs);
  return true;
}


lwb_time_t lwb_sched_get_time(void)
{
  return lwb_time;
}


void lwb_sched_set_time(lwb_time_t time_us)
{
  lwb_time = time_us;
}


void lwb_sched_set_time_offset(uint32_t ofs)
{
  /* convert from ticks to us */
  lwb_time_ofs = LWB_TICKS_TO_US(ofs);
}


#if LWB_USE_TX_DELAY

void lwb_sched_set_delay_nodes(const uint16_t* delay_node_list, uint8_t num_nodes)
{
  /* clear delay node mask */
  memset(delay_mask, 0, LWB_TX_DELAY_MASK_SIZE);
  delay_mask_set = false;

  if (delay_node_list && num_nodes) {
    uint32_t i;
    for (i = 0; i < num_nodes; i++) {
      if (delay_node_list[i] >= LWB_MIN_NODE_ID && delay_node_list[i] <= LWB_MAX_NODE_ID) {
        uint16_t ofs = delay_node_list[i] - LWB_MIN_NODE_ID;
        delay_mask[ofs / 8] |= 1 << (ofs & 0x7);
        delay_mask_set = true;
      }
    }
  }
}

#endif /* LWB_USE_TX_DELAY */

#endif /* LWB_ENABLE */
