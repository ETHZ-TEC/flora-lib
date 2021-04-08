/*
 * Copyright (c) 2021, Swiss Federal Institute of Technology (ETH Zurich).
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
  uint16_t              id;        /* node ID */
  uint16_t              n_pkts;    /* bandwidth demand in number of packets */
  /* note: use 16 bits due to alignment */
  uint32_t              t_last_req; /* time of the last request, in seconds */
} lwb_node_list_t;

static lwb_time_t         lwb_time;                                               /* global time in microseconds */
static uint32_t           lwb_time_ofs;                                           /* time offset */
static uint32_t           base_period = LWB_SCHED_PERIOD * LWB_TIMER_FREQUENCY;   /* base (idle) period in ticks */
static uint32_t           n_nodes;                                                /* # active nodes */
static lwb_node_list_t    node_list[LWB_MAX_NODES];                               /* actual list */
static lwb_node_list_t*   head;                                                   /* head of the linked list */


bool lwb_sched_add_node(uint16_t node_id, uint16_t n_pkts)
{
  if (node_id == DPP_DEVICE_ID_SINK || node_id == DPP_DEVICE_ID_BROADCAST || node_id == NODE_ID) {
    return false;     /* invalid argument */
  }
  if (n_nodes >= LWB_MAX_NODES) {
    LOG_WARNING("request ignored (max #nodes reached)");
    return false;
  }
  lwb_node_list_t* node = 0;
  uint32_t i;
  /* find a free spot */
  for (i = 0; i < LWB_MAX_NODES; i++) {
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
  node->n_pkts     = n_pkts;
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
  LOG_INFO("node %u registered", node_id);
  return true;
}


void lwb_sched_remove_node(lwb_node_list_t* node)
{
  while (node == 0 || n_nodes == 0) {
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


void lwb_sched_process_req(uint16_t id,
                           uint16_t ipi)
{
  uint16_t n_pkts = 0;
  if (ipi) {
    n_pkts = MAX(base_period / ipi, 1);     //TODO: make it compatible with dynamic round periods
    if (n_pkts > LWB_MAX_DATA_SLOTS) {
      n_pkts = LWB_MAX_DATA_SLOTS;
    }
  }
  lwb_node_list_t* node = 0;
  /* check if node already exists */
  for (node = head; node != 0; node = node->next) {
    if (id == node->id) {
      node->n_pkts     = n_pkts;
      node->t_last_req = lwb_time / 1000000;
      return;
    }
  }
  /* does not exist: add the node */
  lwb_sched_add_node(id, n_pkts);
}


uint32_t lwb_sched_compute(lwb_schedule_t* const sched,
                           uint32_t reserve_slots_host)
{
  uint32_t req_nodes         = 0;
  uint32_t req_slots         = 0;
  uint32_t n_slots_assigned  = 0;
  lwb_node_list_t *curr_node = 0;

  memset(sched->slot, 0, sizeof(sched->slot));  /* clear schedule content */

  /* first, go through the list of nodes and calculate the total traffic demand */
  curr_node = head;
  while (curr_node != 0) {
    if (curr_node->n_pkts) {
      req_nodes++;
      req_slots += curr_node->n_pkts;
    }
    curr_node = curr_node->next;
  }
#if LWB_SCHED_FAIR
  /* if total demand exceeds the avail. bandwidth, then scale each request */
  if (req_slots > LWB_MAX_DATA_SLOTS) {
    /* note: don't use floating point calculations here, takes up a lot of
     * flash memory space! */
    /* assumption: n_pkts < 100 and LWB_MAX_DATA_SLOTS < 100 */
    uint32_t scale = LWB_MAX_DATA_SLOTS * 256 / req_slots;
    /* go again through the list of nodes and assign slots in a 'fair' way */
    curr_node = head;
    while (curr_node != 0 &&
          (n_slots_assigned < LWB_MAX_DATA_SLOTS)) {
      if (curr_node->n_pkts) {
        uint32_t n = (scale * curr_node->n_pkts + 128) / 256;
        while (n && (n_slots_assigned < LWB_MAX_DATA_SLOTS)) {
          sched->slot[n_slots_assigned++] = curr_node->id;
          n--;
        }
      }
      curr_node = curr_node->next;
    }
  } else
#endif /* LWB_SCHED_FAIR */
  {
    /* go again through the list of nodes and assign the requested slots */
    curr_node = head;
    while (curr_node != 0 && (n_slots_assigned < LWB_MAX_DATA_SLOTS)) {
      /* assign as many slots as the node requested */
      while (curr_node->n_pkts &&
            (n_slots_assigned < LWB_MAX_DATA_SLOTS)) {
        sched->slot[n_slots_assigned++] = curr_node->id;
        curr_node->n_pkts--;
      }
      curr_node = curr_node->next;
    }
  }
  sched->n_slots = n_slots_assigned;
  sched->period  = base_period;
  sched->time    = lwb_time + lwb_time_ofs;

  uint16_t sched_len = LWB_SCHED_HDR_LEN + n_slots_assigned * 2;

  /* set the network ID */
  sched->header.net_id = LWB_NETWORK_ID;
  sched->header.type   = 1;

#if LWB_SCHED_ADD_CRC
  sched->slot[n_slots_assigned] = crc16((uint8_t*)sched, sched_len, 0);
  sched_len += LWB_SCHED_CRC_LEN;
#endif /* LWB_SCHED_ADD_CRC */

  /* log the parameters of the new schedule */
  LOG_INFO("schedule updated (s=%lu T=%lums n=%lu l=%lu)", n_nodes, LWB_TICKS_TO_MS(sched->period), n_slots_assigned, sched_len);

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
  memset(node_list, 0, sizeof(lwb_node_list_t) * LWB_MAX_NODES);
  head           = 0;
  n_nodes        = 0;
  sched->n_slots = 0;
  sched->time    = lwb_time + lwb_time_ofs;
  sched->period  = base_period;
  sched->host_id = NODE_ID;
  sched->header.net_id = LWB_NETWORK_ID;
  sched->header.type   = 1;

#ifdef LWB_SCHED_NODE_LIST
  const uint16_t node_ids[] = { LWB_SCHED_NODE_LIST };
  uint32_t i;
  for (i = 0; i < sizeof(node_ids) / 2; i++) {
    if (lwb_sched_add_node(node_ids[i], 1)) {     /* add one slot per node by default */
      sched->slot[sched->n_slots++] = node_ids[i];
    }
  }
  LOG_INFO("%u source nodes registered:", sched->n_slots);
#endif /* LWB_SCHED_NODE_LIST */

#if LWB_SCHED_ADD_CRC
  uint16_t crc = crc16((uint8_t*)sched, LWB_SCHED_HDR_LEN, 0);
  memcpy((uint8_t*)sched + LWB_SCHED_HDR_LEN, &crc, 2);
  return LWB_SCHED_HDR_LEN + 2;   /* empty schedule, no slots allocated yet */
#else
  return LWB_SCHED_HDR_LEN;       /* empty schedule, no slots allocated yet */
#endif /* LWB_SCHED_ADD_CRC */
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

#endif /* LWB_ENABLE */
