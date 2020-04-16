/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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

#if ELWB_ENABLE

/* macros for time unit conversions */
#define TICKS_TO_SCHEDUNITS(x) ((x) / (ELWB_TIMER_SECOND / ELWB_PERIOD_SCALE))
#define SCHEDUNITS_TO_MS(x)    ((x) * (1000 / ELWB_PERIOD_SCALE))

/* earliest start (offset) of the request round, + 10ms slack (necessary due
 * to rounding issue) */
#define ELWB_T_IDLE_ROUND     (TICKS_TO_SCHEDUNITS(ELWB_CONF_T_SCHED + \
                                2 * ELWB_CONF_T_CONT + 2 * ELWB_CONF_T_GAP +\
                                ELWB_CONF_SCHED_COMP_TIME))

/* duration of the 'request round' = start of the data round,
 * depends on the max. # nodes (= ELWB_CONF_MAX_N_NODES), + add 10ms slack */
#define ELWB_T_REQ_ROUND_MAX  (TICKS_TO_SCHEDUNITS( \
                                ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP + \
                                ELWB_CONF_MAX_N_NODES * \
                                 (ELWB_CONF_T_CONT + ELWB_CONF_T_GAP) + \
                                ELWB_CONF_SCHED_COMP_TIME))

#define ELWB_T_DATA_ROUND_MAX (TICKS_TO_SCHEDUNITS( \
                                ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP + \
                                ELWB_CONF_MAX_DATA_SLOTS * \
                                 (ELWB_CONF_T_DATA + ELWB_CONF_T_GAP)))

/* note: round up for the following values */
#define ELWB_T_ROUND_MAX      (ELWB_T_IDLE_ROUND + ELWB_T_REQ_ROUND_MAX + \
                               ELWB_T_DATA_ROUND_MAX + \
                               TICKS_TO_SCHEDUNITS(2 * \
                                ELWB_CONF_SCHED_COMP_TIME))

/*---------------------------------------------------------------------------*/
/**
 * @brief struct to store information about active nodes on the host
 */
typedef struct elwb_node_info {
  struct elwb_node_info* next;
  uint16_t               id;        /* node ID */
  uint16_t               n_pkts;    /* bandwidth demand in number of packets */
  /* note: use 16 bits due to alignment */
  uint32_t               t_last_req;/* time of the last request */
} elwb_node_list_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  ELWB_SCHED_STATE_IDLE = 0,
  ELWB_SCHED_STATE_CONT_DET,
  ELWB_SCHED_STATE_REQ,
  ELWB_SCHED_STATE_DATA,
} elwb_sched_state_t;
/*---------------------------------------------------------------------------*/
static uint64_t            time;                              /* global time */
static uint32_t            period;          /* base (idle) period in seconds */
static uint32_t            n_nodes;                        /* # active nodes */
static elwb_sched_state_t  sched_state;
static elwb_node_list_t    node_list[ELWB_CONF_MAX_N_NODES];  /* actual list */
static elwb_node_list_t*   head;                  /* head of the linked list */
static uint16_t            slots_buffer[ELWB_CONF_MAX_DATA_SLOTS];
/*---------------------------------------------------------------------------*/
/* the number of bits for depth and length are stored in the thirds slot;
 * 5 bits are reserved to store the number of bits needed for the depth 
 * (i.e. 0 to 31 bits) */
#define GET_D_BITS()       (compressed_data[2] >> 3)
/* 3 bits are reserved to store the number of bits needed for the length
 * (i.e. 0 to 7 bits) */
#define GET_L_BITS()       (compressed_data[2] & 0x07)  
#define SET_D_L_BITS(d, l) (compressed_data[2] = ((d) << 3) | ((l) & 0x07))
#define COMPR_SLOT(a)      (compressed_data[3 + (a)])
#ifndef MIN
#define MIN(x, y)          ((x) < (y) ? (x) : (y))
#endif /* MIN */
/*---------------------------------------------------------------------------*/
static inline uint32_t get_min_bits(uint32_t a)
{
  uint32_t i;
  for (i = 15; i > 0; i--) {
    if (a & (1 << i)) {
      return i + 1;
    }
  }
  return i + 1;
}
/*---------------------------------------------------------------------------*/
uint32_t elwb_sched_compress(uint8_t* compressed_data, uint32_t n_slots)
{
  if (!compressed_data || n_slots > ELWB_CONF_MAX_DATA_SLOTS) {
    return 0;
  }
  if (n_slots < 2) {  /* don't do anything in case there is only 0 or 1 slot */
    return n_slots * 2;
  }
  /* copy the input data into a buffer */
  memcpy(slots_buffer, compressed_data, n_slots * 2);
  /* clear the output data buffer (except for the first slot!) */
  memset(compressed_data + 2, 0, ELWB_CONF_MAX_DATA_SLOTS * 2 - 2);
  
  /* Note: the first slot holds the first node ID */
  
  uint32_t  n_runs = 0;    /* how many times the delta has changed */
  uint16_t  d[n_slots - 1];
  d[n_runs] = slots_buffer[1] - slots_buffer[0];  /* delta (step size) */
  uint16_t  d_max = d[n_runs];
  /* length (how many consecutive slots with step size d) */
  uint16_t  l[n_slots - 1];
  l[n_runs] = 0;
  uint32_t  l_max = l[n_runs];
  uint32_t  idx;

  for (idx = 1; idx < n_slots - 1; idx++) {
    if ((slots_buffer[idx + 1] - slots_buffer[idx]) == d[n_runs]) {
      l[n_runs]++;
    } else {
      if (l[n_runs] > l_max) {
        /* keep track of the max. num. of conseq. slots with const. delta */
        l_max = l[n_runs]; 
      }
      n_runs++; 
      /* calculate the new delta */
      d[n_runs] = slots_buffer[idx + 1] - slots_buffer[idx]; 
      /* make sure the node IDs are in increasing order */
      if (slots_buffer[idx + 1] < slots_buffer[idx]) {
        return 0; /* node IDs are not sorted! */
      }
      if (d[n_runs] > d_max) {
        d_max = d[n_runs];  /* keep track of the max. delta */
      }
      l[n_runs] = 0;
    }
  }
  if (l[n_runs] > l_max) {
    l_max = l[n_runs];
  }
  n_runs++;
  
  uint32_t d_bits = get_min_bits(d_max);
  uint32_t l_bits = get_min_bits(l_max);
  /* required bits for each delta + length */
  uint32_t run_bits = d_bits + l_bits;
  
  for (idx = 0; idx < n_runs; idx++) {
    uint32_t offset_bits = run_bits * idx;
    /* store the current and the following 3 bytes in a 32-bit variable */
    uint32_t tmp = (uint32_t)COMPR_SLOT(offset_bits / 8) | 
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 1) << 8)  |
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 2) << 16) |
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 3) << 24);
    /* append the new data (d and l) */
    tmp |= ( ( ((uint32_t)d[idx] << l_bits) | l[idx] ) << (offset_bits % 8) );
    COMPR_SLOT(offset_bits / 8)     = (uint8_t)tmp;
    COMPR_SLOT(offset_bits / 8 + 1) = (uint8_t)(tmp >> 8);
    COMPR_SLOT(offset_bits / 8 + 2) = (uint8_t)(tmp >> 16);
    COMPR_SLOT(offset_bits / 8 + 3) = (uint8_t)(tmp >> 24);
  }
  SET_D_L_BITS(d_bits, l_bits);   /* store the number of bits for d and l */
  
  /* return the size of the compressed schedule */
  return 3 + (((n_runs * run_bits) + 7) >> 3);
}
/*---------------------------------------------------------------------------*/
bool elwb_sched_uncompress(uint8_t* compressed_data, uint32_t n_slots)
{
  if (!compressed_data || n_slots > ELWB_SCHED_MAX_SLOTS) {
    return false;
  }
  if (n_slots < 2) {  /* don't do anything in case there is only 0 or 1 slot */
    return true;
  }
  
  slots_buffer[0] = (uint16_t)compressed_data[1] << 8 | compressed_data[0];

  uint32_t d_bits = GET_D_BITS();
  uint32_t l_bits = GET_L_BITS();
  uint32_t run_bits = d_bits + l_bits;
  
  /* check whether the values make sense */
  if (d_bits == 0 || d_bits > 16 || l_bits == 0) {
    return false; /* invalid d or l bits */
  }
  
  uint32_t slot_idx = 1;
  uint32_t idx;
  uint32_t mask = (((uint32_t)1 << run_bits) - 1);
  for (idx = 0; slot_idx < n_slots; idx++) {
    /* extract d and l of this run */
    uint32_t offset_bits = run_bits * idx;
    uint32_t tmp = (uint32_t)COMPR_SLOT(offset_bits / 8) |
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 1) << 8)  |
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 2) << 16) | 
                   ((uint32_t)COMPR_SLOT(offset_bits / 8 + 3) << 24);
    uint32_t run_info = ((tmp >> (offset_bits % 8)) & mask);
    uint32_t d = run_info >> l_bits;
    uint32_t l = run_info & ((1 << l_bits) - 1);
    uint32_t i;
    /* generate the slots */
    for (i = 0; (i < l + 1) && (slot_idx < n_slots); i++) {
      /* add the offset to the previous slot */
      slots_buffer[slot_idx] = slots_buffer[slot_idx - 1] + d;
      slot_idx++;
    }
  }  
  memcpy(compressed_data, slots_buffer, n_slots * 2);
  
  return true;
}
/*---------------------------------------------------------------------------*/
void elwb_sched_add_node(uint16_t id)
{
  if (id == 0) {
    return;     /* invalid argument */
  }
  if (n_nodes >= ELWB_CONF_MAX_N_NODES) {
    LOG_WARNING("request ignored, max #nodes reached");
    EVENT_WARNING(EVENT_CC430_NODE_REMOVED, id);
    return;
  }
  elwb_node_list_t* node = 0;
  uint32_t i;
  /* find a free spot */
  for (i = 0; i < ELWB_CONF_MAX_N_NODES; i++) {
    if (node_list[i].id == 0) {
      node = &node_list[i];   /* use this spot */
      break;
    }
  }
  if (node == 0) {
    /* this will never happen (if there's no bug in the code) */
    LOG_ERROR("out of memory: request dropped");
    return;
  }
  node->id         = id;
  node->n_pkts     = 0;
  node->t_last_req = (time / ELWB_PERIOD_SCALE);
  /* insert the node into the list, ordered by node id */
  if (!head || id < head->id) {
    node->next = head;
    head = node;
  } else {
    elwb_node_list_t* prev;
    for (prev = head; prev != 0; prev = prev->next) {
      if ((id >= prev->id) && ((prev->next == 0) ||
         (id < prev->next->id))) {
        break;  /* this is the spot */
      }
    }
    node->next = prev->next;
    prev->next = node;
  }
  n_nodes++;
  LOG_INFO("node %u registered", id);
  EVENT_INFO(EVENT_CC430_NODE_ADDED, id);
}
/*---------------------------------------------------------------------------*/
void elwb_sched_remove_node(elwb_node_list_t* node)
{
  while (node == 0 || n_nodes == 0) {
    LOG_ERROR("invalid argument for remove_node");
    return;
  }
  LOG_INFO("node %u removed", node->id);
  EVENT_INFO(EVENT_CC430_NODE_REMOVED, node->id);
  node->id = 0;   /* mark as 'unused' by setting the ID to zero */
  n_nodes--;
  if (node == head) {
    head = node->next;
  } else {
    /* find predecessor node in linked list */
    elwb_node_list_t* prev;
    for (prev = head; prev != 0; prev = prev->next) {
      if (prev->next == node) {
        prev->next = node->next;
        break;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void elwb_sched_process_req(uint16_t id,
                            uint32_t n_pkts)
{
  if (n_pkts > ELWB_CONF_MAX_DATA_SLOTS) {
    n_pkts = ELWB_CONF_MAX_DATA_SLOTS;      /* cap */
  }
  elwb_node_list_t *node = 0;
  /* check if node already exists */
  for (node = head; node != 0; node = node->next) {
    if (id == node->id) {
      node->n_pkts = MIN(n_pkts, ELWB_CONF_MAX_DATA_SLOTS);
      node->t_last_req = (time / ELWB_PERIOD_SCALE);
      return;
    }
  }
  /* does not exist: add the node */
  elwb_sched_add_node(id);
}
/*---------------------------------------------------------------------------*/
uint32_t elwb_sched_compute(elwb_schedule_t * const sched,
                            uint32_t reserve_slots_host)
{
#define ELWB_SCHED_PRINT_BUFFER_SIZE  128
  static char print_buffer[ELWB_SCHED_PRINT_BUFFER_SIZE];
  static uint32_t t_round   = ELWB_T_IDLE_ROUND;
  static uint32_t req_nodes = 0;
  static uint32_t req_slots = 0;
  uint32_t n_slots_assigned = 0;

  /*
   * note: the schedule is sent at the beginning of the next round,
   * i.e. it must include the next period
   */
  if (sched_state == ELWB_SCHED_STATE_IDLE && sched->period == 0) {
    /* request detected! prepare 2nd schedule (just update the period) */
    sched->n_slots = 0;
    sched->period  = t_round;                  /* use current round duration */
    sched_state    = ELWB_SCHED_STATE_CONT_DET;              /* change state */
    return 2;                            /* return value doesn't matter here */
  }
  /* use the period of the last round to update the network time */
  time += sched->period;
  
  if (sched_state == ELWB_SCHED_STATE_CONT_DET) {
    /* contention has been detected, now schedule request slots for all 
     * registered nodes in the network */
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    /* every node gets one slot (a chance to request slots) */
    elwb_node_list_t *curr_node = head;
    while (curr_node != 0) {
      sched->slot[n_slots_assigned++] = curr_node->id;
      curr_node = curr_node->next;
    }
    if (n_slots_assigned == 0) {
      /* if there are no registered nodes, then add a dummy slot */
      sched->slot[n_slots_assigned++] = DPP_DEVICE_ID_BROADCAST;
    }
    sched->n_slots = n_slots_assigned;
    /* calculate next round period */
    sched->period  = TICKS_TO_SCHEDUNITS(ELWB_CONF_T_SCHED +
                                         ELWB_CONF_T_GAP + n_slots_assigned *
                                         (ELWB_CONF_T_CONT + ELWB_CONF_T_GAP) +
                                         ELWB_CONF_SCHED_COMP_TIME);
    t_round += sched->period;   /* append period of previous sub-round */
    
    sched_state = ELWB_SCHED_STATE_REQ;
    
  } else if (sched_state == ELWB_SCHED_STATE_REQ) {
    /* a request round has just finished, now calculate the schedule based on
     * the received requests */
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    req_nodes = 0;
    req_slots = 0;
    n_slots_assigned = 0;
    elwb_node_list_t *curr_node = 0;
    
    /* first, go through the list of nodes and calculate the traffic demand */
    curr_node = head;
    while (curr_node != 0) {
      if (curr_node->n_pkts) {
        req_nodes++;
        req_slots += curr_node->n_pkts;
      }
      curr_node = curr_node->next;
    }
  #if ELWB_CONF_SCHED_FAIR
    /* if total demand exceeds the avail. bandwidth, then scale each request */
    if (req_slots > ELWB_CONF_MAX_DATA_SLOTS) {
      /* note: don't use floating point calculations here, takes up a lot of
       * flash memory space! */
      /* assumption: n_pkts < 100 and ELWB_CONF_MAX_DATA_SLOTS < 100 */
      uint32_t scale = ELWB_CONF_MAX_DATA_SLOTS * 256 / req_slots;
      /* go again through the list of nodes and assign slots in a 'fair' way */
      curr_node = head;
      while (curr_node != 0 &&
            (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
        if (curr_node->n_pkts) {
          uint32_t n = (scale * curr_node->n_pkts + 128) / 256;
          while (n && (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
            sched->slot[n_slots_assigned++] = curr_node->id;
            n--;
          }
        }
        curr_node = curr_node->next;
      }
    } else 
  #endif /* ELWB_CONF_SCHED_FAIR */
    {
      /* go again through the list of nodes and assign the requested slots */
      curr_node = head;
      while (curr_node != 0 && (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
        /* assign as many slots as the node requested */
        while (curr_node->n_pkts &&
              (n_slots_assigned < ELWB_CONF_MAX_DATA_SLOTS)) {
          sched->slot[n_slots_assigned++] = curr_node->id;
          curr_node->n_pkts--;
        }
        curr_node = curr_node->next;
      }
    }
    sched->n_slots = n_slots_assigned;
    sched->period  = period - t_round;
    t_round += TICKS_TO_SCHEDUNITS(ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP +
                n_slots_assigned * (ELWB_CONF_T_DATA + ELWB_CONF_T_GAP) + 
#if ELWB_CONF_DATA_ACK
                ELWB_CONF_T_DACK + ELWB_CONF_T_GAP +
#endif /* ELWB_CONF_DATA_ACK */
                ELWB_CONF_SCHED_COMP_TIME);
    ELWB_SCHED_SET_DATA_SLOTS(sched); /* mark the next round as 'data round' */
    ELWB_SCHED_SET_STATE_IDLE(sched);     /* mark as 'idle' after this round */
    
    sched_state = ELWB_SCHED_STATE_DATA;
    
  } else if (sched_state == ELWB_SCHED_STATE_DATA) {
    /* a data round has just finished */
    
    LOG_INFO("%lums round duration, %lu of %lu nodes requested %lu slots",
             SCHEDUNITS_TO_MS(t_round), req_nodes, n_nodes, req_slots);
    
    memset(sched->slot, 0, sizeof(sched->slot));        /* clear the content */
    /* reset all requests (set n_pkts to 0) */
    elwb_node_list_t* curr_node = head;
    uint32_t time_s = (time / ELWB_PERIOD_SCALE);
    uint32_t print_cnt = snprintf(print_buffer, ELWB_SCHED_PRINT_BUFFER_SIZE, "registered nodes: ");
    while (curr_node != 0) {
      /* remove old nodes */
      if ((time_s - curr_node->t_last_req) > ELWB_CONF_SCHED_NODE_TIMEOUT) {
        elwb_node_list_t* next = curr_node->next;
        elwb_sched_remove_node(curr_node);
        curr_node = next;
      } else {
        /* convert the node ID to a string and write it into the print buffer */
        if (print_cnt < (ELWB_SCHED_PRINT_BUFFER_SIZE - 6)) {
          print_cnt += uint16_to_str(curr_node->id, print_buffer + print_cnt);
          print_buffer[print_cnt++] = ' ';
          print_buffer[print_cnt]   = 0;
        }
        curr_node->n_pkts = 0;  /* reset */
        curr_node = curr_node->next;
      }
    }
    LOG_INFO_CONST(print_buffer);
    
    sched_state = ELWB_SCHED_STATE_IDLE;
    /* schedule for next round will be set below */
  }
  
  if (sched_state == ELWB_SCHED_STATE_IDLE) {
    /* regular idle round */
    /* add slots for the host if requested */
    n_slots_assigned = 0;
    while (reserve_slots_host && n_slots_assigned < ELWB_CONF_MAX_SLOTS_HOST) {
      sched->slot[n_slots_assigned++] = NODE_ID;
      reserve_slots_host--;
    }
    sched->n_slots = n_slots_assigned;
    /* calculate round duration (standard idle round + #slots for host) */
    t_round = ELWB_T_IDLE_ROUND + TICKS_TO_SCHEDUNITS(n_slots_assigned * 
                                      (ELWB_CONF_T_DATA + ELWB_CONF_T_GAP));
    sched->period = period;   /* assume idle period for next round */
    if (n_slots_assigned) {
      ELWB_SCHED_SET_DATA_SLOTS(sched);
    }
    ELWB_SCHED_SET_CONT_SLOT(sched);
    ELWB_SCHED_SET_STATE_IDLE(sched);  /* will be used by source nodes */
  }
  
  /* increment the timestamp */
  sched->time = time / ELWB_PERIOD_SCALE;
  
  uint32_t compressed_size;
#if ELWB_CONF_SCHED_COMPRESS
  compressed_size = elwb_sched_compress((uint8_t*)sched->slot, 
                                        n_slots_assigned);
  if ((compressed_size + ELWB_SCHED_HDR_LEN) > ELWB_CONF_MAX_PKT_LEN) {
    LOG_ERROR("compressed schedule is too big!");
  }
#else
  compressed_size = n_slots_assigned * 2;
#endif /* ELWB_CONF_SCHED_COMPRESS */
  
  compressed_size += ELWB_SCHED_HDR_LEN;  /* add header length */
  
#if ELWB_CONF_SCHED_CRC
  uint16_t crc = crc16((uint8_t*)sched, compressed_size, 0);
  memcpy((uint8_t*)sched + compressed_size, &crc, 2);
  compressed_size += 2;
#endif /* ELWB_CONF_SCHED_CRC */
     
  /* log the parameters of the new schedule */
  LOG_INFO("schedule updated (s=%lu T=%u0 n=%lu x=%u l=%lu)",
           n_nodes, sched->period,
           n_slots_assigned,
           sched->n_slots >> 13,
           compressed_size);

  return compressed_size;
}
/*---------------------------------------------------------------------------*/
uint32_t elwb_sched_init(elwb_schedule_t* sched)
{
  LOG_INFO("rounds [ms]: T=%u000 idle=%lu req=%lu data=%lu sum=%lu",
           ELWB_CONF_SCHED_PERIOD_IDLE,
           (uint32_t)SCHEDUNITS_TO_MS(ELWB_T_IDLE_ROUND),
           (uint32_t)SCHEDUNITS_TO_MS(ELWB_T_REQ_ROUND_MAX),
           (uint32_t)SCHEDUNITS_TO_MS(ELWB_T_DATA_ROUND_MAX),
           (uint32_t)SCHEDUNITS_TO_MS(ELWB_T_ROUND_MAX));

  /* make sure the minimal round period is not smaller than the max. round
   * duration! */
  if (((uint32_t)ELWB_CONF_SCHED_PERIOD_MIN * 1000) <=
     (SCHEDUNITS_TO_MS(ELWB_T_ROUND_MAX) + 
      ((uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_PREPROCESS)))) {
    LOG_ERROR_CONST("invalid parameters");
    return 0;
  }
  /* initialize node list */
  memset(node_list, 0, sizeof(elwb_node_list_t) * ELWB_CONF_MAX_N_NODES);
  head           = 0;
  n_nodes        = 0;
  time           = 0;
  period         = ELWB_CONF_SCHED_PERIOD_IDLE * ELWB_PERIOD_SCALE;
  sched_state    = ELWB_SCHED_STATE_IDLE;
  sched->n_slots = 0;
  sched->time    = 0;
  sched->period  = period;
  ELWB_SCHED_SET_CONT_SLOT(sched);              /* include a contention slot */
  ELWB_SCHED_SET_STATE_IDLE(sched);

  elwb_sched_register_nodes();

#if ELWB_CONF_SCHED_CRC
  uint16_t crc = crc16((uint8_t*)sched, ELWB_SCHED_HDR_LEN, 0);
  memcpy((uint8_t*)sched + ELWB_SCHED_HDR_LEN, &crc, 2);
  return ELWB_SCHED_HDR_LEN + 2;   /* empty schedule, no slots allocated yet */
#else
  return ELWB_SCHED_HDR_LEN;       /* empty schedule, no slots allocated yet */
#endif /* ELWB_CONF_SCHED_CRC */
}
/*---------------------------------------------------------------------------*/
void elwb_sched_register_nodes(void)
{
#ifdef ELWB_CONF_SCHED_NODE_LIST
  const uint16_t node_ids[] = { ELWB_CONF_SCHED_NODE_LIST };
  uint32_t cnt = sizeof(node_ids) / 2;
  uint32_t i;
  LOG_INFO("%u source nodes registered:", cnt);
  for (i = 0; i < cnt; i++) {
    elwb_sched_add_node(node_ids[i]);
  }
#endif /* ELWB_CONF_SCHED_NODE_LIST */
}
/*---------------------------------------------------------------------------*/
uint32_t elwb_sched_get_period(void)
{
  /* period in seconds */
  return period / ELWB_PERIOD_SCALE;
}
/*---------------------------------------------------------------------------*/
void elwb_sched_set_period(uint32_t p)
{
  if (p >= ELWB_CONF_SCHED_PERIOD_MIN && p <= ELWB_SCHED_PERIOD_MAX) {
    period = (uint16_t)((uint32_t)p * ELWB_PERIOD_SCALE);
  }
}
/*---------------------------------------------------------------------------*/
uint32_t elwb_sched_get_time(void)
{
  /* network time in seconds */
  return (time / ELWB_PERIOD_SCALE);
}
/*---------------------------------------------------------------------------*/
void elwb_sched_set_time(uint32_t new_time)
{
  time = (uint64_t)new_time * ELWB_PERIOD_SCALE;
}
/*---------------------------------------------------------------------------*/

#endif /* ELWB_ENABLE */
