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

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif /* MIN */
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif /* MAX */

linked_list_t* slwb_streams = NULL;
linked_list_t* lr_slwb_streams[8] = {0};
//static uint8_t n_slwb_streams = 0;
//static uint8_t n_unacked_streams = 0;
static uint8_t n_lr_stream_lists = 0;
static uint8_t n_unacked_lr_streams = 0;
static bool lr_schedule = false;

uint16_t slwb_round_period;
uint16_t slwb_round_idx;

void slwb_initialize_scheduler() {
  slwb_streams = ll_get_new_list(sizeof(slwb_stream_t));
}

void slwb_scheduler_calculate_round_schedule(slwb_round_t* round) {
  list_element_t* elem = NULL;
  slwb_stream_t* stream = NULL;
  slwb_network_t* network = slwb_get_network();
  slwb_round_schedule_t* schedule = round->round_schedule;
  uint8_t lr = slwb_round_idx % SLWB_LR_ROUND_MULT;
  uint8_t idx = 0;
  uint8_t ack_idx = 0;
  uint8_t slot_limit = 0;

  if (!lr_schedule && LR_SCHEDULE && slwb_round_idx >= network->n_clusters) {
    // only start with lr schedule after the nodes in the clusters are synchronized
    lr_schedule = true;
  }

  // general schedule parameters
  schedule->gen_schedule.round_period = slwb_round_period;
  schedule->gen_schedule.contention_slot = 1;
  schedule->gen_schedule.n_data_slots = 0;
  schedule->gen_schedule.n_acks = 0;

  if (lr < network->n_clusters && lr_schedule) {
    // additionally set long range parameters
    round->type = LONG_RANGE;
    slot_limit = slwb_scheduler_number_data_slots(round, true);
    slot_limit = MIN(slot_limit, SLWB_MAX_DATA_SLOTS);

    slwb_cluster_t* cluster = &(network->clusters[lr]);
    schedule->lr_schedule.lr_period = slwb_round_period * SLWB_LR_ROUND_MULT;
    schedule->lr_schedule.lr_cont = schedule->gen_schedule.contention_slot;
    schedule->lr_schedule.lr_ack = false;
    schedule->lr_schedule.lr_base = cluster->cluster_nodes->nodes[((slwb_round_idx/SLWB_LR_ROUND_MULT) - 1) % cluster->cluster_nodes->n_nodes];

    elem = ll_get_head(lr_slwb_streams[lr]);
    for (int i = 0; i < lr_slwb_streams[lr]->list_size; ++i) {
      stream = (slwb_stream_t*) elem->data;
      if (idx >= slot_limit) {
        break;
      }
      if (stream->stream_req.period && stream->backoff <= 0) {
        // allocate a slot for this stream
        schedule->slots[idx++] = stream->stream_req.node_id;
        // increase the backoff by the stream period
        stream->backoff += stream->stream_req.period;
      }
      if (n_unacked_lr_streams && !schedule->lr_schedule.lr_ack && !stream->acked) {
        schedule->lr_schedule.lr_ack = true;
        schedule->stream_acks[ack_idx].node_id = stream->stream_req.node_id;
        schedule->stream_acks[ack_idx].stream_id = stream->stream_req.stream_id;
        stream->acked = true;
        ack_idx++;
        n_unacked_lr_streams--;
      }
      elem = elem->next_element;
    }

    for (int i = 0; i < idx; ++i) {
      if (idx+i >= SLWB_MAX_DATA_SLOTS) {
        break;
      }
      schedule->slots[idx+i] = schedule->lr_schedule.lr_base;
    }

    schedule->lr_schedule.n_lr_data = idx;
    schedule->gen_schedule.n_data_slots += idx;
    idx *= 2;
  }
  else {
    round->type = NORMAL;
  }

  slot_limit = slwb_scheduler_number_data_slots(round, false) + idx;
  slot_limit = MIN(slot_limit, SLWB_MAX_DATA_SLOTS);

  elem = ll_get_head(slwb_streams);
  while (elem) {
    stream = (slwb_stream_t*) elem->data;
    sprintf(slwb_print_buffer, "%d ss: %d, %d, %d, %d", idx, stream->stream_req.node_id, stream->stream_req.stream_id, stream->stream_req.period, stream->backoff);
    print(2, slwb_print_buffer);
    // if stream not acked add ack to schedule
    if (!stream->acked && ack_idx < SLWB_MAX_STREAM_ACKS) {
      schedule->gen_schedule.n_acks++;
      schedule->stream_acks[ack_idx].node_id = stream->stream_req.node_id;
      schedule->stream_acks[ack_idx].stream_id = stream->stream_req.stream_id;
      stream->acked = true;
      ack_idx++;
    }

    if (idx >= slot_limit) {
      break;
    }
    else if (stream->stream_req.period && stream->backoff <= 0) {
      // allocate a slot for this stream
      schedule->slots[idx++] = stream->stream_req.node_id;
      schedule->gen_schedule.n_data_slots += 1;
      // set the backoff to the stream period
      stream->backoff += stream->stream_req.period;
    }

    elem = elem->next_element;
  }
}

/*
 * save received stream for later allocating
 * check if stream req is from a long range (sender != sr->node_id) node or not (sender == sr->node_id)
 * sender: the node_id of the node who sent the stream request to the base
 */
void slwb_scheduler_add_stream_req(slwb_stream_request_t* sr, uint8_t sender, bool acked) {
  list_element_t* elem = NULL;
  slwb_stream_t* stream = NULL;

  if (sr->node_id == sender) {
    // check if stream request already saved
    elem = ll_get_head(slwb_streams);
    while (elem) {
      stream = (slwb_stream_t*) elem->data;
      if (stream->stream_req.node_id == sr->node_id && stream->stream_req.stream_id == sr->stream_id) {
        stream->acked = acked;
        stream->stream_req.period = sr->period;
        stream->stream_req.size = sr->size;
        slwb_print_stream(2, &(stream->stream_req));
        return;
      }
      elem = elem->next_element;
    }

    // save new stream
    stream = (slwb_stream_t*) ll_get_new_element_tail(slwb_streams);
    memcpy(&(stream->stream_req), sr, sizeof(slwb_stream_request_t));
    stream->acked = acked;
    stream->backoff = stream->stream_req.period;
    slwb_print_stream(2, &(stream->stream_req));
  }
  else {
    // save node in network
    slwb_network_t* network = slwb_get_network();
    slwb_cluster_t* cluster;
    uint8_t cluster_id = 0;
    for (int i = 0; i < network->n_clusters; ++i) {
      cluster = &network->clusters[i];
      if (array_find((uint8_t*) cluster->cluster_nodes->nodes, cluster->cluster_nodes->n_nodes, sender) != 0xFF) {
        slwb_network_add_node(cluster, sr->node_id, LONG_RANGE_NODE);
        cluster_id = i;
        break;
      }
    }

    // check if list for this cluster exists
    if (lr_slwb_streams[cluster_id]) {
      // check if stream request already saved
      elem = ll_get_head(lr_slwb_streams[cluster_id]);
      while (elem) {
        stream = (slwb_stream_t*) elem->data;
        if (stream->stream_req.node_id == sr->node_id && stream->stream_req.stream_id == sr->stream_id) {
          if (stream->acked) {
            // ack didn't reach destination -> send again
            stream->acked = false;
            n_unacked_lr_streams++;
          }
          stream->stream_req.period = sr->period;
          stream->stream_req.size = sr->size;
          slwb_print_stream(2, &(stream->stream_req));
          return;
        }
        elem = elem->next_element;
      }
    }
    else {
      // get new list
      lr_slwb_streams[cluster_id] = ll_get_new_list(sizeof(slwb_stream_t));
      n_lr_stream_lists++;
    }


    // save new stream
    stream = (slwb_stream_t*) ll_get_new_element_tail(lr_slwb_streams[cluster_id]);
    memcpy(&(stream->stream_req), sr, sizeof(slwb_stream_request_t));
    stream->acked = false;
    stream->backoff = stream->stream_req.period;
    n_unacked_lr_streams++;

    slwb_print_stream(2, &(stream->stream_req));
    sprintf(slwb_print_buffer, "acked: %d", stream->acked);
    print(1, slwb_print_buffer);
  }
}

/*
 * calculate how many slots can be scheduled in this round
 * long_range: if true calculate amount of lr slots with corresponding normal data slot
 *          if false return amount of normal data slots (already scheduled lr slots are considered)
 */
uint8_t slwb_scheduler_number_data_slots(slwb_round_t* round, bool long_range) {
  uint64_t remaining_time = round->round_schedule->gen_schedule.round_period * HS_TIMER_FREQUENCY - 300 * HS_TIMER_FREQUENCY_MS;
  uint64_t time_per_data = 0;

  // reduce remaining time by the contention and schedule slot times
  const slwb_slot_times_t* slot_times = &slwb_slot_times[round->modulation];
  remaining_time -= slot_times->schedule_slot_time + slot_times->contention_slot_time;
  // reduce remaining time by the already scheduled data slots to relay the long range data
  remaining_time -= round->round_schedule->lr_schedule.n_lr_data * slot_times->data_slot_time;

  // set time per data packet to the normal data slot time
  time_per_data += slot_times->data_slot_time;

  if (round->type == LONG_RANGE) {
    // reduce remaining time by the contention and schedule slot times for long range
    slot_times = &slwb_slot_times[round->lr_mod];
    remaining_time -= slot_times->lr_schedule_slot_time + slot_times->lr_cont_slot_time;
    // reduce remaining time by the already scheduled long range data slots
    remaining_time -= round->round_schedule->lr_schedule.n_lr_data * slot_times->lr_data_slot_time;

    // if the number of lr data slots is requested add the time of a long range data slot to the time per data
    if (long_range) {
      time_per_data += slot_times->lr_data_slot_time;
    }
  }

  uint8_t slots = (uint8_t) (remaining_time / time_per_data);
  sprintf(slwb_print_buffer, "slots: %d", slots);
  print(2, slwb_print_buffer);

  return slots;
}


/*
 * remove all empty streams that have been acked
 * reduce the backoff for all other streams
 */
void slwb_scheduler_update_streams() {
  print(2, "rem srs");
  linked_list_t* list;

  // iterate throug all long range stream lists and the slwb_streams list
  for (int i = 0; i < n_lr_stream_lists+1; ++i) {
    if (i < n_lr_stream_lists) {
      list = lr_slwb_streams[i];
    }
    else {
      list = slwb_streams;
    }

    if (list->list_size) {
      slwb_stream_t* stream = NULL;
      list_element_t* elem = ll_get_head(list);

      // iterate through the elements except the first one
      while (elem->next_element) {
        stream = (slwb_stream_t*) elem->next_element->data;
        slwb_print_stream(2, &stream->stream_req);
        if (!stream->stream_req.period && stream->acked) {
          sprintf(slwb_print_buffer, "rm sr: %d, %d", stream->stream_req.node_id, stream->stream_req.stream_id);
          print(2, slwb_print_buffer);

          list_element_t* tmp = elem->next_element;
          elem->next_element = tmp->next_element;

          if (tmp == list->tail) {
            list->tail = elem;
          }

          free(tmp);
          list->list_size--;
        }
        else {
          // reduce backoff by the round period
          stream->backoff -= slwb_round_period;
        }
        elem = elem->next_element;
      }

      // check first element
      elem = ll_get_head(list);
      if (elem) {
        stream = (slwb_stream_t*) elem->data;
        if (!stream->stream_req.period && stream->acked) {
          sprintf(slwb_print_buffer, "rm sr: %d, %d", stream->stream_req.node_id, stream->stream_req.stream_id);
          print(2, slwb_print_buffer);
          list->head = elem->next_element;
          free(elem);
          list->list_size--;
        }
        else {
          // reduce backoff by the round period
          stream->backoff -= slwb_round_period;
        }
      }
    }
  }
}

#endif /* SLWB_ENABLE */
