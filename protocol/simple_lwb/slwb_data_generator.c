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

linked_list_t* data_streams = NULL;
linked_list_t* data_queue = NULL;
uint16_t packet_id = 0;      // packet counter

slwb_stream_t lr_stream = {0};

uint16_t slwb_round_period;
uint16_t slwb_round_idx;

uint8_t data_period;
bool backpress_stream = false;

/*
 * initialize the stream and data queues
 * generate first stream request and first data packet
 */
void slwb_data_generator_init(uint8_t period) {
  data_streams = ll_get_new_list(sizeof(slwb_stream_t));
  data_queue = ll_get_new_list(sizeof(slwb_data_message_t));
  data_period = period;

  if (!slwb_is_base()) {
    slwb_data_generator_new_stream(0, period);
    slwb_data_generator_generate_data();
  }
}

/*
 * return oldest stream request that was not yet acked
 * if a long range stream request was received it has priority
 */
slwb_stream_t* slwb_data_generator_get_data_stream() {
  // return lr stream if it is valid
  if (lr_stream.stream_req.node_id) {
    return &lr_stream;
  }
  else {
    // iterate through the streams
    list_element_t* elem = ll_get_head(data_streams);
    slwb_stream_t* stream;
    while (elem) {
      stream = (slwb_stream_t*) elem->data;
      if (!stream->acked && !stream->backoff) {
          return stream;
      }
      elem = elem->next_element;
    }
  }
  return NULL;
}

/*
 * mark stream given by node_id and stream_id as acked
 */
void slwb_data_generator_stream_acked(uint8_t node_id, uint8_t stream_id) {
  list_element_t* elem = ll_get_head(data_streams);
  slwb_stream_t* stream;

  // iterate through the streams
  while (elem) {
    stream = (slwb_stream_t*) elem->data;
    // check if the correct stream is found
    if (stream->stream_req.node_id == node_id && stream->stream_req.stream_id == stream_id) {
      stream->acked = true;
      sprintf(slwb_print_buffer, "stream_acked: %d, %d", node_id, stream_id);
      print(2, slwb_print_buffer);
      return;
    }
    elem = elem->next_element;
  }
}

/*
 * set a contention backoff for a stream given by the node_id and the stream_id
 */
void slwb_data_generator_set_backoff(uint8_t node_id, uint8_t stream_id, uint8_t backoff) {
  list_element_t* elem = ll_get_head(data_streams);
  slwb_stream_t* stream;

  // iterate through the streams
  while (elem) {
    stream = (slwb_stream_t*) elem->data;
    // check if the correct stream is found
    if (stream->stream_req.node_id == node_id && stream->stream_req.stream_id == stream_id) {
      stream->backoff = backoff;
      return;
    }
    elem = elem->next_element;
  }
}

/*
 * reduce the backoff for all streams that are not yet acked
 */
void slwb_data_generator_reduce_backoffs() {
  list_element_t* elem = ll_get_head(data_streams);
  slwb_stream_t* stream;

  // iterate through the streams
  while (elem) {
    stream = (slwb_stream_t*) elem->data;
    if (!stream->acked && stream->backoff) {
      stream->backoff--;
    }
    elem = elem->next_element;
  }
}

/*
 * generate stream request and add it to queue
 */
void slwb_data_generator_new_stream(uint8_t id, uint8_t period) {
  slwb_stream_t* new_stream = ll_get_new_element_tail(data_streams);
  new_stream->acked = 0;
  new_stream->backoff = 0;
  new_stream->stream_req.node_id = slwb_get_id();
  new_stream->stream_req.size = 16;
  new_stream->stream_req.stream_id = id;
  new_stream->stream_req.period = period;

  slwb_print_stream(2, &new_stream->stream_req);
}

/*
 * add stream request to queue
 * if the stream already exists, it gets updated
 */
void slwb_data_generator_add_stream(slwb_stream_request_t* sr) {
  // check if the stream request is from a lr node
  if (sr->node_id != slwb_get_id()) {
    lr_stream.acked = 0;
    memcpy(&(lr_stream.stream_req), sr, sizeof(slwb_stream_request_t));
  }
  else {
    list_element_t* elem = ll_get_head(data_streams);
    slwb_stream_t* stream = NULL;

    // check if stream already exists
    while (elem) {
      stream = (slwb_stream_t*) elem->data;
      if (stream->stream_req.stream_id == sr->stream_id && stream->stream_req.node_id == sr->node_id) {
        // update stream
        stream->acked = false;
        stream->backoff = 0;
        memcpy(&(stream->stream_req), sr, sizeof(slwb_stream_request_t));
        slwb_print_stream(2, sr);
        return;
      }
      elem = elem->next_element;
    }

    // add new stream
    stream = ll_get_new_element_tail(data_streams);
    stream->acked = false;
    stream->backoff = 0;
    memcpy(&(stream->stream_req), sr, sizeof(slwb_stream_request_t));
    slwb_print_stream(2, sr);
  }
}

/*
 * generate new data msg and add it to queue
 * gnerate a new data stream if the queue size is to big
 * remove second stream if data queue size is ok again
 */
void slwb_data_generator_generate_data() {
  // set timer for next data generation
  hs_timer_generic(hs_timer_get_current_timestamp() + data_period*HS_TIMER_FREQUENCY, &slwb_data_generator_generate_data);

  if (!slwb_is_base()) {
    slwb_data_message_t* data_msg = ll_get_new_element_tail(data_queue);
    // fill data payload with a sequence constructed from the node_id and the packet_id
    for (int i = 0; i < SLWB_MAX_DATA_PAYLOAD; ++i) {
      data_msg->payload[i] = packet_id*slwb_get_id() + i;
    }
    data_msg->node_id = slwb_get_id();
    data_msg->packet_id = packet_id;
    sprintf(slwb_print_buffer, "data_gen: %d, %d", slwb_get_id(), packet_id);
    print(10, slwb_print_buffer);
    packet_id++;

    // discard oldest data if list size is too large
    if (data_queue->list_size > SLWB_DATA_QUEUE_SIZE) {
      ll_free_head(data_queue);
    }

    // generate new stream if more than 2 messages in queue
    if (data_queue->list_size > SLWB_BACKLOG_HIGHER_LIMIT && !backpress_stream) {
      slwb_stream_request_t sr;
      sr.node_id = slwb_get_id();
      sr.stream_id = 1;
      sr.size = 16;
      sr.period = slwb_round_period * (slwb_is_lr_node()? SLWB_LR_ROUND_MULT:1);
      slwb_data_generator_add_stream(&sr);
      backpress_stream = true;
    }
    else if (data_queue->list_size < SLWB_BACKLOG_LOWER_LIMIT && backpress_stream) {
      // remove back pressure stream if queue size is below 2
      slwb_stream_request_t sr;
      sr.node_id = slwb_get_id();
      sr.stream_id = 1;
      sr.size = 0;
      sr.period = 0;
      slwb_data_generator_add_stream(&sr);
      backpress_stream = false;
    }
  }
}

/*
 * add data msg to queue
 * if the queue is already full, discard the oldest entry
 * long range data is appended to the head of the queue as it has higher priority
 */
void slwb_data_generator_add_data(slwb_data_message_t* msg) {
  // discard oldest data if list size is too large
  if (data_queue->list_size >= SLWB_DATA_QUEUE_SIZE) {
    ll_free_head(data_queue);
  }

  slwb_data_message_t* data_msg;
  if (msg->node_id == slwb_get_id()) {
    // add own data at the end of the queue
    data_msg = ll_get_new_element_tail(data_queue);
  }
  else {
    // add data from long range nodes at the beginning of the queue to relay it in the same round
    data_msg = ll_get_new_element_head(data_queue);
  }

  memcpy(data_msg, msg, sizeof(slwb_data_message_t));

  sprintf(slwb_print_buffer, "data_add: %d, %d", msg->node_id, msg->packet_id);
  print(1, slwb_print_buffer);
}

/*
 * get the current msg to send
 */
slwb_data_message_t* slwb_data_generator_get_data() {
  return ll_get_head_data(data_queue);
}

/*
 * remove sent message from data queue
 */
void slwb_data_generator_msg_sent() {
  ll_free_head(data_queue);
}

/*
 * print messages in queue
 * printed messages are removed from queue
 */
void slwb_data_generator_print_msgs() {
  list_element_t* elem = ll_get_head(data_queue);
  // iterate through the streams
  while (elem) {
    slwb_print_data_msg((slwb_data_message_t*) elem->data);
    ll_free_head(data_queue);
    elem = ll_get_head(data_queue);
  }
}

/*
 * print queue size, number of acked streams and round idx in json format
 */
void slwb_data_generatior_print_stats() {
  cJSON* slwb_stats = cJSON_CreateObject();

  if (cJSON_AddStringToObject(slwb_stats, "type", "data_stats") == NULL) {
    return;
  }

  if (cJSON_AddNumberToObject(slwb_stats, "data_queue", data_queue->list_size) == NULL) {
    return;
  }

  uint8_t stream_count = 0;
  uint8_t unacked_streams = 0;
  list_element_t* elem = ll_get_head(data_streams);
  // iterate through the streams
  while (elem) {
    slwb_stream_t* stream = (slwb_stream_t*) elem->data;
    if (stream->stream_req.period) {
      stream_count++;
    }
    if (!stream->acked) {
      unacked_streams++;
    }
    elem = elem->next_element;
  }


  if (cJSON_AddNumberToObject(slwb_stats, "streams", stream_count) == NULL) {
    return;
  }
  if (cJSON_AddNumberToObject(slwb_stats, "unacked_streams", unacked_streams) == NULL) {
    return;
  }

  if (cJSON_AddNumberToObject(slwb_stats, "round", slwb_round_idx) == NULL) {
    return;
  }

  cli_log_json(slwb_stats, "slwb", CLI_LOG_LEVEL_DEBUG);
}

#endif /* SLWB_ENABLE */
