/*
 * slwb_manager.c
 *
 *  Created on: Dec 6, 2018
 *      Author: kelmicha
 */

#include "flora_lib.h"

#if SLWB_ENABLE

static slwb_round_t* current_round;
static slwb_round_schedule_t* current_schedule;
static slwb_stream_t* current_stream;

uint8_t data_slot_idx;
uint8_t base_id = 0;

// flood
static gloria_message_t message = {0};
static gloria_flood_t slwb_flood = {.message = &message};

bool outstanding_ack = false;
bool node_synced = false;
bool allocated_stream = false;
uint8_t missed_schedules = 0;


/*
 * participate in slwb round
 */
void slwb_start_round(slwb_round_t* round) {
  sprintf(char_buff, "sr: %llu, %llu", hs_timer_get_current_timestamp(), round->round_start);
  print(2, char_buff);
  current_round = round;
  current_schedule = round->round_schedule;
  data_slot_idx = 0;
  if (!slwb_is_base()) {
    slwb_data_generator_reduce_backoffs();
  }

  slwb_flood.marker = current_round->round_start;
  if (!slwb_is_lr_node()) {
    slwb_schedule_slot();
  }
  else {
    slwb_flood.marker += slwb_slot_times[current_round->modulation].schedule_slot_time;
    slwb_lr_schedule_slot();
  }
}

/*
 * finish slwb round
 * tell the scheduler to update the streams
 * print important information
 */
void slwb_finish_round() {
  if (slwb_is_base()) {
    slwb_data_generator_print_msgs();
    slwb_scheduler_update_streams();
//    slwb_network_print(2, slwb_get_network());
  }
  else {
    slwb_adapt_timer();
    slwb_print_schedule(2, current_round);
  }
  slwb_data_generatior_print_stats();
  slwb_round_finished();
}



/*  ____  _       _     ____  _             _     _____                 _   _
 * / ___|| | ___ | |_  / ___|| |_ __ _ _ __| |_  |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
 * \___ \| |/ _ \| __| \___ \| __/ _` | '__| __| | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 *  ___) | | (_) | |_   ___) | || (_| | |  | |_  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
 * |____/|_|\___/ \__| |____/ \__\__,_|_|   \__| |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
 *
 */

/*
 * process schedule slot
 */
void slwb_schedule_slot() {
  print(1, "ss");

  if (slwb_is_base()) {
    message.header.dst = 0;
    message.header.sync = 1;

    // always copy the general schedule plus the normal data slots
    uint8_t copy_size = sizeof(slwb_gen_schedule_t) + current_schedule->gen_schedule.n_data_slots;
    // set start address to begin of schedule
    uint8_t* start_addr = (uint8_t*) current_schedule;

    if (current_round->type == NORMAL) {
      message.header.type = SLWB_ROUND_SCHEDULE;
      // increase start address to general schedule
      start_addr += sizeof(slwb_lr_schedule_t);
    }
    else if (current_round->type == LONG_RANGE){
      message.header.type = SLWB_LR_ROUND_SCHEDULE;
      // also copy the long range schedule and the long range slots
      copy_size += sizeof(slwb_lr_schedule_t) + current_schedule->lr_schedule.n_lr_data;
    }
    else {
      cli_log("Wrong round type in schedule!", "slwb_manager", CLI_LOG_LEVEL_ERROR);
    }

    memcpy(message.payload, start_addr, copy_size);
    slwb_flood.payload_size = copy_size;

    // get number of stream acks to send
    uint8_t n_acks = current_schedule->lr_schedule.lr_ack + current_schedule->gen_schedule.n_acks;
    if (n_acks) {
      // add stream acks to payload
      start_addr = message.payload + copy_size;
      copy_size = n_acks * sizeof(slwb_stream_ack_t);
      memcpy(start_addr, &(current_schedule->stream_acks), copy_size);
      slwb_flood.payload_size += copy_size;
    }

    slwb_set_flood_tx_defaults();
  }
  else {
    if (node_synced) {
      // if a schedule has already been received
      uint32_t drift_comp;
      if(slwb_timer_drift_comp_active()) {
        drift_comp = SLWB_SYNCED_DRIFT_PPM * (slwb_flood.marker - slwb_get_latest_sync()) / 1e6;
      }
      else {
        drift_comp = SLWB_UNSYNCED_DRIFT_PPM * (slwb_flood.marker - slwb_get_latest_sync()) / 1e6;
      }
      sprintf(char_buff, "dc: %lu", drift_comp);
      print(2, char_buff);
      slwb_flood.rx_timeout = slwb_slot_times[current_round->modulation].schedule_slot_time - SLWB_SLOT_OVERHEAD;
      slwb_flood.guard_time = drift_comp;
    }
    else {
      // listen forever
      slwb_flood.rx_timeout = 0;
    }
    slwb_flood.lp_listening = false;
    slwb_set_flood_rx_defaults();
  }

  slwb_set_flood_defaults(current_round->modulation, current_round->power_lvl, slwb_max_flood_slots[current_round->modulation],
      gloria_default_retransmissions[current_round->modulation], gloria_default_acks[current_round->modulation], 0);
  gloria_run_flood(&slwb_flood, &slwb_schedule_slot_callback);
}

/*
 * prepare for lr schedule slot
 */
void slwb_lr_schedule_slot() {
  print(1, "lrs");
  if (slwb_is_lr_base()) {
    message.header.type = SLWB_LR_SCHEDULE;
    message.header.dst = 0;
    message.header.sync = 1;

    // copy lr schedule
    memcpy(message.payload, &(current_schedule->lr_schedule), sizeof(slwb_lr_schedule_t));
    slwb_flood.payload_size = sizeof(slwb_lr_schedule_t);

    // copy lr data slots
    memcpy(message.payload + slwb_flood.payload_size, &current_schedule->slots, current_schedule->lr_schedule.n_lr_data);
    slwb_flood.payload_size += current_schedule->lr_schedule.n_lr_data;

    if (current_schedule->lr_schedule.lr_ack) {
      memcpy(message.payload + slwb_flood.payload_size, current_schedule->stream_acks, sizeof(slwb_stream_ack_t));
      slwb_flood.payload_size += sizeof(slwb_stream_ack_t);
    }

    slwb_set_flood_tx_defaults();
  }
  else {
    if (node_synced) {
      // if a schedule has already been received
      uint32_t drift_comp;
      if(slwb_timer_drift_comp_active()) {
        drift_comp = SLWB_SYNCED_DRIFT_PPM * (slwb_flood.marker - slwb_get_latest_sync()) / 1e6;
      }
      else {
        drift_comp = SLWB_UNSYNCED_DRIFT_PPM * (slwb_flood.marker - slwb_get_latest_sync()) / 1e6;
      }
      sprintf(char_buff, "dc: %lu", drift_comp);
      print(2, char_buff);
      slwb_flood.rx_timeout = slwb_slot_times[current_round->modulation].schedule_slot_time - SLWB_SLOT_OVERHEAD;
      slwb_flood.guard_time = drift_comp;
    }
    else {
      // listen forever
      slwb_flood.rx_timeout = 0;
    }
    slwb_flood.lp_listening = false;
    slwb_set_flood_rx_defaults();
  }

  slwb_set_flood_defaults(current_round->lr_mod, current_round->lr_pwr, 1, 1, 0, 0);
  gloria_run_flood(&slwb_flood, &slwb_lr_schedule_slot_callback);
}

/*
 * prepare for lr contention slot
 */
void slwb_lr_contention_slot() {
  print(1, "lrc");
  if (slwb_is_lr_base()) {
    slwb_flood.lp_listening = true;
    slwb_flood.guard_time = 0;
    slwb_flood.payload_size = sizeof(slwb_stream_request_t);
    message.header.sync = 0;
    slwb_set_flood_rx_defaults();
  }
  else {
    current_stream = slwb_data_generator_get_data_stream();
    if (current_stream) {
      outstanding_ack = true;

      message.header.type = SLWB_STREAM_REQUEST;
      message.header.dst = base_id;
      message.header.sync = 0;

      memcpy(&message.payload, &current_stream->stream_req, sizeof(slwb_stream_request_t));
      slwb_flood.payload_size = sizeof(slwb_stream_request_t);

      slwb_print_stream(1, &(current_stream->stream_req));

      slwb_set_flood_tx_defaults();
    }
    else {
      slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_cont_slot_time;
      slwb_flood.marker += slwb_slot_times[current_round->modulation].contention_slot_time;
      slwb_lr_data_slot();
      return;
    }
  }

  slwb_set_flood_defaults(current_round->lr_mod, current_round->lr_pwr, 1, 1, 0, 0);
  gloria_run_flood(&slwb_flood, &slwb_lr_contention_slot_callback);
}


/*
 * process contention slot
 */
void slwb_contention_slot() {
  if (current_schedule->gen_schedule.contention_slot) {
    print(1, "cs");

    current_stream = slwb_data_generator_get_data_stream();
    if (current_stream && !slwb_is_base() && !allocated_stream) {
      outstanding_ack = true;

      message.header.type = SLWB_STREAM_REQUEST;
      message.header.dst = base_id;
      message.header.sync = 0;

      memcpy(&message.payload, &current_stream->stream_req, sizeof(slwb_stream_request_t));
      slwb_flood.payload_size = sizeof(slwb_stream_request_t);

      slwb_print_stream(1, &(current_stream->stream_req));

      // if it is a stream requested from a long range node mark it as invalid (set node id to 0)
      if (current_stream->stream_req.node_id != slwb_get_id()) {
        current_stream->stream_req.node_id = 0;
      }

      slwb_set_flood_tx_defaults();
    }
    else {
      slwb_flood.lp_listening = true;
      slwb_flood.guard_time = 0;
      slwb_flood.payload_size = sizeof(slwb_stream_request_t);
      message.header.sync = 0;
      slwb_set_flood_rx_defaults();
    }

    slwb_set_flood_defaults(current_round->modulation, current_round->power_lvl, slwb_max_flood_slots[current_round->modulation],
      gloria_default_retransmissions[current_round->modulation], gloria_default_acks[current_round->modulation], 2);
    gloria_run_flood(&slwb_flood, &slwb_contention_slot_callback);
  }
  else {
    if (current_round->type == NORMAL) {
      slwb_data_slot();
    }
    else {
      if (slwb_is_lr_participant()) {
        slwb_lr_data_slot();
      }
      else {
        data_slot_idx = current_schedule->lr_schedule.n_lr_data;
        slwb_flood.marker += data_slot_idx * slwb_slot_times[current_round->lr_mod].lr_data_slot_time;
        slwb_data_slot();
      }
    }
  }
}

/*
 * prepare for lr data slot
 */
void slwb_lr_data_slot() {
  if (data_slot_idx < current_schedule->lr_schedule.n_lr_data) {
    print(1, "lrd");
    if (current_schedule->slots[data_slot_idx] == slwb_get_id()) {
      slwb_data_message_t* msg = slwb_data_generator_get_data();
      if (msg) {
        sprintf(char_buff, "lr_send: %d, %d", msg->node_id, msg->packet_id);
        print(10, char_buff);

        message.header.dst = base_id;
        message.header.sync = 0;

        memcpy(&message.payload, msg, sizeof(slwb_data_message_t));
        slwb_flood.payload_size = sizeof(slwb_data_message_t);

        // check for stream requests to piggy-back
        current_stream = slwb_data_generator_get_data_stream();
        if (current_stream) {
          message.header.type = SLWB_DATA_PLUS_SR;
          memcpy(message.payload + slwb_flood.payload_size, &current_stream->stream_req, sizeof(slwb_stream_request_t));
          slwb_flood.payload_size += sizeof(slwb_stream_request_t);
          current_stream->backoff = rand() % SLWB_LR_BACKOFF + 1;
        }
        else {
          message.header.type = SLWB_DATA_MESSAGE;
        }

        slwb_set_flood_tx_defaults();
      }
      else {
        data_slot_idx++;
        slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_data_slot_time;
        slwb_lr_data_slot();
        return;
      }
    }
    else if (slwb_is_lr_base()) {
      slwb_flood.lp_listening = true;
      slwb_flood.guard_time = 0;
      slwb_flood.payload_size = sizeof(slwb_data_message_t) + sizeof(slwb_stream_request_t);
      message.header.sync = 0;
      slwb_set_flood_rx_defaults();
    }
    else {
      data_slot_idx++;
      slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_data_slot_time;
      slwb_lr_data_slot();
      return;
    }

    slwb_set_flood_defaults(current_round->lr_mod, current_round->lr_pwr, 1, 1, 0, 0);
    gloria_run_flood(&slwb_flood, &slwb_lr_data_slot_callback);
  }
  else {
    slwb_data_slot();
  }
}


/*
 * process data slot
 */
void slwb_data_slot() {
  if (data_slot_idx < current_schedule->gen_schedule.n_data_slots + current_schedule->lr_schedule.n_lr_data) {
    print(1, "ds");
    if (current_schedule->slots[data_slot_idx] == slwb_get_id()) {
      slwb_data_message_t* msg = slwb_data_generator_get_data();
      if (msg) {
        sprintf(char_buff, "data_send: %d, %d", msg->node_id, msg->packet_id);
        print(10, char_buff);

        message.header.dst = base_id;
        message.header.sync = 0;

        memcpy(&message.payload, msg, sizeof(slwb_data_message_t));
        slwb_flood.payload_size = sizeof(slwb_data_message_t);

        // check for stream requests to piggy-back
        current_stream = slwb_data_generator_get_data_stream();
        if (current_stream) {
          message.header.type = SLWB_DATA_PLUS_SR;
          memcpy(message.payload + slwb_flood.payload_size, &current_stream->stream_req, sizeof(slwb_stream_request_t));
          slwb_flood.payload_size += sizeof(slwb_stream_request_t);
          current_stream->backoff = rand() % SLWB_BACKOFF + 1;

          // if it is a stream requested from a long range node mark it as invalid (set node id to 0)
          if (current_stream->stream_req.node_id != slwb_get_id()) {
            current_stream->stream_req.node_id = 0;
          }
        }
        else {
          message.header.type = SLWB_DATA_MESSAGE;
        }

        slwb_set_flood_tx_defaults();
      }
      else {
        print(2, "no msg");
        data_slot_idx++;
        slwb_flood.marker += slwb_slot_times[current_round->modulation].data_slot_time;
        slwb_data_slot();
        return;
      }
    }
    else {
      slwb_flood.rx_timeout = slwb_slot_times[current_round->modulation].data_slot_time - SLWB_SLOT_OVERHEAD;
      slwb_flood.lp_listening = false;
      slwb_flood.guard_time = 0;
      slwb_flood.payload_size = sizeof(slwb_data_message_t);
      message.header.sync = 0;
      slwb_set_flood_rx_defaults();
    }

    slwb_set_flood_defaults(current_round->modulation, current_round->power_lvl, slwb_max_flood_slots[current_round->modulation],
        gloria_default_retransmissions[current_round->modulation], gloria_default_acks[current_round->modulation], 0);
    gloria_run_flood(&slwb_flood, &slwb_data_slot_callback);
  }
  else {
    slwb_finish_round();
  }
}



/*   ____      _ _ _                _      _____                 _   _
 *  / ___|__ _| | | |__   __ _  ___| | __ |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
 * | |   / _` | | | '_ \ / _` |/ __| |/ / | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
 * | |__| (_| | | | |_) | (_| | (__|   <  |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
 *  \____\__,_|_|_|_.__/ \__,_|\___|_|\_\ |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
 */

/*
 * callback function for the schedule floods
 */
void slwb_schedule_slot_callback() {
  print(1, "ssc");
  sprintf(char_buff, "base: %d, rec: %d", slwb_is_base(), slwb_flood.msg_received);
  print(1, char_buff);
  if (!slwb_is_base()) {
    if (slwb_flood.msg_received) {

      uint8_t copy_size = sizeof(slwb_gen_schedule_t);
      uint8_t* start_addr = (uint8_t*) current_schedule;

      switch (message.header.type) {
        case SLWB_ROUND_SCHEDULE:
          current_round->type = NORMAL;
          // only copy the general schedule -> set start addr to schedule->gen_schedule
          start_addr += sizeof(slwb_lr_schedule_t);
          break;
        case SLWB_LR_ROUND_SCHEDULE:
          current_round->type = LONG_RANGE;
          // also copy the long range schedule
          copy_size += sizeof(slwb_lr_schedule_t);
          break;
        default:
          slwb_finish_round();
          return;
          break;
      }

      // get the parameters from the general schedule and the long range schedule if available
      memcpy(start_addr, message.payload, copy_size);

      // get the slots
      start_addr = message.payload + copy_size;
      copy_size = current_schedule->gen_schedule.n_data_slots + current_schedule->lr_schedule.n_lr_data;
      memcpy(&(current_schedule->slots), start_addr, copy_size);

      uint8_t n_acks = current_schedule->lr_schedule.lr_ack + current_schedule->gen_schedule.n_acks;
      // if there are acks appended get those
      if (n_acks) {
        start_addr += copy_size;
        copy_size = n_acks * sizeof(slwb_stream_ack_t);
        memcpy(&(current_schedule->stream_acks), start_addr, copy_size);
        for (int i = 0; i < n_acks; ++i) {
          slwb_stream_ack_t* sa = &current_schedule->stream_acks[i];
          if (sa->node_id == slwb_get_id()) {
            slwb_data_generator_stream_acked(sa->node_id, sa->stream_id);
          }
        }
      }

      base_id = message.header.src;
      current_round->round_start = slwb_flood.received_marker;

      // save the markers for synchronization
      slwb_save_markers(&slwb_flood);
      node_synced = true;

      // configure node as lr base if it is the chosen one for this round
      if (current_round->type == LONG_RANGE) {
        slwb_set_lr_base(current_schedule->lr_schedule.lr_base == slwb_get_id());
      }
    }
    else {
      if (node_synced) {
        if (++missed_schedules >= SLWB_MISSED_SCHEDULES) {
          node_synced = false;
          missed_schedules = 0;
          slwb_reset_timer();
        }
      }
      slwb_finish_round();
      return;
    }
  }

  slwb_flood.marker = slwb_flood.reconstructed_marker + slwb_slot_times[current_round->modulation].schedule_slot_time;

  if (current_round->type == NORMAL) {
    slwb_contention_slot();
  }
  else {
    if (slwb_is_lr_participant(current_schedule)) {
      slwb_lr_schedule_slot(current_round);
    }
    else {
      slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_schedule_slot_time;
      if (current_schedule->lr_schedule.lr_cont) {
        slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_cont_slot_time;
      }
      slwb_contention_slot();
    }
  }
}

/*
 * callback function for the lr schedule floods
 */
void slwb_lr_schedule_slot_callback() {
  print(1, "lrsc");
  if (!slwb_is_lr_base()) {
    if (slwb_flood.msg_received && slwb_flood.message->header.type == SLWB_LR_SCHEDULE) {
      current_round->type = LONG_RANGE;

      // copy lr schedule
      memcpy(&(current_schedule->lr_schedule), message.payload, sizeof(slwb_lr_schedule_t));

      // copy lr data slots
      memcpy(&current_schedule->slots, message.payload + sizeof(slwb_lr_schedule_t), current_schedule->lr_schedule.n_lr_data);


      if (current_schedule->lr_schedule.lr_ack) {
        // if schedule contains a stream ack
        slwb_stream_ack_t* ack = (slwb_stream_ack_t*) (message.payload + sizeof(slwb_lr_schedule_t) + current_schedule->lr_schedule.n_lr_data);

        if (ack->node_id == slwb_get_id()) {
          // if ack is for this node mark stream as acked
          slwb_data_generator_stream_acked(ack->node_id, ack->stream_id);
          allocated_stream = true;
        }
      }
      else if (outstanding_ack) {
        // stream request sent but no ack received
        current_stream->backoff = rand() % SLWB_LR_BACKOFF + 1;
        sprintf(char_buff, "bo: %d", current_stream->backoff);
        print(1, char_buff);
      }
      outstanding_ack = false;

      // save markers for synchronization
      slwb_save_markers(&slwb_flood);

      current_round->round_start = slwb_flood.received_marker - slwb_slot_times[current_round->modulation].schedule_slot_time;

      node_synced = true;
      missed_schedules = 0;
      base_id = current_schedule->lr_schedule.lr_base;
    }
    else {
      if (node_synced) {
        if (++missed_schedules > SLWB_MISSED_SCHEDULES) {
          node_synced = false;
          missed_schedules = 0;
          slwb_reset_timer();
        }
      }
      slwb_finish_round();
      return;
    }
  }

  slwb_flood.marker = slwb_flood.reconstructed_marker + slwb_slot_times[current_round->lr_mod].lr_schedule_slot_time;
  if (current_schedule->lr_schedule.lr_cont) {
    slwb_lr_contention_slot();
  }
  else {
    slwb_lr_data_slot();
  }
}

/*
 * callback function for the lr contention floods
 */
void slwb_lr_contention_slot_callback() {
  print(1, "lrcc");
  if (slwb_is_lr_base()) {
    if (slwb_flood.msg_received && message.header.type == SLWB_STREAM_REQUEST) {
      slwb_data_generator_add_stream((slwb_stream_request_t*) message.payload);
    }
  }

  slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_cont_slot_time;

  if (slwb_is_lr_node()) {
    if (current_schedule->lr_schedule.lr_cont) {
      slwb_flood.marker += slwb_slot_times[current_round->modulation].contention_slot_time;
    }
    slwb_lr_data_slot();
  }
  else {
    slwb_contention_slot();
  }
}


/*
 * callback function for the contention floods
 */
void slwb_contention_slot_callback() {
  if (slwb_is_base()) {
    if (slwb_flood.msg_received && message.header.type == SLWB_STREAM_REQUEST) {
      slwb_stream_request_t* sr = (slwb_stream_request_t*) message.payload;
      slwb_scheduler_add_stream_req(sr, message.header.src, true);
    }
  }
  else {
    if (slwb_flood.acked && slwb_flood.ack_message.dst == slwb_get_id()) {
      current_stream->acked = true;
      if (current_stream->stream_req.node_id == slwb_get_id()) {
        allocated_stream = true;
      }
    }
    else if (outstanding_ack) {
      // stream request sent but no ack received
      current_stream->backoff = rand() % SLWB_BACKOFF + 1;
      sprintf(char_buff, "bo: %d", current_stream->backoff);
      print(1, char_buff);
    }
    outstanding_ack = false;
  }

  slwb_flood.marker += slwb_slot_times[current_round->modulation].contention_slot_time;

  if (current_round->type == NORMAL) {
    slwb_data_slot();
  }
  else {
    if (slwb_is_lr_participant()) {
      slwb_lr_data_slot();
    }
    else {
      data_slot_idx = current_schedule->lr_schedule.n_lr_data;
      slwb_flood.marker += data_slot_idx * slwb_slot_times[current_round->lr_mod].lr_data_slot_time;
      slwb_data_slot();
    }
  }
}


/*
 * callback function for the long range data floods
 */
void slwb_lr_data_slot_callback() {
  print(1, "lrdc");
  if (slwb_is_lr_base()) {
    if (slwb_flood.msg_received) {
      switch (message.header.type) {
        case SLWB_DATA_MESSAGE:
          slwb_data_generator_add_data((slwb_data_message_t*) message.payload);
          break;
        case SLWB_DATA_PLUS_SR:
          slwb_data_generator_add_data((slwb_data_message_t*) message.payload);
          slwb_data_generator_add_stream((slwb_stream_request_t*) (message.payload + sizeof(slwb_data_message_t)));
          break;
        default:
          break;
      }
    }
  }
  else if (slwb_flood.initial) {
    slwb_data_generator_msg_sent();
  }

  slwb_flood.marker += slwb_slot_times[current_round->lr_mod].lr_data_slot_time;
  data_slot_idx++;
  slwb_lr_data_slot();
}


/*
 * callback function for the data floods
 */
void slwb_data_slot_callback() {
  print(1, "dsc");
  if (slwb_is_base()) {
    if (slwb_flood.msg_received) {
      switch (message.header.type) {
        case SLWB_DATA_MESSAGE:
          slwb_data_generator_add_data((slwb_data_message_t*) message.payload);
          break;
        case SLWB_DATA_PLUS_SR:
          slwb_data_generator_add_data((slwb_data_message_t*) message.payload);
          print(2, "pb");
          slwb_scheduler_add_stream_req((slwb_stream_request_t*) (message.payload + sizeof(slwb_data_message_t)), message.header.src, false);
          break;
        default:
          break;
      }
    }
  }
  else if (slwb_flood.initial) {
    slwb_data_generator_msg_sent();
  }

  data_slot_idx++;
  slwb_flood.marker += slwb_slot_times[current_round->modulation].data_slot_time;
  slwb_data_slot();
}


/*  _____ _                 _   ____       _   _   _
 * |  ___| | ___   ___   __| | / ___|  ___| |_| |_(_)_ __   __ _ ___
 * | |_  | |/ _ \ / _ \ / _` | \___ \ / _ \ __| __| | '_ \ / _` / __|
 * |  _| | | (_) | (_) | (_| |  ___) |  __/ |_| |_| | | | | (_| \__ \
 * |_|   |_|\___/ \___/ \__,_| |____/ \___|\__|\__|_|_| |_|\__, |___/
 *                                                         |___/
 */

/*
 * set flood settings, independent if node is initiator or not
 */
void slwb_set_flood_defaults(uint8_t modulation, int8_t power, uint8_t slots, uint8_t retransmissions, uint8_t acks, uint8_t ack_mode) {
  slwb_flood.band = RADIO_DEFAULT_BAND;
  slwb_flood.modulation = modulation;
  slwb_flood.power = power;
  slwb_flood.data_slots = slots;
  slwb_flood.max_retransmissions = retransmissions;
  slwb_flood.max_acks = acks;
  slwb_flood.ack_mode = ack_mode;
}


/*
 * set the default values before sending a flood
 */
void slwb_set_flood_tx_defaults() {
  slwb_flood.initial = true;
}


/*
 * set the default values to start listening for a flood
 */
void slwb_set_flood_rx_defaults() {
  slwb_flood.initial = false;
  slwb_flood.sync_timer = false;
}

#endif /* SLWB_ENABLE */
