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

#if ELWB_ENABLE

/* internal sync state of the eLWB */
typedef enum {
  BOOTSTRAP = 0,
  SYNCED,
  UNSYNCED,
  UNSYNCED2,
  NUM_OF_SYNC_STATES
} elwb_syncstate_t;


typedef enum {
  EVT_SCHED_RCVD = 0,
  EVT_SCHED_MISSED,
  NUM_OF_SYNC_EVENTS
} sync_event_t;


static const elwb_syncstate_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] =
{/* STATES:                                         EVENTS:         */
 /* BOOTSTRAP, SYNCED,   UNSYNCED,  UNSYNCED2                       */
  { SYNCED,    SYNCED,   SYNCED,    SYNCED    }, /* schedule rcvd   */
  { BOOTSTRAP, UNSYNCED, UNSYNCED2, BOOTSTRAP }  /* schedule missed */
};
static const char* elwb_syncstate_to_string[NUM_OF_SYNC_STATES] = {
  "BOOTSTRAP", "SYN", "USYN", "USYN2"
};


static elwb_stats_t       stats;
static elwb_time_t        last_synced;
static elwb_time_t        network_time;
static void*              task_handle  = 0;
static void*              post_task    = 0;
static void*              pre_task     = 0;
static void*              rx_queue     = 0;
static void*              tx_queue     = 0;
static void*              re_tx_queue  = 0;
static bool               elwb_running = false;
static bool               is_host      = 0;
static elwb_timeout_cb_t  timeout_cb   = 0;
static elwb_slot_cb_t     slot_cb      = 0;
static elwb_schedule_t    schedule;
static uint_fast8_t       schedule_len;
static elwb_packet_t      packet;             /* packet buffer */

static uint8_t            n_tx      = ELWB_CONF_N_TX;
static uint8_t            num_hops  = ELWB_CONF_NUM_HOPS;
uint32_t                  t_sched   = 0;      /* slot length for schedule packets */
uint32_t                  t_data    = 0;      /* slot length for data packets */
uint32_t                  t_cont    = 0;      /* slot length for contention, request and sched2 packets */
uint32_t                  t_dack    = 0;      /* slot length for data ACKs */

/* variables specific to the host node */
#if ELWB_CONF_DATA_ACK
static uint8_t            data_ack[(ELWB_CONF_MAX_DATA_SLOTS + 7) / 8] = { 0 };
static uint_fast16_t      my_slots;
#endif /* ELWB_CONF_DATA_ACK */

/* variables specific to a source node */
#if ELWB_CONF_CONT_USE_HSTIMER
static elwb_time_t        last_synced_hs;
#endif /* ELWB_CONF_CONT_USE_HSTIMER */
static elwb_syncstate_t   sync_state;
static uint_fast16_t      period_idle;        /* last known base period */
static uint_fast8_t       rand_backoff;

/* private (not exposed) scheduler functions */
uint32_t elwb_sched_init(elwb_schedule_t* sched);
void     elwb_sched_process_req(uint16_t id, uint32_t n_pkts);
uint32_t elwb_sched_compute(elwb_schedule_t * const sched, uint32_t reserve_slots_host);
bool     elwb_sched_uncompress(uint8_t* compressed_data, uint32_t n_slots);
void     elwb_sched_set_time_offset(uint32_t ofs);


/*
 * This function can be called from an interrupt context to poll the GMW task.
 */
void elwb_notify(void)
{
  if (task_handle) {
    if (xTaskGetCurrentTaskHandle() == task_handle) {
      LOG_WARNING("elwb_notify(): task is already running");
      if (post_task) {
        ELWB_TASK_NOTIFY(post_task);
      }
    }
    ELWB_ON_WAKEUP();
    ELWB_TASK_NOTIFY_FROM_ISR(task_handle);
  }
}


void elwb_wait_until(elwb_time_t timeout)
{
  if (elwb_running) {
    ELWB_TIMER_SET(timeout, elwb_notify);
    ELWB_SUSPENDED();
    ELWB_TASK_YIELD();
    ELWB_RESUMED();
  }
}


void elwb_schedule_received_callback(void)
{
  ELWB_TIMER_SET(0, 0);   /* cancel timer */
  elwb_notify();
}


const elwb_stats_t * const elwb_get_stats(void)
{
  return &stats;
}


/* returns the max. round duration in ticks, including the time allocated for the pre-process task */
uint32_t elwb_get_max_round_duration(uint32_t t_sched_arg, uint32_t t_cont_arg, uint32_t t_data_arg)
{
  if (!t_sched_arg) {
    t_sched_arg = t_sched;
  }
  if (!t_cont_arg) {
    t_cont_arg = t_cont;
  }
  if (!t_data_arg) {
    t_data_arg = t_data;
  }
  uint32_t t_round_max = (t_sched_arg + 2 * t_cont_arg + 2 * ELWB_CONF_T_GAP + ELWB_CONF_SCHED_COMP_TIME) +                                   /* idle round */
                         (t_sched_arg + ELWB_CONF_T_GAP + ELWB_CONF_MAX_NODES * (t_cont_arg + ELWB_CONF_T_GAP) + ELWB_CONF_SCHED_COMP_TIME) + /* request round */
                         (t_sched_arg + ELWB_CONF_T_GAP + ELWB_CONF_MAX_DATA_SLOTS * (t_data_arg + ELWB_CONF_T_GAP)) +                        /* data round */
                         2 * ELWB_CONF_SCHED_COMP_TIME +                                                                                      /* schedule computation time */
                         ELWB_CONF_T_PREPROCESS;                                                                                              /* execution of the pre-process task */
#if ELWB_CONF_DATA_ACK
  t_round_max += (t_data_arg + ELWB_CONF_T_GAP);
#endif /* ELWB_CONF_DATA_ACK */
  return t_round_max;
}


void elwb_get_last_syncpoint(elwb_time_t* time, elwb_time_t* rx_timestamp)
{
  if (time) {
    *time = network_time;
  }
  if (rx_timestamp) {
    *rx_timestamp = last_synced;
  }
}


/* if argument is given, converts the timestamp (in elwb timer ticks) to global network time
 * returns the current network time if no argument is given */
elwb_time_t elwb_get_time(const uint64_t* timestamp)
{
  uint64_t ts;
  if (timestamp) {
    ts = *timestamp;
  } else {
    ts = ELWB_TIMER_NOW();
  }
  return network_time + ((int64_t)ts - (int64_t)last_synced) * (1000000 - stats.drift) / (ELWB_TIMER_FREQUENCY);
}

uint32_t elwb_get_time_sec(void)
{
  return elwb_get_time(0) / 1000000;
}


void elwb_set_drift(int32_t drift_ppm)
{
  stats.drift = drift_ppm;
}


void elwb_register_slot_callback(elwb_slot_cb_t cb)
{
  slot_cb = cb;
}


bool elwb_update_slot_durations(uint8_t n_tx_arg, uint8_t num_hops_arg)
{
  if (!n_tx_arg) {
    n_tx_arg = n_tx;
  }
  if (!num_hops_arg) {
    num_hops_arg = num_hops;
  }
  uint32_t t_sched_new = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, (ELWB_CONF_MAX_DATA_SLOTS * sizeof(uint16_t) + ELWB_SCHED_HDR_LEN + ELWB_SCHED_CRC_LEN));
  uint32_t t_data_new  = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, ELWB_CONF_MAX_PAYLOAD_LEN);
  uint32_t t_cont_new  = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, MAX(ELWB_REQ_PKT_LEN, ELWB_2ND_SCHED_LEN) + ELWB_PKT_HDR_LEN);
  uint32_t t_dack_new  = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8 + ELWB_PKT_HDR_LEN);

  if (is_host && !elwb_sched_check_params(0, t_sched_new, t_cont_new, t_data_new)) {
    return false;
  }
  t_sched = t_sched_new;
  t_data  = t_data_new;
  t_cont  = t_cont_new;
  t_dack  = t_dack_new;

  return true;
}


uint8_t elwb_get_n_tx(void)
{
  return n_tx;
}


bool elwb_set_n_tx(uint8_t n_tx_arg)
{
  if (elwb_update_slot_durations(n_tx_arg, 0)) {
    n_tx = n_tx_arg;
    return true;
  }
  return false;
}


uint8_t elwb_get_num_hops(void)
{
  return num_hops;
}


bool elwb_set_num_hops(uint8_t num_hops_arg)
{
  if (elwb_update_slot_durations(0, num_hops_arg)) {
    num_hops = num_hops_arg;
    return true;
  }
  return false;
}


static void elwb_update_rssi_snr(void)
{
  int32_t rssi_curr = gloria_get_rssi();
  if (rssi_curr != 0) {
    if (stats.rssi_avg != 0) {
      stats.rssi_avg = (stats.rssi_avg + rssi_curr) / 2;  /* update the average RSSI value */
    } else {
      stats.rssi_avg = rssi_curr;
    }
  }
  int32_t snr_curr = gloria_get_snr();
  if (snr_curr != 0) {
    if (stats.snr_avg != 0) {
      stats.snr_avg = (stats.snr_avg + snr_curr) / 2;     /* update the average SNR value */
    } else {
      stats.snr_avg = snr_curr;
    }
  }
}


static bool elwb_bootstrap_sched_pkt_filter(uint8_t* pkt, uint8_t len)
{
  if ((len > ELWB_PKT_HDR_LEN) && ELWB_IS_PKT_HEADER_VALID((elwb_packet_t*)pkt) && ELWB_IS_SCHEDULE_PACKET((elwb_packet_t*)pkt) && ELWB_SCHED_IS_FIRST((elwb_schedule_t*)pkt)) {
    return true;
  }
  return false;
}

static bool elwb_is_schedule_valid(elwb_schedule_t* schedule)
{
  if (!gloria_is_t_ref_updated() || !ELWB_IS_PKT_HEADER_VALID(schedule) || !ELWB_IS_SCHEDULE_PACKET(schedule) || (gloria_get_payload_len() < (ELWB_SCHED_HDR_LEN + ELWB_SCHED_CRC_LEN))) {
    return false;
  }
  return true;
}


static void elwb_bootstrap(void)
{
  schedule.n_slots = 0;
  stats.bootstrap_cnt++;
  LOG_INFO("bootstrap");
  elwb_time_t bootstrap_timeout = ELWB_TIMER_NOW() + ELWB_CONF_BOOTSTRAP_TIMEOUT;
  /* keep listening until we receive a valid schedule packet */
  do {
    gloria_register_flood_callback(elwb_schedule_received_callback);
    gloria_set_pkt_filter(elwb_bootstrap_sched_pkt_filter);
    gloria_start(false, (uint8_t*)&schedule, 0, n_tx, 1);
    elwb_wait_until(ELWB_TIMER_NOW() + t_sched);
    gloria_stop();
    if (ELWB_TIMER_NOW() > bootstrap_timeout) {
      /* go to sleep for ELWB_CONF_T_DEEPSLEEP ticks */
      stats.sleep_cnt++;
      LOG_WARNING("timeout");
      /* poll the post process */
      if (post_task) {
        ELWB_TASK_NOTIFY(post_task);
      }
      if (timeout_cb) {
        timeout_cb();
      }
      elwb_wait_until(ELWB_TIMER_NOW() + ELWB_CONF_T_DEEPSLEEP);
      bootstrap_timeout = ELWB_TIMER_NOW() + ELWB_CONF_BOOTSTRAP_TIMEOUT;
    }
  } while (elwb_running && (!elwb_is_schedule_valid(&schedule) || !ELWB_SCHED_IS_FIRST(&schedule)));

  if (slot_cb) {
    slot_cb(schedule.host_id, ELWB_PHASE_SCHED1, (elwb_packet_t*)&schedule);
  }
}


static void elwb_send_schedule(elwb_time_t start_of_round)
{
  gloria_start(true, (uint8_t*)&schedule, schedule_len, n_tx, 1);
  elwb_wait_until(start_of_round + t_sched);
  gloria_stop();
  stats.pkt_tx_all++;

  /* update sync point */
  if (ELWB_SCHED_IS_FIRST(&schedule)) {
    network_time = schedule.time;
    last_synced  = start_of_round;
    /* calculate the reference offset for the source nodes (time between start_of_round and the tx marker) */
    stats.ref_ofs = (stats.ref_ofs + (gloria_get_t_ref() - start_of_round)) / 2;
    elwb_sched_set_time_offset(stats.ref_ofs);
  }

  if (slot_cb) {
    slot_cb(schedule.host_id, ELWB_PHASE_SCHED1, (elwb_packet_t*)&schedule);
  }
}


static void elwb_receive_schedule(elwb_time_t start_of_round)
{
  gloria_start(false, (uint8_t*)&schedule, 0, n_tx, 1);
  elwb_wait_until(start_of_round + t_sched + ELWB_CONF_T_GUARD_ROUND);
  gloria_stop();

  if (slot_cb) {
    slot_cb(schedule.host_id, ELWB_PHASE_SCHED1, (elwb_packet_t*)&schedule);
  }
}


static elwb_time_t elwb_sync(elwb_time_t start_of_round, bool expected_first_sched)
{
  elwb_time_t t_ref = 0;

  /* valid schedule received? */
  if (elwb_is_schedule_valid(&schedule)) {

#if ELWB_CONF_SCHED_CRC
    uint8_t packet_len = gloria_get_payload_len();
    /* check the CRC */
    uint16_t pkt_crc = ((uint16_t)*((uint8_t*)&schedule + packet_len - 1)) << 8 |
                       *((uint8_t*)&schedule + packet_len - 2);
    if (crc16((uint8_t*)&schedule, packet_len - 2, 0) != pkt_crc) {
      /* not supposed to happen => go back to bootstrap */
      LOG_ERROR("invalid CRC for eLWB schedule");
      sync_state = BOOTSTRAP;
      return 0;
    }
#endif /* ELWB_CONF_SCHED_CRC */

    /* schedule sanity check (#slots mustn't exceed the compile-time fixed max. # slots!) */
    if (ELWB_SCHED_N_SLOTS(&schedule) > ELWB_CONF_MAX_DATA_SLOTS) {
      LOG_ERROR("n_slots exceeds limit!");
      ELWB_SCHED_CLR_SLOTS(&schedule);
      schedule.n_slots += ELWB_CONF_MAX_DATA_SLOTS;
    }
    /* update the sync state machine */
    sync_state = next_state[EVT_SCHED_RCVD][sync_state];

    if (ELWB_SCHED_IS_FIRST(&schedule)) {
      t_ref = gloria_get_t_ref();
      /* do some basic drift estimation:
       * measured elapsed time minus effective elapsed time (given by host) */
      int64_t elapsed_network_us = (schedule.time - network_time);
      int64_t elapsed_local_us   = (t_ref - last_synced) * 1000000 / ELWB_TIMER_FREQUENCY;
      int32_t delta_us           = (elapsed_local_us - elapsed_network_us);
      /* now scale the difference from ticks to ppm (note: a negative drift means the local clock runs slower than the network clock) */
      int32_t drift_ppm = (delta_us * 1000) / (elapsed_network_us / 1000);
      if (drift_ppm < ELWB_CONF_MAX_CLOCK_DRIFT &&
          drift_ppm > -ELWB_CONF_MAX_CLOCK_DRIFT) {
        stats.drift = (stats.drift + drift_ppm) / 2;
      }
      /* only update the timestamp during the idle period */
      period_idle  = schedule.period;
      network_time = schedule.time;
      last_synced  = t_ref;
#if ELWB_CONF_CONT_USE_HSTIMER
      /* also store the HF timestamp in case LF is used for slot wakeups */
      last_synced_hs = gloria_get_t_ref_hs();
#endif /* ELWB_CONF_CONT_USE_HSTIMER */

    } else {
      /* just use the previous wakeup time as start time */
      t_ref = start_of_round + ELWB_CONF_T_GUARD_ROUND;
    }
    /* update stats */
    elwb_update_rssi_snr();
    stats.pkt_rx_all++;

  } else {
    /* update the sync state machine */
    sync_state = next_state[EVT_SCHED_MISSED][sync_state];

    if (sync_state != BOOTSTRAP) {
      stats.unsynced_cnt++;
      LOG_WARNING("schedule missed");

      /* we can only estimate t_ref */
      if (!expected_first_sched) {
        /* missed schedule was during a contention/data round */
        t_ref = last_synced;
        LOG_INFO("start_of_round restored from last_synced (%llu)", last_synced);
      } else {
        /* missed schedule is at beginning of a round */
        t_ref = start_of_round;
      }
      schedule.period = period_idle;        /* reset period to idle period */
      ELWB_SCHED_SET_STATE_IDLE(&schedule); /* mark as 'idle state' such that other processes can run */
    }
  }

  return t_ref;
}


static void elwb_send_packet(elwb_time_t slot_start, uint32_t slot_length, uint32_t slot_idx)
{
  bool data_packet = ELWB_SCHED_HAS_DATA_SLOTS(&schedule);

  /* send a data packet (if there is any) */
  if (ELWB_QUEUE_SIZE(tx_queue) > 0) {
    uint8_t packet_len;
    /* request round? -> only relevant for the source node */
    if (!is_host && !data_packet) {
      packet_len = ELWB_REQ_PKT_LEN;
      /* request as many data slots as there are packets in the queue */
      packet.req.num_slots = ELWB_QUEUE_SIZE(tx_queue);
    } else {
      /* prepare a data packet for dissemination */
      packet_len = 0;
      if (ELWB_QUEUE_POP(tx_queue, packet.payload)) {
        packet_len = ELWB_PAYLOAD_LEN(packet.payload);
        /* sanity check for packet size */
        if (packet_len > ELWB_CONF_MAX_PAYLOAD_LEN) {
          LOG_ERROR("invalid packet length detected");
          packet_len = 0;
        }
      }
    }
    /* send the packet */
    if (packet_len) {
      ELWB_SET_PKT_HEADER(&packet);
      packet_len += ELWB_PKT_HDR_LEN;
#if ELWB_CONF_DATA_ACK
      /* only source nodes receive a D-ACK */
      if (!is_host && data_packet) {
        if (my_slots == 0xffff) {
          my_slots = (slot_idx << 8);   /* store the index of the first assigned slot in the upper 8 bytes */
        }
        my_slots++;
        /* copy the packet into the queue for retransmission (in case we don't receive a D-ACK for this packet) */
        if (!ELWB_QUEUE_PUSH(re_tx_queue, packet.payload)) {
          LOG_ERROR("failed to insert packet into retransmit queue");
        }
      }
#endif /* ELWB_CONF_DATA_ACK */
      /* wait until the data slot starts */
      elwb_wait_until(slot_start);
      gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
      elwb_wait_until(slot_start + slot_length);
      gloria_stop();
      stats.pkt_tx_all++;
      if (data_packet) {
        stats.pkt_sent++;   /* only count data packets */
        LOG_VERBOSE("packet sent (%lub)", packet_len);
      }
    }

  } else if (data_packet) {
    LOG_VERBOSE("no message to send (data slot ignored)");
  }

  if (slot_cb) {
    slot_cb(schedule.slot[slot_idx], data_packet ? ELWB_PHASE_DATA : ELWB_PHASE_REQ, &packet);
  }
}


static void elwb_receive_packet(elwb_time_t slot_start, uint32_t slot_length, uint32_t slot_idx)
{
  uint8_t packet_len = 0;
  bool data_packet = ELWB_SCHED_HAS_DATA_SLOTS(&schedule);

  if (!data_packet) {
    /* the packet length is known in the request round */
    packet_len = ELWB_REQ_PKT_LEN + ELWB_PKT_HDR_LEN;
  }
  memset(&packet, 0, sizeof(packet));    /* clear packet before receiving the packet */

  elwb_wait_until(slot_start - ELWB_CONF_T_GUARD_SLOT);
  gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
  elwb_wait_until(slot_start + slot_length + ELWB_CONF_T_GUARD_SLOT);
  gloria_stop();

  packet_len = gloria_get_payload_len();
  if (gloria_get_rx_cnt() && ELWB_IS_PKT_HEADER_VALID(&packet)) {                   /* data received? */
    if (data_packet) {
      /* check whether to keep this packet */
      bool keep_packet = ELWB_IS_SINK() || ELWB_RCV_PKT_FILTER();
      if (!is_host) {
        /* source nodes keep all packets received from the host */
        keep_packet |= (schedule.slot[slot_idx] == DPP_DEVICE_ID_SINK) || (schedule.slot[slot_idx] == schedule.host_id);
      }
      if (keep_packet) {
        LOG_VERBOSE("data received from node %u (%lub)", schedule.slot[slot_idx], packet_len);
        if (ELWB_QUEUE_PUSH(rx_queue, packet.payload)) {
          stats.pkt_rcvd++;
#if ELWB_CONF_DATA_ACK
          /* set the corresponding bit in the data ack packet */
          data_ack[slot_idx >> 3] |= (1 << (slot_idx & 0x07));
#endif /* ELWB_CONF_DATA_ACK */
        } else {
          stats.pkt_dropped++;
          LOG_WARNING("RX queue full, message dropped");
        }
      } else {
        stats.pkt_dropped++;
      }

    } else if (is_host && (packet_len == (ELWB_REQ_PKT_LEN + ELWB_PKT_HDR_LEN))) {
      /* this is a request packet */
      elwb_sched_process_req(schedule.slot[slot_idx], packet.req.num_slots);
    }
    stats.pkt_rx_all++;

  } else if (data_packet) {
    LOG_VERBOSE("no data received from node %u", schedule.slot[slot_idx]);
  }

  if (slot_cb) {
    slot_cb(schedule.slot[slot_idx], data_packet ? ELWB_PHASE_DATA : ELWB_PHASE_REQ, &packet);
  }
}


#if ELWB_CONF_DATA_ACK

/* data acknowledgment slot */
static void elwb_data_ack(elwb_time_t slot_start)
{
  uint8_t packet_len = 0;

  if (is_host) {
    /* acknowledge each received packet of the last round */
    packet_len = (ELWB_SCHED_N_SLOTS(&schedule) + 7) / 8;
    if (packet_len) {
      memcpy(packet.payload, data_ack, packet_len);
      ELWB_SET_PKT_HEADER(&packet);
      packet_len += ELWB_PKT_HDR_LEN;
      elwb_wait_until(slot_start);
      /* send D-ACK */
      gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
      elwb_wait_until(slot_start + t_dack);
      gloria_stop();
      stats.pkt_tx_all++;
      LOG_INFO("D-ACK sent (%u bytes)", packet_len);
    }
    memset(data_ack, 0, (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8);

  } else {
    /* receive D-ACK */
    elwb_wait_until(slot_start - ELWB_CONF_T_GUARD_SLOT);
    gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
    elwb_wait_until(slot_start + t_dack + ELWB_CONF_T_GUARD_SLOT);
    gloria_stop();
    packet_len = gloria_get_payload_len();
    /* only look into the D-ACK packet if we actually sent some data in the previous round */
    if (my_slots != 0xffff) {
      uint32_t first_slot = my_slots >> 8;
      uint32_t num_slots  = my_slots & 0xff;
      if (gloria_get_rx_cnt() && ELWB_IS_PKT_HEADER_VALID(&packet)) {
        LOG_VERBOSE("D-ACK received");
        memcpy(data_ack, packet.payload, packet_len);
        uint32_t i;
        for (i = 0; i < num_slots; i++) {
          if (ELWB_QUEUE_POP(re_tx_queue, packet.payload)) {
            /* bit not set? => not acknowledged */
            if (!(data_ack[(first_slot + i) >> 3] & (1 << ((first_slot + i) & 0x07)))) {
              /* resend the packet (re-insert it into the output FIFO) */
              if (ELWB_QUEUE_PUSH(tx_queue, packet.payload)) {
                LOG_VERBOSE("packet queued for retransmission");
              } else {
                LOG_ERROR("failed to requeue packet");
              }
            } else {
              stats.pkt_ack++;
            }
          } else {
            LOG_ERROR("retransmit queue empty");
            break;
          }
        }
        stats.pkt_rx_all++;

      } else {
        /* requeue all packets */
        while (ELWB_QUEUE_POP(re_tx_queue, packet.payload)) {
          if (!ELWB_QUEUE_PUSH(tx_queue, packet.payload)) {
            LOG_ERROR("failed to requeue packet");
            break;
          }
        }
        LOG_WARNING("D-ACK pkt missed, %u pkt requeued", num_slots);
      }
      my_slots = 0xffff;

    } else if (gloria_get_rx_cnt() && ELWB_IS_PKT_HEADER_VALID(&packet)) {
      stats.pkt_rx_all++;
    }
    ELWB_QUEUE_CLEAR(re_tx_queue);  /* make sure the retransmit queue is empty */
  }

  if (slot_cb) {
    slot_cb(schedule.host_id, ELWB_PHASE_DACK, &packet);
  }
}

#endif /* ELWB_CONF_DATA_ACK */


static void elwb_contention(elwb_time_t slot_start, bool node_registered)
{
  const uint8_t packet_len = ELWB_REQ_PKT_LEN + ELWB_PKT_HDR_LEN;
  packet.cont.node_id = 0;

  /* if there is data in the output buffer, then request a slot */
  if (!is_host &&
      (ELWB_QUEUE_SIZE(tx_queue) >= ELWB_CONF_CONT_TH) &&
      rand_backoff == 0) {
    /* node not yet registered? -> include node ID in the request */
    if (!node_registered) {
      packet.cont.node_id = NODE_ID;
      LOG_INFO("transmitting node ID");
    }
    ELWB_SET_PKT_HEADER(&packet);
#if ELWB_CONF_CONT_USE_HSTIMER
    /* contention slot requires precise timing: use the high-speed timer for this wake-up! */
    ELWB_TIMER_HS_SET(last_synced_hs + LPTIMER_TICKS_TO_HS_TIMER(slot_start - last_synced), elwb_notify);
    ELWB_SUSPENDED();
    ELWB_TASK_YIELD();
    ELWB_RESUMED();
#else /* ELWB_CONF_CONT_USE_HSTIMER */
    /* wait until the contention slot starts */
    elwb_wait_until(slot_start);
#endif /* ELWB_CONF_CONT_USE_HSTIMER */
    gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
    elwb_wait_until(slot_start + t_cont);
    gloria_stop();
    /* set random backoff time between 0 and 3 */
    rand_backoff = (rand() % ELWB_CONF_RAND_BACKOFF);

    if (slot_cb) {
      slot_cb(NODE_ID, ELWB_PHASE_CONT, &packet);
    }

  } else {

    /* just receive / relay packets */
    elwb_wait_until(slot_start - ELWB_CONF_T_GUARD_SLOT);
    gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
    elwb_wait_until(slot_start + t_cont + ELWB_CONF_T_GUARD_SLOT);
    gloria_stop();
    if (rand_backoff) {
      rand_backoff--;
    }

    /* NOTE: do not move slot_cb call further down, otherwise the buffer "packet" can be overwritten by the following statements! */
    if (slot_cb) {
      slot_cb(packet.cont.node_id, ELWB_PHASE_CONT, &packet);
    }

    if (is_host) {
      if (gloria_get_rx_cnt() &&
          ELWB_IS_PKT_HEADER_VALID(&packet) &&
          (gloria_get_payload_len() == packet_len) &&
          (packet.cont.node_id != 0)) {
        /* process the request only if there is a valid node ID */
        elwb_sched_process_req(packet.cont.node_id, 0);
      }
      if (gloria_get_rx_started_cnt()) {      /* contention detected? */
        /* set the period to 0 to notify the scheduler that at least one nodes has data to send */
        schedule.period = 0;
        LOG_VERBOSE("contention detected");
        /* compute 2nd schedule */
        elwb_sched_compute(&schedule, 0);  /* do not allocate slots for host */
        packet.sched2.period = schedule.period;
      } else {
        /* else: no update to schedule needed; set period to 0 to indicate 'no change in period' */
        packet.sched2.period = 0;
      }
      elwb_update_rssi_snr();
    }
  }
}


static void elwb_send_rcv_sched2(elwb_time_t slot_start)
{
  uint8_t packet_len = ELWB_2ND_SCHED_LEN + ELWB_PKT_HDR_LEN;

  if (is_host) {
    /* note: packet content is set above during the contention slot */
    ELWB_SET_PKT_HEADER(&packet);
    elwb_wait_until(slot_start);
    /* send as normal packet without sync */
    gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
    elwb_wait_until(slot_start + t_cont);       /* length of a contention slot is sufficient for this short packet */
    gloria_stop();
    stats.pkt_tx_all++;

  } else {
    elwb_wait_until(slot_start - ELWB_CONF_T_GUARD_SLOT);
    gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
    elwb_wait_until(slot_start + t_cont + ELWB_CONF_T_GUARD_SLOT);
    gloria_stop();
    if (gloria_get_rx_cnt() &&
        ELWB_IS_PKT_HEADER_VALID(&packet) &&
        (gloria_get_payload_len() == packet_len)) {     /* packet received? */
      if (packet.sched2.period != 0) {             /* zero means no change */
        schedule.period  = packet.sched2.period;   /* extract updated period */
        schedule.n_slots = 0;
      } /* else: all good, no need to change anything */
      stats.pkt_rx_all++;

    } else {
      LOG_WARNING("2nd schedule missed");
    }
  }

  if (slot_cb) {
    slot_cb(schedule.host_id, ELWB_PHASE_SCHED2, &packet);
  }
}


int32_t elwb_calc_drift_comp(uint32_t elapsed_ticks)
{
  static uint32_t drift_counter = 0;
  int32_t         drift_comp    = 0;
  if (stats.drift != 0) {
    drift_counter    += elapsed_ticks;
    int32_t drift_div = 1000000 / stats.drift;
    drift_comp        = drift_counter / drift_div;
    drift_counter    -= drift_comp * drift_div;
  } else {
    drift_counter = 0;
  }
  return drift_comp;
}


static void elwb_print_stats(void)
{
  if (is_host) {
    LOG_INFO("%llu | T: %lus, slots: %u, rx/tx/drop/rx_all/tx_all: %lu/%lu/%lu/%lu/%lu, rssi: %ddBm",
             network_time,
             elwb_sched_get_period(),
             ELWB_SCHED_N_SLOTS(&schedule),
             stats.pkt_rcvd,
             stats.pkt_sent,
             stats.pkt_dropped,
             stats.pkt_rx_all,
             stats.pkt_tx_all,
             stats.rssi_avg);
  } else {
    /* print out some stats (note: takes ~2ms to compose this string!) */
    LOG_INFO("%s %llu | T: %lus, slots: %u, rx/tx/ack/drop/rx_all/tx_all: %lu/%lu/%lu/%lu/%lu/%lu, usync: %lu/%lu, drift: %ld, rssi: %ddBm",
             elwb_syncstate_to_string[sync_state],
             schedule.time,
             ELWB_TICKS_TO_S(schedule.period),
             ELWB_SCHED_N_SLOTS(&schedule),
             stats.pkt_rcvd,
             stats.pkt_sent,
             stats.pkt_ack,
             stats.pkt_dropped,
             stats.pkt_rx_all,
             stats.pkt_tx_all,
             stats.unsynced_cnt,
             stats.bootstrap_cnt,
             stats.drift,
             stats.rssi_avg);
  }
}


/* ELWB MAIN FUNCTION */
static void elwb_run(void)
{
  elwb_time_t start_of_round  = ELWB_TIMER_NOW();
  bool        node_registered = false;
  bool        call_preprocess = false;

  /* --- begin MAIN LOOP for eLWB --- */
  while (elwb_running) {

    /* --- PREPROCESS --- */
    if (ELWB_CONF_T_PREPROCESS && call_preprocess) {    /* call the preprocess task? */
      if (pre_task) {
        ELWB_TASK_NOTIFY(pre_task);
        /* note: this is cooperative multitasking, the pre-task must complete within ELWB_CONF_T_PREPROCESS time */
      }
      start_of_round += ELWB_CONF_T_PREPROCESS;
      elwb_wait_until(start_of_round);
    }

    /* --- ROUND STARTS --- */

    if (is_host) {
      /* --- SEND SCHEDULE --- */
      elwb_send_schedule(start_of_round);

    } else {

      /* --- RECEIVE SCHEDULE --- */
      if (sync_state == BOOTSTRAP) {
        elwb_bootstrap();
      } else {
        elwb_receive_schedule(start_of_round);
      }
      /* validate the schedule and update the sync state / reference time */
      start_of_round = elwb_sync(start_of_round, call_preprocess);

      if (sync_state == BOOTSTRAP) {
        call_preprocess = false;
        continue;     /* abort this round and go back to bootstrapping */
      }
    }

    /* only synced nodes may participate in the round */
    if (sync_state == SYNCED) {

      uint64_t slot_ofs = start_of_round + (t_sched + ELWB_CONF_T_GAP);
      uint32_t slot_time;

      /* --- DATA SLOTS --- */

      if (ELWB_SCHED_HAS_SLOTS(&schedule)) {

      #if ELWB_CONF_SCHED_COMPRESS
        if (!elwb_sched_uncompress((uint8_t*)schedule.slot, ELWB_SCHED_N_SLOTS(&schedule))) {
          LOG_ERROR("failed to uncompress the schedule");
        }
      #endif /* ELWB_CONF_SCHED_COMPRESS */

        /* set the slot duration */
        if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
          slot_time = t_data;
    #if ELWB_CONF_DATA_ACK
          my_slots  = 0xffff;
    #endif /* ELWB_CONF_DATA_ACK */
        } else {
          /* it's a request round */
          slot_time       = t_cont;
          node_registered = false;
          rand_backoff    = 0;        /* reset, contention was successful */
        }
        /* loop through all slots in this round */
        uint32_t slot_idx;
        for (slot_idx = 0; slot_idx < ELWB_SCHED_N_SLOTS(&schedule); slot_idx++) {
          /* note: slots with node ID 0 belong to the host */
          bool is_initiator = (schedule.slot[slot_idx] == NODE_ID) || (is_host && schedule.slot[slot_idx] == DPP_DEVICE_ID_SINK);
          if (is_initiator) {
            node_registered = true;
            elwb_send_packet(slot_ofs, slot_time, slot_idx);    /* initiator -> send packet */
          } else {
            elwb_receive_packet(slot_ofs, slot_time, slot_idx); /* not initiator -> receive / relay packets */
          }
          slot_ofs += (slot_time + ELWB_CONF_T_GAP);
        }
      }

    #if ELWB_CONF_DATA_ACK
      /* --- D-ACK SLOT --- */
      if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule) && !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        elwb_data_ack(slot_ofs);
        slot_ofs += (t_dack + ELWB_CONF_T_GAP);
      }
    #endif /* ELWB_CONF_DATA_ACK */

      /* is there a contention slot in this round? */
      if (ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {

        /* --- CONTENTION SLOT --- */
        elwb_contention(slot_ofs, node_registered);
        slot_ofs += t_cont + ELWB_CONF_T_GAP;

        /* --- 2ND SCHEDULE (only in case of a contention slot) --- */

        elwb_send_rcv_sched2(slot_ofs);
        slot_ofs += t_cont + ELWB_CONF_T_GAP;
      }
    }

    /* --- ROUND ENDS --- */

    if (ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      elwb_print_stats();
      /* poll the post process */
      if (post_task) {
        ELWB_TASK_NOTIFY(post_task);
      }
      if (ELWB_CONF_T_PREPROCESS) {
        call_preprocess = true;
      }
    } else {
      call_preprocess = false;
    }

    uint32_t round_ticks = schedule.period;
    ELWB_SCHED_CLR_SLOTS(&schedule);
    if (is_host) {
      /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
      schedule_len = elwb_sched_compute(&schedule, ELWB_QUEUE_SIZE(tx_queue));
    }

    /* calculate extra ticks for drift compensation */
    int32_t drift_comp = elwb_calc_drift_comp(round_ticks);

    /* schedule the wakeup for the next round */
    start_of_round += round_ticks + drift_comp;
    if (!is_host) {
      start_of_round -= ELWB_CONF_T_GUARD_ROUND;   /* add guard time on a source node */
    }
    if (call_preprocess) {
      start_of_round -= ELWB_CONF_T_PREPROCESS;    /* wake up earlier such that the pre task can run */
    }
    elwb_wait_until(start_of_round);
  }

  LOG_INFO("stopped");
}


bool elwb_init(void* elwb_task,
               void* pre_elwb_task,
               void* post_elwb_task,
               void* in_queue_handle,
               void* out_queue_handle,
               void* retransmit_queue_handle,
               elwb_timeout_cb_t listen_timeout_cb,
               bool host)
{
  if (!in_queue_handle || !out_queue_handle || !elwb_task) {
    LOG_ERROR("invalid parameters");
    return false;
  }

#if ELWB_CONF_DATA_ACK
  if (!retransmit_queue_handle) {
    LOG_ERROR("invalid parameters");
    return false;
  }
#endif /* ELWB_CONF_DATA_ACK */

  task_handle  = elwb_task;
  pre_task     = pre_elwb_task;
  post_task    = post_elwb_task;
  rx_queue     = in_queue_handle;
  tx_queue     = out_queue_handle;
  re_tx_queue  = retransmit_queue_handle;
  timeout_cb   = listen_timeout_cb;
  elwb_update_slot_durations(0, 0);

  /* clear all queues */
  ELWB_QUEUE_CLEAR(rx_queue);
  ELWB_QUEUE_CLEAR(tx_queue);
#if ELWB_CONF_DATA_ACK
  ELWB_QUEUE_CLEAR(re_tx_queue);
#endif /* ELWB_CONF_DATA_ACK */

  memset(&stats, 0, sizeof(elwb_stats_t));

  is_host = host;
  if (is_host) {
    LOG_INFO("host node, network ID 0x%04x", (ELWB_CONF_NETWORK_ID & ELWB_NETWORK_ID_BITMASK));
    sync_state   = SYNCED;
    schedule_len = elwb_sched_init(&schedule);
    if (!schedule_len) {
      LOG_ERROR("schedule has length 0");
      return false;
    }
  } else {
    LOG_INFO("source node, network ID 0x%04x", (ELWB_CONF_NETWORK_ID & ELWB_NETWORK_ID_BITMASK));
    sync_state = BOOTSTRAP;
  }

  return true;
}


void elwb_start(void)
{
  LOG_INFO("pkt_len: %u, slots: %u, n_tx: %u, t_sched: %lu, t_data: %lu, t_cont: %lu",
           ELWB_CONF_MAX_PAYLOAD_LEN,
           ELWB_CONF_MAX_DATA_SLOTS,
           n_tx,
           (uint32_t)ELWB_TICKS_TO_MS(t_sched),
           (uint32_t)ELWB_TICKS_TO_MS(t_data),
           (uint32_t)ELWB_TICKS_TO_MS(t_cont));

  elwb_running = true;

  /* instead of calling elwb_run(), schedule the start */
#if ELWB_CONF_STARTUP_DELAY > 0
  elwb_time_t starttime = ELWB_MS_TO_TICKS(ELWB_CONF_STARTUP_DELAY);
  if (is_host) {
    starttime += ELWB_CONF_T_GUARD_ROUND;    /* delay the host by ELWB_CONF_T_GUARD_ROUND */
  }
  if (ELWB_TIMER_NOW() < starttime) {
    elwb_wait_until(starttime);
  }
#endif /* ELWB_CONF_STARTUP_DELAY */

  elwb_run();
}


void elwb_stop(void)
{
  ELWB_TIMER_STOP();
  elwb_running = false;
  elwb_notify();
}

#endif /* ELWB_ENABLE */
