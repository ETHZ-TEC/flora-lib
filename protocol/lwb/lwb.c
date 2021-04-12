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

/* internal sync state of the eLWB */
typedef enum {
  BOOTSTRAP = 0,
  SYNCED,
  UNSYNCED,
  UNSYNCED2,
  NUM_OF_SYNC_STATES
} lwb_syncstate_t;


typedef enum {
  EVT_SCHED_RCVD = 0,
  EVT_SCHED_MISSED,
  NUM_OF_SYNC_EVENTS
} sync_event_t;


static const lwb_syncstate_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] =
{/* STATES:                                         EVENTS:         */
 /* BOOTSTRAP, SYNCED,   UNSYNCED,  UNSYNCED2                       */
  { SYNCED,    SYNCED,   SYNCED,    SYNCED    }, /* schedule rcvd   */
  { BOOTSTRAP, UNSYNCED, UNSYNCED2, BOOTSTRAP }  /* schedule missed */
};
static const char* lwb_syncstate_to_string[NUM_OF_SYNC_STATES] = {
  "BOOTSTRAP", "SYN", "USYN", "USYN2"
};


static lwb_stats_t        stats;
static lwb_time_t         last_synced;
static lwb_time_t         network_time;
static void*              task_handle  = 0;
static void*              post_task    = 0;
static void*              pre_task     = 0;
static void*              rx_queue     = 0;
static void*              tx_queue     = 0;
static bool               lwb_running  = false;
static bool               is_host      = 0;
static lwb_timeout_cb_t   timeout_cb   = 0;
static lwb_slot_cb_t      slot_cb      = 0;
static lwb_schedule_t     schedule;
static uint_fast8_t       schedule_len;
static lwb_packet_t       packet;             /* packet buffer */

static uint8_t            n_tx         = LWB_N_TX;
static uint8_t            num_hops     = LWB_NUM_HOPS;
static uint32_t           t_sched      = 0;      /* slot length for schedule packets */
static uint32_t           t_data       = 0;      /* slot length for data packets */
static uint32_t           t_cont       = 0;      /* slot length for contention, request and sched2 packets */
static uint16_t           ipi          = 0;
static bool               ipi_changed  = false;

#if LWB_CONT_USE_HSTIMER
static lwb_time_t         last_synced_hs;
#endif /* LWB_CONT_USE_HSTIMER */
static lwb_syncstate_t    sync_state;
static uint_fast16_t      period_idle;        /* last known base period */

/* private (not exposed) scheduler functions */
uint32_t lwb_sched_init(lwb_schedule_t* sched);
bool     lwb_sched_process_req(uint16_t id, uint16_t ipi);
uint32_t lwb_sched_compute(lwb_schedule_t * const sched, uint32_t reserve_slots_host);
void     lwb_sched_set_time_offset(uint32_t ofs);


/*
 * This function can be called from an interrupt context to poll the LWB task.
 */
void lwb_notify(void)
{
  if (task_handle) {
    if (xTaskGetCurrentTaskHandle() == task_handle) {
      LOG_WARNING("lwb_notify(): task is already running");
      if (post_task) {
        LWB_TASK_NOTIFY(post_task);
      }
    }
    LWB_ON_WAKEUP();
    LWB_TASK_NOTIFY_FROM_ISR(task_handle);
  }
}


void lwb_wait_until(lwb_time_t timeout)
{
  if (lwb_running) {
    LWB_TIMER_SET(timeout, lwb_notify);
    LWB_SUSPENDED();
    LWB_TASK_YIELD();
    LWB_RESUMED();
  }
}


void lwb_schedule_received_callback(void)
{
  LWB_TIMER_SET(0, 0);   /* cancel timer */
  lwb_notify();
}


const lwb_stats_t * const lwb_get_stats(void)
{
  return &stats;
}


/* returns the max. round duration in ticks, including the time allocated for the pre-process task */
uint32_t lwb_get_max_round_duration(uint32_t t_sched_arg, uint32_t t_cont_arg, uint32_t t_data_arg)
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
  return LWB_T_PREPROCESS + t_sched_arg + LWB_T_GAP + LWB_MAX_DATA_SLOTS * (t_data_arg + LWB_T_GAP) + LWB_T_GAP + t_cont_arg + LWB_T_GAP + LWB_SCHED_COMP_TIME + t_cont_arg;
}


void lwb_get_last_syncpoint(lwb_time_t* time, lwb_time_t* rx_timestamp)
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
lwb_time_t lwb_get_time(const uint64_t* timestamp)
{
  uint64_t ts;
  if (timestamp) {
    ts = *timestamp;
  } else {
    ts = LWB_TIMER_NOW();
  }
  return network_time + ((int64_t)ts - (int64_t)last_synced) * (1000000 - stats.drift) / (LWB_TIMER_FREQUENCY);
}

uint32_t lwb_get_time_sec(void)
{
  return lwb_get_time(0) / 1000000;
}


void lwb_set_drift(int32_t drift_ppm)
{
  stats.drift = drift_ppm;
}


void lwb_register_slot_callback(lwb_slot_cb_t cb)
{
  slot_cb = cb;
}


bool lwb_update_slot_durations(uint8_t n_tx_arg, uint8_t num_hops_arg)
{
  if (!n_tx_arg) {
    n_tx_arg = n_tx;
  }
  if (!num_hops_arg) {
    num_hops_arg = num_hops;
  }
  uint32_t t_sched_new = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, (LWB_MAX_DATA_SLOTS * sizeof(uint16_t) + LWB_SCHED_HDR_LEN + LWB_SCHED_CRC_LEN));
  uint32_t t_data_new  = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, LWB_MAX_PAYLOAD_LEN);
  uint32_t t_cont_new  = GLORIA_INTERFACE_FLOOD_DURATION(n_tx_arg, num_hops_arg, MAX(LWB_CONT_PKT_LEN, LWB_2ND_SCHED_LEN) + LWB_PKT_HDR_LEN);

  if (is_host && !lwb_sched_check_params(0, t_sched_new, t_cont_new, t_data_new)) {
    return false;
  }
  t_sched = t_sched_new;
  t_data  = t_data_new;
  t_cont  = t_cont_new;

  return true;
}


uint8_t lwb_get_n_tx(void)
{
  return n_tx;
}


bool lwb_set_n_tx(uint8_t n_tx_arg)
{
  if (lwb_update_slot_durations(n_tx_arg, 0)) {
    n_tx = n_tx_arg;
    return true;
  }
  return false;
}


uint8_t lwb_get_num_hops(void)
{
  return num_hops;
}


bool lwb_set_num_hops(uint8_t num_hops_arg)
{
  if (lwb_update_slot_durations(0, num_hops_arg)) {
    num_hops = num_hops_arg;
    return true;
  }
  return false;
}


void lwb_set_ipi(uint16_t ipi_secs)
{
  if (ipi != ipi_secs) {
    ipi = ipi_secs;
    ipi_changed = true;
  }
}


static void lwb_update_rssi_snr(void)
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


static bool lwb_bootstrap_sched_pkt_filter(uint8_t* pkt, uint8_t len)
{
  if ((len > LWB_PKT_HDR_LEN) && LWB_IS_PKT_HEADER_VALID((lwb_packet_t*)pkt) && LWB_IS_SCHEDULE_PACKET((lwb_packet_t*)pkt)) {
    return true;
  }
  return false;
}

static bool lwb_is_schedule_valid(lwb_schedule_t* schedule)
{
  if (!gloria_is_t_ref_updated() || !LWB_IS_PKT_HEADER_VALID(schedule) || !LWB_IS_SCHEDULE_PACKET(schedule) || (gloria_get_payload_len() < (LWB_SCHED_HDR_LEN + LWB_SCHED_CRC_LEN))) {
    return false;
  }
  return true;
}


static void lwb_bootstrap(void)
{
  stats.bootstrap_cnt++;
  LOG_INFO("bootstrap");
  lwb_time_t bootstrap_timeout = LWB_TIMER_NOW() + LWB_BOOTSTRAP_TIMEOUT;
  /* keep listening until we receive a valid schedule packet */
  do {
    gloria_register_flood_callback(lwb_schedule_received_callback);
    gloria_set_pkt_filter(lwb_bootstrap_sched_pkt_filter);
    gloria_start(false, (uint8_t*)&schedule, 0, n_tx, 1);
    lwb_wait_until(LWB_TIMER_NOW() + t_sched);
    gloria_stop();
    if (LWB_TIMER_NOW() > bootstrap_timeout) {
      /* go to sleep for LWB_T_DEEPSLEEP ticks */
      stats.sleep_cnt++;
      LOG_WARNING("timeout");
      /* poll the post process */
      if (post_task) {
        LWB_TASK_NOTIFY(post_task);
      }
      if (timeout_cb) {
        timeout_cb();
      }
      lwb_wait_until(LWB_TIMER_NOW() + LWB_T_DEEPSLEEP);
      bootstrap_timeout = LWB_TIMER_NOW() + LWB_BOOTSTRAP_TIMEOUT;
    }
  } while (lwb_running && !lwb_is_schedule_valid(&schedule));

  if (slot_cb) {
    slot_cb(schedule.host_id, LWB_PHASE_SCHED1, (lwb_packet_t*)&schedule);
  }
}


static void lwb_send_schedule(lwb_time_t start_of_round)
{
  gloria_start(true, (uint8_t*)&schedule, schedule_len, n_tx, 1);
  lwb_wait_until(start_of_round + t_sched);
  gloria_stop();
  stats.pkt_tx_all++;

  /* update sync point */
  network_time = schedule.time;
  last_synced  = start_of_round;
  /* calculate the reference offset for the source nodes (time between start_of_round and the tx marker) */
  stats.ref_ofs = (stats.ref_ofs + (gloria_get_t_ref() - start_of_round)) / 2;
  lwb_sched_set_time_offset(stats.ref_ofs);

  if (slot_cb) {
    slot_cb(schedule.host_id, LWB_PHASE_SCHED1, (lwb_packet_t*)&schedule);
  }
}


static void lwb_receive_schedule(lwb_time_t start_of_round)
{
  gloria_start(false, (uint8_t*)&schedule, 0, n_tx, 1);
  lwb_wait_until(start_of_round + t_sched + LWB_T_GUARD_ROUND);
  gloria_stop();

  if (slot_cb) {
    slot_cb(schedule.host_id, LWB_PHASE_SCHED1, (lwb_packet_t*)&schedule);
  }
}


static lwb_time_t lwb_sync(lwb_time_t start_of_round)
{
  lwb_time_t t_ref = 0;

  /* valid schedule received? */
  if (lwb_is_schedule_valid(&schedule)) {

#if LWB_SCHED_ADD_CRC
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
#endif /* LWB_SCHED_ADD_CRC */

    /* schedule sanity check (#slots mustn't exceed the compile-time fixed max. # slots!) */
    if (schedule.n_slots > LWB_MAX_DATA_SLOTS) {
      LOG_ERROR("n_slots exceeds limit!");
      schedule.n_slots = LWB_MAX_DATA_SLOTS;
    }
    /* update the sync state machine */
    sync_state = next_state[EVT_SCHED_RCVD][sync_state];

    t_ref = gloria_get_t_ref();
    /* do some basic drift estimation:
     * measured elapsed time minus effective elapsed time (given by host) */
    int64_t elapsed_network_us = (schedule.time - network_time);
    int64_t elapsed_local_us   = (t_ref - last_synced) * 1000000 / LWB_TIMER_FREQUENCY;
    int32_t delta_us           = (elapsed_local_us - elapsed_network_us);
    /* now scale the difference from ticks to ppm (note: a negative drift means the local clock runs slower than the network clock) */
    int32_t drift_ppm = (delta_us * 1000) / (elapsed_network_us / 1000);
    if (drift_ppm < LWB_MAX_CLOCK_DRIFT &&
        drift_ppm > -LWB_MAX_CLOCK_DRIFT) {
      stats.drift = (stats.drift + drift_ppm) / 2;
    }
    /* only update the timestamp during the idle period */
    period_idle  = schedule.period;
    network_time = schedule.time;
    last_synced  = t_ref;
#if LWB_CONT_USE_HSTIMER
    /* also store the HF timestamp in case LF is used for slot wakeups */
    last_synced_hs = gloria_get_t_ref_hs();
#endif /* LWB_CONT_USE_HSTIMER */

    /* update stats */
    lwb_update_rssi_snr();
    stats.pkt_rx_all++;

  } else {
    /* update the sync state machine */
    sync_state = next_state[EVT_SCHED_MISSED][sync_state];

    if (sync_state != BOOTSTRAP) {
      stats.unsynced_cnt++;
      LOG_WARNING("schedule missed");

      /* estimate t_ref and reset period */
      t_ref           = start_of_round;
      schedule.period = period_idle;
    }
  }

  return t_ref;
}


static void lwb_send_packet(lwb_time_t slot_start, uint32_t slot_length, uint32_t slot_idx)
{
  /* send a data packet (if there is any) */
  if (LWB_QUEUE_SIZE(tx_queue) > 0) {
    uint8_t packet_len = 0;
    if (LWB_QUEUE_POP(tx_queue, packet.payload)) {
      packet_len = LWB_PAYLOAD_LEN(packet.payload);
      /* sanity check for packet size */
      if (packet_len > LWB_MAX_PAYLOAD_LEN) {
        LOG_ERROR("invalid packet length detected");
        packet_len = 0;
      }
    }
    /* send the packet */
    if (packet_len) {
      LWB_SET_PKT_HEADER(&packet);
      packet_len += LWB_PKT_HDR_LEN;
      /* wait until the data slot starts */
      lwb_wait_until(slot_start);
      gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
      lwb_wait_until(slot_start + slot_length);
      gloria_stop();
      stats.pkt_tx_all++;
      stats.pkt_sent++;   /* only count data packets */
      LOG_VERBOSE("packet sent (%lub)", packet_len);
    }

  } else {
    LOG_VERBOSE("no message to send (data slot ignored)");
  }

  if (slot_cb) {
    slot_cb(schedule.slot[slot_idx], LWB_PHASE_DATA, &packet);
  }
}


static void lwb_receive_packet(lwb_time_t slot_start, uint32_t slot_length, uint16_t initiator_id)
{
  uint8_t packet_len = 0;

  memset(&packet, 0, sizeof(packet));    /* clear packet before receiving the packet */

#if LWB_USE_TX_DELAY
  uint8_t* delay_mask = (uint8_t*)&schedule.slot[schedule.n_slots];  /* delay mask starts after the last slot in the schedule */
  uint16_t id_ofs     = (NODE_ID - LWB_MIN_NODE_ID);
  if (delay_mask[id_ofs / 8] & (1 << (id_ofs & 0x7))) {
    gloria_set_tx_delay(1);
  }
#endif /* LWB_USE_TX_DELAY */

  lwb_wait_until(slot_start - LWB_T_GUARD_SLOT);
  gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
  lwb_wait_until(slot_start + slot_length + LWB_T_GUARD_SLOT);
  gloria_stop();

  packet_len = gloria_get_payload_len();
  if (gloria_get_rx_cnt() && LWB_IS_PKT_HEADER_VALID(&packet)) {                   /* data received? */
    /* check whether to keep this packet */
    bool keep_packet = LWB_IS_SINK() || LWB_RCV_PKT_FILTER();
    if (!is_host) {
      /* source nodes keep all packets received from the host */
      keep_packet |= (initiator_id == DPP_DEVICE_ID_SINK) || (initiator_id == schedule.host_id);
    }
    if (keep_packet) {
      LOG_VERBOSE("data received from node %u (%lub)", initiator_id, packet_len);
      if (LWB_QUEUE_PUSH(rx_queue, packet.payload)) {
        stats.pkt_rcvd++;
      } else {
        stats.pkt_dropped++;
        LOG_WARNING("RX queue full, message dropped");
      }
    } else {
      stats.pkt_dropped++;
    }
    stats.pkt_rx_all++;

  } else {
    LOG_VERBOSE("no data received from node %u", initiator_id);
  }

  if (slot_cb) {
    slot_cb(initiator_id, LWB_PHASE_DATA, &packet);
  }
}


static void lwb_contention(lwb_time_t slot_start)
{
  static uint_fast8_t rand_backoff = 0;

  const uint8_t packet_len = LWB_CONT_PKT_LEN + LWB_PKT_HDR_LEN;
  packet.cont.node_id = 0;

  /* participate in contention only if the source node has not receive a data slot in the current round and backoff window has elapsed */
  if (!is_host &&
      ipi_changed &&
      rand_backoff == 0) {
    packet.cont.node_id = NODE_ID;
    packet.cont.ipi     = ipi;
    LWB_SET_PKT_HEADER(&packet);
    LOG_INFO("transmitting node ID");
#if LWB_CONT_USE_HSTIMER
    /* contention slot requires precise timing: use the high-speed timer for this wake-up! */
    LWB_TIMER_HS_SET(last_synced_hs + LPTIMER_TICKS_TO_HS_TIMER(slot_start - last_synced), lwb_notify);
    LWB_SUSPENDED();
    LWB_TASK_YIELD();
    LWB_RESUMED();
#else /* LWB_CONT_USE_HSTIMER */
    /* wait until the contention slot starts */
    lwb_wait_until(slot_start);
#endif /* LWB_CONT_USE_HSTIMER */
    gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
    lwb_wait_until(slot_start + t_cont);
    gloria_stop();
    /* set random backoff time between 0 and 3 */
    rand_backoff = (rand() % LWB_RAND_BACKOFF);

    if (slot_cb) {
      slot_cb(NODE_ID, LWB_PHASE_CONT, &packet);
    }

  } else {

    /* just receive / relay packets */
    lwb_wait_until(slot_start - LWB_T_GUARD_SLOT);
    gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
    lwb_wait_until(slot_start + t_cont + LWB_T_GUARD_SLOT);
    gloria_stop();
    if (rand_backoff) {
      rand_backoff--;
    }

    /* NOTE: do not move slot_cb call further down, otherwise the buffer "packet" can be overwritten by the following statements! */
    if (slot_cb) {
      slot_cb(packet.cont.node_id, LWB_PHASE_CONT, &packet);
    }

    if (is_host) {
      packet.sched2.cont_winner = 0;
      if (gloria_get_rx_cnt() &&
          LWB_IS_PKT_HEADER_VALID(&packet) &&
          (gloria_get_payload_len() == packet_len) &&
          (packet.cont.node_id != 0)) {
        /* process the request only if there is a valid node ID */
        if (lwb_sched_process_req(packet.cont.node_id, packet.cont.ipi)) {
          packet.sched2.cont_winner = packet.cont.node_id;
        }
      }
      lwb_update_rssi_snr();
    }
  }
}


static void lwb_send_rcv_sched2(lwb_time_t slot_start)
{
  uint8_t packet_len = LWB_2ND_SCHED_LEN + LWB_PKT_HDR_LEN;

  if (is_host) {
    /* note: packet content is set above during the contention slot */
    packet.sched2.period = schedule.period;
    LWB_SET_PKT_HEADER(&packet);
    lwb_wait_until(slot_start);
    /* send as normal packet without sync */
    gloria_start(true, (uint8_t*)&packet, packet_len, n_tx, 0);
    lwb_wait_until(slot_start + t_cont);       /* length of a contention slot is sufficient for this short packet */
    gloria_stop();
    stats.pkt_tx_all++;

  } else {
    lwb_wait_until(slot_start - LWB_T_GUARD_SLOT);
    gloria_start(false, (uint8_t*)&packet, packet_len, n_tx, 0);
    lwb_wait_until(slot_start + t_cont + LWB_T_GUARD_SLOT);
    gloria_stop();
    if (gloria_get_rx_cnt() &&
        LWB_IS_PKT_HEADER_VALID(&packet) &&
        (gloria_get_payload_len() == packet_len)) {   /* packet received? */
      schedule.period = packet.sched2.period;         /* extract updated period */
      if (packet.sched2.cont_winner == NODE_ID) {
        ipi_changed = false;
        LOG_VERBOSE("IPI change confirmed");
      }
      stats.pkt_rx_all++;

    } else {
      LOG_WARNING("2nd schedule missed");
    }
  }

  if (slot_cb) {
    slot_cb(schedule.host_id, LWB_PHASE_SCHED2, &packet);
  }
}


int32_t lwb_calc_drift_comp(uint32_t elapsed_ticks)
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


static void lwb_print_stats(void)
{
  if (is_host) {
    LOG_INFO("%llu | T: %lus, slots: %u, rx/tx/drop/rx_all/tx_all: %lu/%lu/%lu/%lu/%lu, rssi: %ddBm",
             network_time,
             lwb_sched_get_period(),
             schedule.n_slots,
             stats.pkt_rcvd,
             stats.pkt_sent,
             stats.pkt_dropped,
             stats.pkt_rx_all,
             stats.pkt_tx_all,
             stats.rssi_avg);
  } else {
    /* print out some stats (note: takes ~2ms to compose this string!) */
    LOG_INFO("%s %llu | T: %lus, slots: %u, rx/tx/ack/drop/rx_all/tx_all: %lu/%lu/%lu/%lu/%lu/%lu, usync: %lu/%lu, drift: %ld, rssi: %ddBm",
             lwb_syncstate_to_string[sync_state],
             schedule.time,
             LWB_TICKS_TO_S(schedule.period),
             schedule.n_slots,
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
static void lwb_run(void)
{
  lwb_time_t start_of_round = LWB_TIMER_NOW();

  /* --- begin MAIN LOOP for eLWB --- */
  while (lwb_running) {

    /* --- PREPROCESS --- */
    if (LWB_T_PREPROCESS && (sync_state != BOOTSTRAP)) {
      if (pre_task) {
        LWB_TASK_NOTIFY(pre_task);
        /* note: this is cooperative multitasking, the pre-task must complete within LWB_T_PREPROCESS time */
      }
      start_of_round += LWB_T_PREPROCESS;
      lwb_wait_until(start_of_round);
    }

    /* --- ROUND STARTS --- */

    if (is_host) {
      /* --- SEND SCHEDULE --- */
      lwb_send_schedule(start_of_round);

    } else {

      /* --- RECEIVE SCHEDULE --- */
      if (sync_state == BOOTSTRAP) {
        lwb_bootstrap();
      } else {
        lwb_receive_schedule(start_of_round);
      }
      /* validate the schedule and update the sync state / reference time */
      start_of_round = lwb_sync(start_of_round);

      if (sync_state == BOOTSTRAP) {
        continue;     /* abort this round and go back to bootstrapping */
      }
    }

    /* only synced nodes may participate in the round */
    if (sync_state == SYNCED) {

      uint64_t slot_ofs = start_of_round + (t_sched + LWB_T_GAP);

      /* --- DATA SLOTS --- */

      /* loop through all slots in this round */
      uint32_t slot_idx;
      for (slot_idx = 0; slot_idx < schedule.n_slots; slot_idx++) {
        /* note: slots with node ID 0 belong to the host */
        bool is_initiator = (schedule.slot[slot_idx] == NODE_ID) || (is_host && schedule.slot[slot_idx] == DPP_DEVICE_ID_SINK);
        if (is_initiator) {
          lwb_send_packet(slot_ofs, t_data, slot_idx);                    /* initiator -> send packet */
        } else {
          lwb_receive_packet(slot_ofs, t_data, schedule.slot[slot_idx]);  /* not initiator -> receive / relay packets */
        }
        slot_ofs += (t_data + LWB_T_GAP);
      }

      /* --- CONTENTION SLOT --- */

      lwb_contention(slot_ofs);
      slot_ofs += t_cont + LWB_T_GAP;

      /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
      if (is_host) {
        schedule_len = lwb_sched_compute(&schedule, LWB_QUEUE_SIZE(tx_queue));
      }
      slot_ofs += LWB_SCHED_COMP_TIME;

      /* --- 2ND SCHEDULE --- */

      lwb_send_rcv_sched2(slot_ofs);
    }

    /* --- ROUND ENDS --- */

    lwb_print_stats();
    /* poll the post process */
    if (post_task) {
      LWB_TASK_NOTIFY(post_task);
    }

    /* drift compensation */
    int32_t drift_comp   = lwb_calc_drift_comp(schedule.period);

    /* schedule the wakeup for the next round */
    start_of_round += schedule.period + drift_comp;
    if (!is_host) {
      start_of_round -= LWB_T_GUARD_ROUND;   /* add guard time on a source node */
    }
    if (LWB_T_PREPROCESS) {
      start_of_round -= LWB_T_PREPROCESS;    /* wake up earlier such that the pre task can run */
    }
    lwb_wait_until(start_of_round);
  }

  LOG_INFO("stopped");
}


bool lwb_init(void* lwb_task,
              void* pre_lwb_task,
              void* post_lwb_task,
              void* in_queue_handle,
              void* out_queue_handle,
              lwb_timeout_cb_t listen_timeout_cb,
              bool host)
{
  if (!in_queue_handle || !out_queue_handle || !lwb_task) {
    LOG_ERROR("invalid parameters");
    return false;
  }

  task_handle  = lwb_task;
  pre_task     = pre_lwb_task;
  post_task    = post_lwb_task;
  rx_queue     = in_queue_handle;
  tx_queue     = out_queue_handle;
  timeout_cb   = listen_timeout_cb;
  lwb_update_slot_durations(0, 0);

  /* clear all queues */
  LWB_QUEUE_CLEAR(rx_queue);
  LWB_QUEUE_CLEAR(tx_queue);

  memset(&stats, 0, sizeof(lwb_stats_t));

  is_host = host;
  if (is_host) {
    LOG_INFO("host node, network ID 0x%04x", (LWB_NETWORK_ID & LWB_NETWORK_ID_BITMASK));
    sync_state   = SYNCED;
    schedule_len = lwb_sched_init(&schedule);
    if (!schedule_len) {
      LOG_ERROR("schedule has length 0");
      return false;
    }
  } else {
    LOG_INFO("source node, network ID 0x%04x", (LWB_NETWORK_ID & LWB_NETWORK_ID_BITMASK));
    sync_state = BOOTSTRAP;
  }

  return true;
}


void lwb_start(void)
{
  LOG_INFO("pkt_len=%u slots=%u n_tx=%u t_sched=%lums t_data=%lums t_cont=%lums",
           LWB_MAX_PAYLOAD_LEN,
           LWB_MAX_DATA_SLOTS,
           n_tx,
           (uint32_t)LWB_TICKS_TO_MS(t_sched),
           (uint32_t)LWB_TICKS_TO_MS(t_data),
           (uint32_t)LWB_TICKS_TO_MS(t_cont));

  lwb_running = true;

  /* instead of calling lwb_run(), schedule the start */
#if LWB_STARTUP_DELAY > 0
  lwb_time_t starttime = LWB_MS_TO_TICKS(LWB_STARTUP_DELAY);
  if (is_host) {
    starttime += LWB_T_GUARD_ROUND;    /* delay the host by LWB_T_GUARD_ROUND */
  }
  if (LWB_TIMER_NOW() < starttime) {
    lwb_wait_until(starttime);
  }
#endif /* LWB_STARTUP_DELAY */

  lwb_run();
}


void lwb_stop(void)
{
  LWB_TIMER_STOP();
  lwb_running = false;
  lwb_notify();
}

#endif /* LWB_ENABLE */
