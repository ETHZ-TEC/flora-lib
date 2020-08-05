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


#define ELWB_SEND_SCHED() \
{\
  ELWB_GLOSSY_START(NODE_ID, (uint8_t *)&schedule, schedule_len, ELWB_CONF_N_TX, 1);\
  ELWB_WAIT_UNTIL(ELWB_TIMER_LAST_EXP() + ELWB_CONF_T_SCHED);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_RCV_SCHED() \
{\
  ELWB_GLOSSY_START(0, (uint8_t *)&schedule, payload_len, ELWB_CONF_N_TX, 1);\
  ELWB_WAIT_UNTIL(ELWB_TIMER_LAST_EXP() + ELWB_CONF_T_SCHED + ELWB_CONF_T_GUARD_ROUND);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_SEND_PACKET() \
{\
  ELWB_GLOSSY_START(NODE_ID, (uint8_t*)payload, payload_len, ELWB_CONF_N_TX, 0);\
  ELWB_WAIT_UNTIL(ELWB_TIMER_LAST_EXP() + t_slot);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_RCV_PACKET() \
{\
  ELWB_GLOSSY_START(0, (uint8_t*)payload, payload_len, ELWB_CONF_N_TX, 0);\
  ELWB_WAIT_UNTIL(ELWB_TIMER_LAST_EXP() + t_slot + ELWB_CONF_T_GUARD_SLOT);\
  ELWB_GLOSSY_STOP();\
}


#define ELWB_WAIT_UNTIL(time) \
{\
  if (elwb_running) {\
    ELWB_TIMER_SET(time, elwb_notify);\
    ELWB_SUSPENDED();\
    ELWB_TASK_YIELD();\
    ELWB_RESUMED();\
  }\
}


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
static uint32_t           host_id      = 0;
static void               (*listen_timeout_cb)(void);


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


const elwb_stats_t * const elwb_get_stats(void)
{
  return &stats;
}


void elwb_get_last_syncpoint(elwb_time_t* time, elwb_time_t* rx_timestamp)
{
  if (network_time) {
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
  return network_time + ((int64_t)ts - (int64_t)last_synced) * (1000000 - stats.drift) / (ELWB_TIMER_SECOND);
}

uint32_t elwb_get_time_sec(void)
{
  return elwb_get_time(0) / 1000000;
}


void elwb_set_drift(int32_t drift_ppm)
{
  stats.drift = drift_ppm;
}


static void elwb_update_rssi_snr(void)
{
  int32_t rssi_curr = ELWB_GLOSSY_GET_RSSI();
  if (rssi_curr != 0) {
    if (stats.rssi_avg != 0) {
      stats.rssi_avg = (stats.rssi_avg + rssi_curr) / 2;  /* update the average RSSI value */
    } else {
      stats.rssi_avg = rssi_curr;
    }
  }
  int32_t snr_curr = ELWB_GLOSSY_GET_SNR();
  if (snr_curr != 0) {
    if (stats.snr_avg != 0) {
      stats.snr_avg = (stats.snr_avg + snr_curr) / 2;     /* update the average SNR value */
    } else {
      stats.snr_avg = snr_curr;
    }
  }
}


static void elwb_run(void)
{
  static elwb_schedule_t  schedule;
  static elwb_time_t      start_of_next_round;
  static elwb_time_t      t_start;
  static uint32_t         t_slot;
  static uint64_t         t_slot_ofs;
  static uint32_t         drift_counter = 0;
  static uint32_t         drift_comp = 0;
  static uint32_t         payload_len = 0;
  static uint16_t         payload[(ELWB_CONF_MAX_PKT_LEN + 1) / 2];
  static bool             call_preprocess = false;

  /* variables specific to the host node */
  static uint_fast8_t     schedule_len;
#if ELWB_CONF_DATA_ACK
  static uint8_t          data_ack[(ELWB_CONF_MAX_DATA_SLOTS + 7) / 8] = { 0 };
#endif /* ELWB_CONF_DATA_ACK */

  /* variables specific to a source node */
#if ELWB_CONF_CONT_USE_HFTIMER
  static elwb_time_t      t_ref_hf;
#endif /* ELWB_CONF_CONT_USE_HFTIMER */
  static elwb_syncstate_t sync_state;
  static uint_fast16_t    period_idle;        /* last base period */
  static uint_fast8_t     rand_backoff = 0;
  static bool             node_registered;
#if ELWB_CONF_DATA_ACK
  static uint_fast16_t    my_slots = 0;
#endif /* ELWB_CONF_DATA_ACK */

  /* --- INIT --- */
  if (ELWB_IS_HOST()) {
    schedule_len = elwb_sched_init(&schedule);
    if (!schedule_len) {
      LOG_ERROR("schedule has length 0");
    }
  } else {
    sync_state      = BOOTSTRAP;
    node_registered = false;
  }

  start_of_next_round = ELWB_TIMER_LAST_EXP();

  /* --- begin MAIN LOOP for eLWB --- */
  while (elwb_running) {

    /* --- PREPROCESS --- */
  #if ELWB_CONF_T_PREPROCESS
    if (call_preprocess) {    /* call the preprocess task? */
      if (pre_task) {
        ELWB_TASK_NOTIFY(pre_task);
        /* note: this is cooperative multitasking, the pre-task must complete within ELWB_CONF_T_PREPROCESS time */
      }
      start_of_next_round += ELWB_CONF_T_PREPROCESS;
      ELWB_WAIT_UNTIL(start_of_next_round);
      call_preprocess = false;
    }
  #endif /* ELWB_CONF_T_PREPROCESS */

    /* --- COMMUNICATION ROUND STARTS --- */

    if (ELWB_IS_HOST()) {
      /* at this point start_of_next_round and ELWB_TIMER_LAST_EXP() should be the same. However, if the wakeup time is in the past (for whatever reason),
       * the lptimer will set last expiration to the current time and thus prevents that the host gets stuck in a "wakeup too late" loop */
      t_start = ELWB_TIMER_LAST_EXP();

      /* --- SEND SCHEDULE --- */
      ELWB_SEND_SCHED();

      if (ELWB_SCHED_IS_FIRST(&schedule)) {
        /* sync point */
        network_time = schedule.time;
        last_synced  = t_start;
        /* calculate the reference offset for the source nodes (time between t_start and the tx marker) */
        stats.ref_ofs = (stats.ref_ofs + (ELWB_GLOSSY_GET_T_REF() - t_start)) / 2;
        elwb_sched_set_time_offset(stats.ref_ofs);
      }

    } else {

      /* --- RECEIVE SCHEDULE --- */
      payload_len = 0;
      if (sync_state == BOOTSTRAP) {
        ELWB_WAIT_UNTIL(ELWB_TIMER_NOW() + ELWB_CONF_T_GUARD_ROUND);    /* this is required, otherwise we get "wakeup time in the past" warnings */
        while (elwb_running) {
          schedule.n_slots = 0;   /* reset */
          stats.bootstrap_cnt++;
          elwb_time_t bootstrap_started = ELWB_TIMER_NOW();
          LOG_INFO("bootstrap");
          /* synchronize first! wait for the first schedule... */
          do {
            ELWB_RCV_SCHED();
            if ((ELWB_TIMER_NOW() - bootstrap_started) >= ELWB_CONF_BOOTSTRAP_TIMEOUT) {
              break;
            }
          } while (elwb_running && !ELWB_GLOSSY_IS_T_REF_UPDATED() && !ELWB_SCHED_IS_FIRST(&schedule));
          /* exit bootstrap mode if schedule received, exit bootstrap state */
          if (ELWB_GLOSSY_IS_T_REF_UPDATED()) {
            break;
          }
          /* go to sleep for ELWB_CONF_T_DEEPSLEEP ticks */
          stats.sleep_cnt++;
          LOG_WARNING("timeout");
          /* poll the post process */
          if (post_task) {
            ELWB_TASK_NOTIFY(post_task);
          }
          if (listen_timeout_cb) {
            listen_timeout_cb();
          }
          ELWB_WAIT_UNTIL(ELWB_TIMER_NOW() + ELWB_CONF_T_DEEPSLEEP);
        }
      } else {
        ELWB_RCV_SCHED();
      }

      if (ELWB_GLOSSY_IS_T_REF_UPDATED()) {      /* schedule received? */
    #if ELWB_CONF_SCHED_CRC
        /* check the CRC */
        payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
        uint16_t pkt_crc = ((uint16_t)*((uint8_t*)&schedule + payload_len - 1)) << 8 |
                           *((uint8_t*)&schedule + payload_len - 2);
        if (crc16((uint8_t*)&schedule, payload_len - 2, 0) != pkt_crc) {
          /* not supposed to happen => go back to bootstrap */
          LOG_ERROR("invalid CRC for eLWB schedule");
          sync_state = BOOTSTRAP;
          continue;
        }
    #endif /* ELWB_CONF_SCHED_CRC */
        /* update the sync state machine */
        sync_state = next_state[EVT_SCHED_RCVD][sync_state];
    #if ELWB_CONF_CONT_USE_HFTIMER
        /* also store the HF timestamp in case LF is used for slot wakeups */
        t_ref_hf = ELWB_GLOSSY_GET_T_REF_HF();
    #endif /* ELWB_CONF_CONT_USE_HFTIMER */
        if (ELWB_SCHED_IS_FIRST(&schedule)) {
          t_start = ELWB_GLOSSY_GET_T_REF();
          /* do some basic drift estimation:
           * measured elapsed time minus effective elapsed time (given by host) */
          int32_t elapsed_network_us = (schedule.time - network_time);   // NOTE: max. difference is ~2100s
          int32_t elapsed_local_us   = (t_start - last_synced) * 1000000 / ELWB_TIMER_SECOND;
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
          last_synced  = t_start;
        } else {
          /* just use the previous wakeup time as start time */
          t_start = start_of_next_round + ELWB_CONF_T_GUARD_ROUND;
        }
        /* update stats */
        elwb_update_rssi_snr();

      } else {
        /* update the sync state machine */
        sync_state = next_state[EVT_SCHED_MISSED][sync_state];
        if (sync_state == BOOTSTRAP) {
          call_preprocess = false;
          continue;
        }
        stats.unsynced_cnt++;
        LOG_WARNING("schedule missed");
        /* we can only estimate t_ref */
        if (!ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
          /* missed schedule was during a contention/data round -> reset t_ref */
          t_start = last_synced;
          /* mark as 'idle state' such that other processes can run */
          ELWB_SCHED_SET_STATE_IDLE(&schedule);
        } else {
          /* missed schedule is at beginning of a round */
          t_start = start_of_next_round;
        }
        schedule.period = period_idle;  /* reset period to idle period */
      }

      /* permission to participate in this round? */
      if (sync_state != SYNCED) {
        goto end_of_round;
      }

      /* schedule sanity check (#slots mustn't exceed the compile-time fixed max. # slots!) */
      if (ELWB_SCHED_N_SLOTS(&schedule) > ELWB_SCHED_MAX_SLOTS) {
        LOG_ERROR("n_slots exceeds limit!");
        ELWB_SCHED_CLR_SLOTS(&schedule);
        schedule.n_slots += ELWB_SCHED_MAX_SLOTS;
      }
    }
    t_slot_ofs = t_start + (ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP);

    /* --- DATA SLOTS --- */

    if (ELWB_SCHED_HAS_SLOTS(&schedule)) {
      bool is_data_round = ELWB_SCHED_HAS_DATA_SLOTS(&schedule);

    #if ELWB_CONF_SCHED_COMPRESS
      elwb_sched_uncompress((uint8_t*)schedule.slot, ELWB_SCHED_N_SLOTS(&schedule));
    #endif /* ELWB_CONF_SCHED_COMPRESS */

      /* set the slot duration */
      if (is_data_round) {
        t_slot   = ELWB_CONF_T_DATA;
  #if ELWB_CONF_DATA_ACK
        my_slots = 0xffff;
  #endif /* ELWB_CONF_DATA_ACK */
      } else {
        /* it's a request round */
        t_slot          = ELWB_CONF_T_CONT;
        node_registered = false;
        rand_backoff    = 0;        /* reset, contention was successful */
      }
      /* loop through all slots in this round */
      uint32_t slot_idx;
      for (slot_idx = 0; slot_idx < ELWB_SCHED_N_SLOTS(&schedule); slot_idx++) {

        /* note: slots with node ID 0 belong to the host */
        bool is_initiator = (schedule.slot[slot_idx] == NODE_ID) || (ELWB_IS_HOST() && schedule.slot[slot_idx] == 0);
        if (is_initiator) {
          node_registered = true;
          /* send a data packet (if there is any) */
          if (ELWB_QUEUE_SIZE(tx_queue) > 0) {
            /* request round? -> only relevant for the source node */
            if (!ELWB_IS_HOST() && !is_data_round) {
              payload_len = ELWB_REQ_PKT_LEN;
              /* request as many data slots as there are packets in the queue */
              payload[0] = ELWB_QUEUE_SIZE(tx_queue);
            } else {
              /* prepare a data packet for dissemination */
              payload_len = 0;
              if (ELWB_QUEUE_POP(tx_queue, payload)) {
                payload_len = ELWB_PAYLOAD_LEN(payload);
                /* sanity check for packet size */
                if (payload_len > ELWB_CONF_MAX_PKT_LEN) {
                  LOG_ERROR("invalid payload length detected");
                  payload_len = 0;
                }
              }
            }
            /* send the packet */
            if (payload_len) {
    #if ELWB_CONF_DATA_ACK
              /* only source nodes receive a D-ACK */
              if (!ELWB_IS_HOST() && is_data_round) {
                if (my_slots == 0xffff) {
                  my_slots = (slot_idx << 8);   /* store the index of the first assigned slot in the upper 8 bytes */
                }
                my_slots++;
                /* copy the packet into the queue for retransmission (in case we don't receive a D-ACK for this packet) */
                if (!ELWB_QUEUE_PUSH(re_tx_queue, payload)) {
                  LOG_ERROR("failed to insert packet into retransmit queue");
                }
              }
    #endif /* ELWB_CONF_DATA_ACK */
              /* wait until the data slot starts */
              ELWB_WAIT_UNTIL(t_slot_ofs);
              ELWB_SEND_PACKET();
              if (is_data_round) {
                stats.pkt_sent++;   /* only count data packets */
                LOG_VERBOSE("packet sent (%lub)", payload_len);
              }
            }

          } else if (is_data_round) {
            LOG_VERBOSE("no message to send (data slot ignored)");
          }

        } else {
          /* not the initiator -> receive / relay packets */
          payload_len = 0;
          if (!is_data_round) {
            /* the payload length is known in the request round */
            payload_len = ELWB_REQ_PKT_LEN;
          }
          memset(payload, 0, sizeof(payload));    /* clear payload before receiving the packet */
          ELWB_WAIT_UNTIL(t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
          ELWB_RCV_PACKET();
          payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
          if (ELWB_GLOSSY_RX_CNT()) {                   /* data received? */
            if (is_data_round) {
              /* check whether to keep this packet */
              bool keep_packet = ELWB_IS_SINK() || ELWB_RCV_PKT_FILTER();
              if (!ELWB_IS_HOST()) {
                /* source nodes keep all packets received from the host */
                keep_packet |= (schedule.slot[slot_idx] == DPP_DEVICE_ID_SINK) || (schedule.slot[slot_idx] == host_id);
              }
              if (keep_packet) {
                LOG_VERBOSE("data received from node %u (%lub)", schedule.slot[slot_idx], payload_len);
                if (ELWB_QUEUE_PUSH(rx_queue, payload)) {
                  stats.pkt_rcvd++;
        #if ELWB_CONF_DATA_ACK
                  /* set the corresponding bit in the data ack packet */
                  data_ack[slot_idx >> 3] |= (1 << (slot_idx & 0x07));
        #endif /* ELWB_CONF_DATA_ACK */
                } else {
                  stats.pkt_dropped++;
                  LOG_WARNING("RX queue full, message dropped");
                }
              }
              stats.pkt_cnt++;

            } else if (ELWB_IS_HOST()) {
              /* this is a request packet */
              elwb_sched_process_req(schedule.slot[slot_idx], *payload);
            }
          } else if (is_data_round) {
            LOG_VERBOSE("no data received from node %u", schedule.slot[slot_idx]);
          }
        }
        t_slot_ofs += (t_slot + ELWB_CONF_T_GAP);
      }
    }

  #if ELWB_CONF_DATA_ACK
    /* --- D-ACK SLOT --- */

    if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule) && !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot = ELWB_CONF_T_DACK;
      if (ELWB_IS_HOST()) {
        /* acknowledge each received packet of the last round */
        payload_len = (ELWB_SCHED_N_SLOTS(&schedule) + 7) / 8;
        if (payload_len) {
          memcpy((uint8_t*)payload, data_ack, payload_len);
          ELWB_WAIT_UNTIL(t_slot_ofs);
          ELWB_SEND_PACKET();
          LOG_INFO("D-ACK sent (%u bytes)", payload_len);
        }
        memset(data_ack, 0, (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8);

      } else {
        ELWB_WAIT_UNTIL(t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
        ELWB_RCV_PACKET();                 /* receive data ack */
        payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
        /* only look into the D-ACK packet if we actually sent some data in the previous round */
        if (my_slots != 0xffff) {
          uint32_t first_slot = my_slots >> 8;
          uint32_t num_slots  = my_slots & 0xff;
          if (ELWB_GLOSSY_RX_CNT()) {
            LOG_VERBOSE("D-ACK received");
            memcpy(data_ack, payload, payload_len);
            uint32_t i;
            for (i = 0; i < num_slots; i++) {
              if (ELWB_QUEUE_POP(re_tx_queue, payload)) {
                /* bit not set? => not acknowledged */
                if (!(data_ack[(first_slot + i) >> 3] & (1 << ((first_slot + i) & 0x07)))) {
                  /* resend the packet (re-insert it into the output FIFO) */
                  if (ELWB_QUEUE_PUSH(tx_queue, payload)) {
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
          } else {
            /* requeue all packets */
            while (ELWB_QUEUE_POP(re_tx_queue, payload)) {
              if (!ELWB_QUEUE_PUSH(tx_queue, payload)) {
                LOG_ERROR("failed to requeue packet");
                break;
              }
            }
            LOG_WARNING("D-ACK pkt missed, %u pkt requeued", num_slots);
          }
          my_slots = 0xffff;
        }
        ELWB_QUEUE_CLEAR(re_tx_queue);  /* make sure the retransmit queue is empty */
      }
      t_slot_ofs += (ELWB_CONF_T_DACK + ELWB_CONF_T_GAP);
    }
  #endif /* ELWB_CONF_DATA_ACK */

    /* --- CONTENTION SLOT --- */

    /* is there a contention slot in this round? */
    if (ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot      = ELWB_CONF_T_CONT;
      payload_len = ELWB_REQ_PKT_LEN;
      payload[0]  = 0;

      /* if there is data in the output buffer, then request a slot */
      if (!ELWB_IS_HOST() &&
          (ELWB_QUEUE_SIZE(tx_queue) >= ELWB_CONF_CONT_TH) &&
          rand_backoff == 0) {
        /* node not yet registered? -> include node ID in the request */
        if (!node_registered) {
          payload[0] = NODE_ID;
          LOG_INFO("transmitting node ID");
        }
  #if ELWB_CONF_CONT_USE_HFTIMER
        /* contention slot requires precise timing: better to use HF timer for this wake-up! */
        ELWB_HFTIMER_SCHEDULE(t_ref_hf + (t_slot_ofs - t_start) * RTIMER_HF_LF_RATIO, elwb_notify);
        ELWB_SUSPENDED();
        ELWB_TASK_YIELD();
        ELWB_RESUMED();
  #else /* ELWB_CONF_CONT_USE_HFTIMER */
        /* wait until the contention slot starts */
        ELWB_WAIT_UNTIL(t_slot_ofs);
  #endif /* ELWB_CONF_CONT_USE_HFTIMER */
        ELWB_SEND_PACKET();
        /* set random backoff time between 0 and 3 */
        rand_backoff = (rand() & 0x0003);

      } else {

        /* just receive / relay packets */
        ELWB_WAIT_UNTIL(t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
        ELWB_RCV_PACKET();
        if (rand_backoff) {
          rand_backoff--;
        }
        if (ELWB_IS_HOST()) {
          uint16_t req_id = payload[0];
          if (ELWB_GLOSSY_RX_CNT() && req_id != 0) {
            /* process the request only if there is a valid node ID */
            elwb_sched_process_req(req_id, 0);
          }
          if (ELWB_GLOSSY_SIGNAL_DETECTED()) {
            /* set the period to 0 to notify the scheduler that at least one nodes has data to send */
            schedule.period = 0;
            LOG_VERBOSE("contention detected");
            /* compute 2nd schedule */
            elwb_sched_compute(&schedule, 0);  /* do not allocate slots for host */
            payload[0] = schedule.period;
          } else {
            /* else: no update to schedule needed; set period to 0 to indicate 'no change in period' */
            payload[0] = 0;
          }
          elwb_update_rssi_snr();
        }
      }
      t_slot_ofs += ELWB_CONF_T_CONT + ELWB_CONF_T_GAP;

      /* --- 2ND SCHEDULE (only in case of a contention slot) --- */

      payload_len = ELWB_2ND_SCHED_LEN;
      if (ELWB_IS_HOST()) {
        ELWB_WAIT_UNTIL(t_slot_ofs);
        ELWB_SEND_PACKET();    /* send as normal packet without sync */
      } else {
        ELWB_WAIT_UNTIL(t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
        ELWB_RCV_PACKET();
        if (ELWB_GLOSSY_RX_CNT()) {         /* packet received? */
          if (payload[0] != 0) {            /* zero means no change */
            schedule.period  = payload[0];  /* extract updated period */
            schedule.n_slots = 0;
          } /* else: all good, no need to change anything */
        } else {
          LOG_WARNING("2nd schedule missed");
        }
      }
    }

end_of_round:

    /* --- COMMUNICATION ROUND ENDS --- */

    if (ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      if (ELWB_IS_HOST()) {
        LOG_INFO("%llu | period: %lus, slots: %u, pkt cnt (rx/tx/drop/all): %lu/%lu/%lu/%lu, rssi: %ddBm",
                 network_time,
                 elwb_sched_get_period(),
                 ELWB_SCHED_N_SLOTS(&schedule),
                 stats.pkt_rcvd,
                 stats.pkt_sent,
                 stats.pkt_cnt,
                 stats.pkt_dropped,
                 stats.rssi_avg);
      } else {
        /* print out some stats (note: takes ~2ms to compose this string!) */
        LOG_INFO("%s %llu | period: %us, slots: %u, pkt cnt (rx/tx/ack/drop/all): %lu/%lu/%lu/%lu/%lu, usync: %lu/%lu, drift: %ld, rssi: %ddBm",
                 elwb_syncstate_to_string[sync_state],
                 schedule.time,
                 schedule.period / ELWB_PERIOD_SCALE,
                 ELWB_SCHED_N_SLOTS(&schedule),
                 stats.pkt_rcvd,
                 stats.pkt_sent,
                 stats.pkt_cnt,
                 stats.pkt_ack,
                 stats.pkt_dropped,
                 stats.unsynced_cnt,
                 stats.bootstrap_cnt,
                 stats.drift,
                 stats.rssi_avg);
      }
      /* poll the post process */
      if (post_task) {
        ELWB_TASK_NOTIFY(post_task);
      }
    #if ELWB_CONF_T_PREPROCESS
      call_preprocess = true;
    #endif /* ELWB_CONF_T_PREPROCESS */
    }

    uint32_t round_ticks = (uint32_t)schedule.period * ELWB_TIMER_SECOND / ELWB_PERIOD_SCALE;
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));
    ELWB_SCHED_CLR_SLOTS(&schedule);
    if (ELWB_IS_HOST()) {
      /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
      schedule_len = elwb_sched_compute(&schedule, ELWB_QUEUE_SIZE(tx_queue));
    }

    /* calculate extra ticks for drift compensation */
    if (stats.drift != 0) {
      drift_counter    += round_ticks;
      int32_t drift_div = 1000000 / stats.drift;
      drift_comp        = drift_counter / drift_div;
      drift_counter    -= drift_comp * drift_div;
      //LOG_VERBOSE("extra ticks: %lu, counter: %lu", drift_comp, drift_counter);
    } else {
      drift_counter = 0;
      drift_comp = 0;
    }

    /* schedule the wakeup for the next round */
    start_of_next_round = t_start + round_ticks + drift_comp;
    if (!ELWB_IS_HOST()) {
      start_of_next_round -= ELWB_CONF_T_GUARD_ROUND;   /* add guard time on a source node */
    }
    if (call_preprocess) {
      start_of_next_round -= ELWB_CONF_T_PREPROCESS;    /* wake up earlier such that the pre task can run */
    }
    //LOG_VERBOSE("wakeup in %d ticks", (int32_t)(start_of_next_round - lptimer_now()));
    ELWB_WAIT_UNTIL(start_of_next_round);
  }

  LOG_INFO("stopped");
}


void elwb_init(void* elwb_task,
               void* pre_elwb_task,
               void* post_elwb_task,
               void* in_queue_handle,
               void* out_queue_handle,
               void* retransmit_queue_handle,
               void* listen_timeout_callback)
{
  if (!in_queue_handle || !out_queue_handle || !elwb_task) {
    LOG_ERROR("invalid parameters");
    return;
  }

#if ELWB_CONF_DATA_ACK
  if (!retransmit_queue_handle) {
    LOG_ERROR("invalid parameters");
    return;
  }
#endif /* ELWB_CONF_DATA_ACK */

  task_handle  = elwb_task;
  pre_task     = pre_elwb_task;
  post_task    = post_elwb_task;
  rx_queue     = in_queue_handle;
  tx_queue     = out_queue_handle;
  re_tx_queue  = retransmit_queue_handle;
  listen_timeout_cb = listen_timeout_callback;
  elwb_running = true;

  /* clear all queues */
  ELWB_QUEUE_CLEAR(rx_queue);
  ELWB_QUEUE_CLEAR(tx_queue);
#if ELWB_CONF_DATA_ACK
  ELWB_QUEUE_CLEAR(re_tx_queue);
#endif /* ELWB_CONF_DATA_ACK */


  memset(&stats, 0, sizeof(elwb_stats_t));
}


void elwb_start(uint32_t _host_id)
{
  host_id = _host_id;
  if (ELWB_IS_HOST()) {
    LOG_INFO("host node");
  } else {
    LOG_INFO("source node");
  }

  LOG_INFO("pkt_len: %u, slots: %u, n_tx: %u, t_sched: %lu, t_data: %lu, t_cont: %lu",
           ELWB_CONF_MAX_PKT_LEN,
           ELWB_CONF_MAX_DATA_SLOTS,
           ELWB_CONF_N_TX,
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_SCHED),
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_DATA),
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_CONT));

  /* instead of calling elwb_run(), schedule the start */
#if ELWB_CONF_STARTUP_DELAY > 0
  ELWB_WAIT_UNTIL(ELWB_TIMER_NOW() + ELWB_CONF_STARTUP_DELAY * LPTIMER_SECOND / 1000);
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
