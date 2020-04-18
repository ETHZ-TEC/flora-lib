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


static const 
elwb_syncstate_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{/* STATES:                                         EVENTS:         */
 /* BOOTSTRAP, SYNCED,   UNSYNCED,  UNSYNCED2                       */
  { SYNCED,    SYNCED,   SYNCED,    SYNCED    }, /* schedule rcvd   */
  { BOOTSTRAP, UNSYNCED, UNSYNCED2, BOOTSTRAP }  /* schedule missed */
};
static const char* elwb_syncstate_to_string[NUM_OF_SYNC_STATES] = {
  "BOOTSTRAP", "SYN", "USYN", "USYN2"
};


#ifndef ELWB_RESUMED
  #define ELWB_RESUMED()
  #define ELWB_SUSPENDED()
#endif /* ELWB_RESUMED */


#define ELWB_SEND_SCHED() \
{\
  ELWB_GLOSSY_START(NODE_ID, (uint8_t *)&schedule, schedule_len, ELWB_CONF_N_TX, 1);\
  ELWB_WAIT_UNTIL(lptimer_get() + ELWB_CONF_T_SCHED);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_RCV_SCHED() \
{\
  ELWB_GLOSSY_START(0, (uint8_t *)&schedule, payload_len, ELWB_CONF_N_TX, 1);\
  ELWB_WAIT_UNTIL(lptimer_get() + ELWB_CONF_T_SCHED + ELWB_CONF_T_GUARD_ROUND);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_SEND_PACKET() \
{\
  ELWB_GLOSSY_START(NODE_ID, (uint8_t*)payload, payload_len, ELWB_CONF_N_TX, 0);\
  ELWB_WAIT_UNTIL(lptimer_get() + t_slot);\
  ELWB_GLOSSY_STOP();\
}
#define ELWB_RCV_PACKET() \
{\
  ELWB_GLOSSY_START(0, (uint8_t*)payload, payload_len, ELWB_CONF_N_TX, 0);\
  ELWB_WAIT_UNTIL(lptimer_get() + t_slot + ELWB_CONF_T_GUARD_SLOT);\
  ELWB_GLOSSY_STOP();\
}


#define ELWB_WAIT_UNTIL(time) \
{\
  ELWB_TIMER_SET(time, elwb_notify);\
  ELWB_SUSPENDED();\
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);\
  ELWB_RESUMED();\
}


static void*              post_task;
static void*              pre_task;
static elwb_schedule_t    schedule;
static elwb_stats_t       stats;
static elwb_time_t        last_synced;
static elwb_time_t        start_of_next_round;
static elwb_time_t        network_time;
static uint32_t           t_slot;
static uint32_t           t_slot_ofs;
static void*              task_handle = 0;
static void*              rx_queue = 0;
static void*              tx_queue = 0;
static uint16_t           payload[(ELWB_CONF_MAX_PKT_LEN + 1) / 2];
static uint32_t           payload_len = 0;
static bool               call_preprocess = false;


/* private (not exposed) scheduler functions */
uint32_t elwb_sched_init(elwb_schedule_t* sched);
void     elwb_sched_process_req(uint16_t id,
                                uint32_t n_pkts);
uint32_t elwb_sched_compute(elwb_schedule_t * const sched,
                            uint32_t reserve_slots_host);
bool     elwb_sched_uncompress(uint8_t* compressed_data, uint32_t n_slots);
void     elwb_sched_set_time_offset(uint32_t ofs);

/*
 * This function can be called from an interrupt context to poll the GMW task.
 */
void elwb_notify(void)
{
  if (task_handle) {
    ELWB_ON_WAKEUP();
    xTaskNotifyFromISR(task_handle, 0, eNoAction, 0);
  }
}


#if ELWB_CONF_DATA_ACK
void elwb_requeue_pkt(uint32_t pkt_addr)
{
  /* re-insert the packet into the tx queue */
  /* find an empty spot in the queue */
  uint32_t new_pkt_addr = fifo_put(&tx_queue);
  if (new_pkt_addr != FIFO_ERROR) {
    if (new_pkt_addr == pkt_addr) {
      return; /* same address? -> nothing to do */
    }
    memcpy((uint8_t*)(uint16_t)new_pkt_addr, (uint8_t*)(uint16_t)pkt_addr, 
           sizeof(elwb_queue_elem_t));
    LOG_INFO_CONST("packet requeued");
  } else {
    stats.txbuf_drop++;
    LOG_ERROR_CONST("requeue failed, out queue full");
  }
}
#endif /* ELWB_CONF_DATA_ACK */


const elwb_stats_t * const elwb_get_stats(void)
{
  return &stats;
}


void elwb_get_time(elwb_time_t* time, elwb_time_t* rx_timestamp)
{
  if (network_time) {
    *time = network_time;
  }
  if (rx_timestamp) {
    *rx_timestamp = last_synced;
  }
}


elwb_time_t elwb_get_timestamp(void)
{
  return network_time + (ELWB_RTIMER_NOW() - last_synced) * (1000000 - stats.drift) / (ELWB_TIMER_SECOND);
}


/**
 * @brief thread of the host node
 */
void elwb_host_run(void)
{
  /* variables specific to the host (all must be static) */
  static elwb_time_t    t_start;
  static uint_fast16_t  curr_period = 0;
  static uint_fast8_t   schedule_len;
  static uint32_t       t_ref_ofs = 0;
#if ELWB_CONF_DATA_ACK
  static uint8_t  data_ack[(ELWB_CONF_MAX_DATA_SLOTS + 7) / 8] = { 0 };
#endif /* ELWB_CONF_DATA_ACK */
  
  schedule_len = elwb_sched_init(&schedule);
  if (!schedule_len) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }

  while (1) {
  
  #if ELWB_CONF_T_PREPROCESS
    if (call_preprocess) {
      if (pre_task) {
        xTaskNotify(pre_task, 0, eNoAction);
      }
      start_of_next_round += ELWB_CONF_T_PREPROCESS;
      ELWB_WAIT_UNTIL(start_of_next_round);
      call_preprocess = false;
    }
  #endif /* ELWB_CONF_T_PREPROCESS */

    /* --- COMMUNICATION ROUND STARTS --- */

    t_start = start_of_next_round;

    /* --- SEND SCHEDULE --- */
    ELWB_SEND_SCHED();
   
    if (ELWB_SCHED_IS_FIRST(&schedule)) {
      /* sync point */
      network_time = schedule.time;
      last_synced  = ELWB_GLOSSY_GET_T_REF();  // was t_start before
      if (t_start) {
        t_ref_ofs = (t_ref_ofs + (last_synced - t_start)) / 2;
        elwb_sched_set_time_offset(t_ref_ofs);
      }
    }
    t_slot_ofs = (ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP);
    
    /* --- DATA SLOTS --- */
    
    if (ELWB_SCHED_HAS_SLOTS(&schedule)) {

    #if ELWB_CONF_SCHED_COMPRESS
      elwb_sched_uncompress((uint8_t*)schedule.slot,
                            ELWB_SCHED_N_SLOTS(&schedule));
    #endif /* ELWB_CONF_SCHED_COMPRESS */

      if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
        /* this is a data round */
        t_slot     = ELWB_CONF_T_DATA;
        /* calculate the load (moving average over 10 rounds) */
        stats.load = (stats.load * 9 + 
                      ELWB_SCHED_N_SLOTS(&schedule) * 100 /
                      ELWB_CONF_MAX_DATA_SLOTS) / 10;
      } else {
        t_slot = ELWB_CONF_T_CONT;
      }
      uint16_t i;
      for (i = 0; i < ELWB_SCHED_N_SLOTS(&schedule); i++) {
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if (schedule.slot[i] == 0 || schedule.slot[i] == NODE_ID) {
          /* send a data packet (if there is any) */
          if (ELWB_QUEUE_SIZE(tx_queue)) {
            ELWB_QUEUE_POP(tx_queue, payload);
            payload_len = ELWB_PAYLOAD_LEN(payload);
          }
          if (payload_len && payload_len <= ELWB_CONF_MAX_PKT_LEN) {
            /* wait until the data slot starts */
            ELWB_WAIT_UNTIL(t_start + t_slot_ofs);
            ELWB_SEND_PACKET();
            stats.pkt_snd++;
          }
        } else {
          if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
            payload_len = 0;
          } else {
            /* it's a request round */
            payload_len = ELWB_REQ_PKT_LEN;
          }
          /* wait until the data slot starts */
          ELWB_WAIT_UNTIL(t_start + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
          ELWB_RCV_PACKET();  /* receive a data packet */
          payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
          if (ELWB_GLOSSY_RX_CNT()) {
            if (!ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
              elwb_sched_process_req(schedule.slot[i], *payload);
            } else {
              LOG_INFO("data received from node %u (%lub)", schedule.slot[i], payload_len);
              if (ELWB_QUEUE_PUSH(rx_queue, payload)) {
                stats.pkt_fwd++;
  #if ELWB_CONF_DATA_ACK
                /* set the corresponding bit in the data ack packet */
                data_ack[i >> 3] |= (1 << (i & 0x07));
  #endif /* ELWB_CONF_DATA_ACK */
              } else {
                LOG_WARNING_CONST("RX queue full, message dropped");
              }
              stats.pkt_rcv++;
            }
          } else {
            LOG_INFO("no data received from node %u", schedule.slot[i]);
          }
        }
        t_slot_ofs += (t_slot + ELWB_CONF_T_GAP);
      }
    }
    
#if ELWB_CONF_DATA_ACK
    /* --- D-ACK SLOT --- */
    
    if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule) &&
       !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      /* acknowledge each received packet of the last round */
      payload_len = (ELWB_SCHED_N_SLOTS(&schedule) + 7) / 8;
      if (payload_len) {
        memcpy((uint8_t*)payload, data_ack, payload_len);
        t_slot = ELWB_CONF_T_DACK;
        ELWB_WAIT_UNTIL(t_start + t_slot_ofs);
        ELWB_SEND_PACKET();
        LOG_INFO("D-ACK sent (%u bytes)", payload_len);
      }
      t_slot_ofs += (ELWB_CONF_T_DACK + ELWB_CONF_T_GAP);
      memset(data_ack, 0, (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8);
    }
#endif /* ELWB_CONF_DATA_ACK */
    
    /* --- CONTENTION SLOT --- */
    
    if (ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      t_slot = ELWB_CONF_T_CONT;
      payload_len = ELWB_REQ_PKT_LEN;
      payload[0] = payload[1] = 0;
      /* wait until the slot starts, then receive the packet */
      ELWB_WAIT_UNTIL(t_start + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
      ELWB_RCV_PACKET();
      uint32_t req_id = *payload;
      if (ELWB_GLOSSY_RX_CNT() && req_id != 0) {
        /* process the request only if there is a valid node ID */
        elwb_sched_process_req(req_id, 0);
      }
      if (ELWB_GLOSSY_SIGNAL_DETECTED()) {
        /* set the period to 0 to notify the scheduler that at 
         * least one nodes has data to send */
        schedule.period = 0;
        LOG_VERBOSE_CONST("contention detected");
        
        /* compute 2nd schedule */
        elwb_sched_compute(&schedule, 0);  /* do not allocate slots for host */
        payload[0] = schedule.period;
      } else {
        /* else: no update to schedule needed; set period to 0 to indicate to 
         * source nodes that there is no change in period (and no request round
         * following) */
        payload[0] = 0;
      }
      t_slot_ofs += ELWB_CONF_T_CONT + ELWB_CONF_T_GAP;
  
      /* --- SEND 2ND SCHEDULE --- */
      
      /* (just a 2-byte packet to indicate a change in the round period) */    
      /* send the 2nd schedule only if there was a contention slot */
      payload_len = ELWB_2ND_SCHED_LEN;
      ELWB_WAIT_UNTIL(t_start + t_slot_ofs);
      ELWB_SEND_PACKET();    /* send as normal packet! saves energy */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
    
    /* poll the other processes to allow them to run after the eLWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    if (ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      /* print out some stats */
      LOG_INFO("%llu | period: %lus, slots: %u, pck cnt: %u, rcvd: %u, sent: %u, ref ofs: %lu",
               network_time,
               elwb_sched_get_period(),
               ELWB_SCHED_N_SLOTS(&schedule),
               stats.pkt_rcv,
               stats.pkt_fwd,
               stats.pkt_snd,
               t_ref_ofs);

      if (post_task) {
        xTaskNotify(post_task, 0, eNoAction);
      }
    #if ELWB_CONF_T_PREPROCESS
      call_preprocess = true;
    #endif /* ELWB_CONF_T_PREPROCESS */
    }
    
    /* --- COMPUTE NEW SCHEDULE (for the next round) --- */
    curr_period  = schedule.period;   /* required to schedule the next wake-up */
    schedule_len = elwb_sched_compute(&schedule, ELWB_QUEUE_SIZE(tx_queue));
    
    /* suspend this task and wait for the next round */
    start_of_next_round = t_start + (uint32_t)curr_period * ELWB_TIMER_SECOND / ELWB_PERIOD_SCALE;
    if (call_preprocess) {
      start_of_next_round -= ELWB_CONF_T_PREPROCESS;
    }
    if (lptimer_now() > start_of_next_round) {
      LOG_ERROR_CONST("wakeup time is in the past");
    }
    ELWB_WAIT_UNTIL(start_of_next_round);
  }
}


/**
 * @brief source node
 */
void elwb_src_run(void)
{
  /* variables specific to the source node (all must be static) */
  static elwb_time_t      t_ref;
#if ELWB_CONF_CONT_USE_HFTIMER
  static elwb_time_t      t_ref_hf,
                          rt_time_last;
#endif /* ELWB_CONF_CONT_USE_HFTIMER */
  static elwb_syncstate_t sync_state;
  static uint8_t          node_registered;
  static uint16_t         period_idle;        /* last base period */
#if ELWB_CONF_DATA_ACK
  static uint8_t          first_slot = 0xff,
                          num_slots  = 0;
#endif /* ELWB_CONF_DATA_ACK */
  static uint8_t          rand_backoff = 0;
  static uint32_t         drift_counter = 0;
  static uint32_t         drift_comp = 0;

  sync_state      = BOOTSTRAP;
  node_registered = 0;
  
  while (1) {
    
  #if ELWB_CONF_T_PREPROCESS
    if (call_preprocess) {
      if (pre_task) {
        xTaskNotify(pre_task, 0, eNoAction);
      }
      start_of_next_round += ELWB_CONF_T_PREPROCESS;
      ELWB_WAIT_UNTIL(start_of_next_round);
      call_preprocess = false;
    }
  #endif /* ELWB_CONF_T_PREPROCESS */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
    /* --- RECEIVE SCHEDULE --- */
    
    payload_len = 0;
    if (sync_state == BOOTSTRAP) {
      while (1) {
        schedule.n_slots = 0;   /* reset */
        stats.bootstrap_cnt++;
        elwb_time_t bootstrap_started = ELWB_RTIMER_NOW();
        LOG_INFO_CONST("bootstrap");
        /* synchronize first! wait for the first schedule... */
        do {
  #if WATCHDOG_CONF_ON && !WATCHDOG_CONF_RESET_ON_TA1IFG
          watchdog_reset();
  #endif /* WATCHDOG_CONF_ON */
          ELWB_RCV_SCHED();
        } while (!ELWB_GLOSSY_IS_T_REF_UPDATED() && ((ELWB_RTIMER_NOW() - bootstrap_started) < ELWB_CONF_BOOTSTRAP_TIMEOUT));
        if (ELWB_GLOSSY_IS_T_REF_UPDATED()) {
          break;  /* schedule received, exit bootstrap state */
        }
        /* go to sleep for ELWB_CONF_T_DEEPSLEEP ticks */
        stats.sleep_cnt++;
        LOG_WARNING_CONST("timeout");

        /* poll the post process */
        if (post_task) {
          xTaskNotify(post_task, 0, eNoAction);
        }
        ELWB_WAIT_UNTIL(ELWB_RTIMER_NOW() + ELWB_CONF_T_DEEPSLEEP);
      }
    } else {
      ELWB_RCV_SCHED();
    }
    
    /* schedule received? */
    if (ELWB_GLOSSY_IS_T_REF_UPDATED()) {
  #if ELWB_CONF_SCHED_CRC
      /* check the CRC */
      payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
      uint16_t pkt_crc =
                    ((uint16_t)*((uint8_t*)&schedule + payload_len - 1)) << 8 |
                    *((uint8_t*)&schedule + payload_len - 2);
      if (crc16((uint8_t*)&schedule, payload_len - 2, 0) != pkt_crc) {
        /* not supposed to happend, all we can do now is go back to bootstrap
         * since the previous (valid) schedule has been overwritten with a
         * corrupted one */
        EVENT_ERROR(EVENT_CC430_CORRUPTED_SCHEDULE, 0);
        LOG_ERROR_CONST("invalid CRC for eLWB schedule");
        sync_state = BOOTSTRAP;
        continue;
      }
  #endif /* ELWB_CONF_SCHED_CRC */
      /* update the sync state machine */
      sync_state = next_state[EVT_SCHED_RCVD][sync_state];
      /* subtract const offset to align src and host */
      t_ref = ELWB_GLOSSY_GET_T_REF();
  #if ELWB_CONF_CONT_USE_HFTIMER
      /* also store the HF timestamp in case LF is used for slot wakeups */
      t_ref_hf = ELWB_GLOSSY_GET_T_REF_HF();
  #endif /* ELWB_CONF_CONT_USE_HFTIMER */
      if (ELWB_SCHED_IS_FIRST(&schedule)) {
        /* do some basic drift estimation:
         * measured elapsed time minus effective elapsed time (given by host) */
        int32_t elapsed_network_us = (schedule.time - network_time);   // NOTE: max. difference is ~2100s
        int32_t elapsed_local_us   = (t_ref - last_synced) * 1000000 / ELWB_TIMER_SECOND;
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
      }
    } else {
      /* update the sync state machine */
      sync_state = next_state[EVT_SCHED_MISSED][sync_state];
      if (sync_state == BOOTSTRAP) {
        call_preprocess = false;
        continue;
      }
      stats.unsynced_cnt++;
      LOG_WARNING_CONST("schedule missed");
      /* we can only estimate t_ref and t_ref */
      if (!ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
        /* missed schedule was during a contention/data round -> reset t_ref */
        t_ref = last_synced;
        /* mark as 'idle state' such that other processes can run */
        ELWB_SCHED_SET_STATE_IDLE(&schedule);
      } else {
        /* missed schedule is at beginning of a round: add last period */
        t_ref += schedule.period * ELWB_TIMER_SECOND / ELWB_PERIOD_SCALE + drift_comp;
      }
      schedule.period = period_idle;  /* reset period to idle period */
    }
    
    /* permission to participate in this round? */
    if (sync_state == SYNCED) {
      
      /* sanity check (mustn't exceed the compile-time fixed max. # slots!) */
      if (ELWB_SCHED_N_SLOTS(&schedule) > ELWB_SCHED_MAX_SLOTS) {
        LOG_ERROR_CONST("n_slots exceeds limit!");
        EVENT_ERROR(EVENT_CC430_MEM_OVERFLOW, 1);
        ELWB_SCHED_CLR_SLOTS(&schedule);
        schedule.n_slots += ELWB_SCHED_MAX_SLOTS;
      }
    #if ELWB_CONF_SCHED_COMPRESS
      /* uncompress the schedule */
      elwb_sched_uncompress((uint8_t*)schedule.slot, 
                            ELWB_SCHED_N_SLOTS(&schedule));
    #endif /* ELWB_CONF_SCHED_COMPRESS */
      
      static uint16_t i;
      t_slot_ofs = (ELWB_CONF_T_SCHED + ELWB_CONF_T_GAP);
    
      /* --- DATA SLOTS --- */
      
      if (ELWB_SCHED_HAS_SLOTS(&schedule)) {
        /* set the slot duration */
        if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
          /* this is a data round */
          t_slot = ELWB_CONF_T_DATA;
    #if ELWB_CONF_DATA_ACK
          first_slot = 0xff;
    #endif /* ELWB_CONF_DATA_ACK */
        } else {
          /* it's a request round */
          t_slot = ELWB_CONF_T_CONT;
          node_registered = 0;
          rand_backoff = 0;              /* reset, contention was successful */
        }
        for(i = 0; i < ELWB_SCHED_N_SLOTS(&schedule); i++) {
          if (schedule.slot[i] == NODE_ID) {
            node_registered = 1;
            /* this is our data slot, send a data packet (if there is any) */
            if (ELWB_QUEUE_SIZE(tx_queue) > 0) {
              if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
                /* data round */
                payload_len = 0;
                if (ELWB_QUEUE_POP(tx_queue, payload)) {
                  payload_len = ELWB_PAYLOAD_LEN(payload);
                  /* sanitiy check */
                  if (payload_len > ELWB_CONF_MAX_PKT_LEN) {
                    LOG_ERROR_CONST("invalid payload length detected");
                    payload_len = 0;
                  }
                }
                if (payload_len) {
    #if ELWB_CONF_DATA_ACK
                  if (first_slot == 0xff) {
                    first_slot = i;
                    num_slots = 0;
                  }
                  num_slots++;
    #endif /* ELWB_CONF_DATA_ACK */
                  stats.pkt_snd++;
                }
              } else {
                payload_len = ELWB_REQ_PKT_LEN;
                /* request as many data slots as there are packets in the queue */
                payload[0] = ELWB_QUEUE_SIZE(tx_queue);
                stats.load = payload[0];
              }
              if (payload_len) {
                ELWB_WAIT_UNTIL(t_ref + t_slot_ofs);
                ELWB_SEND_PACKET();
                LOG_VERBOSE("packet sent (%lub)", payload_len);
              }
            } else {
              LOG_VERBOSE_CONST("no message to send (data slot ignored)");
            }
          } else
          {
            payload_len = 0;
            if (!ELWB_SCHED_HAS_DATA_SLOTS(&schedule)) {
              /* the payload length is known in the request round */
              payload_len = ELWB_REQ_PKT_LEN;
            }
            /* receive a data packet */
            ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
            ELWB_RCV_PACKET();
            payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
            if (ELWB_SCHED_HAS_DATA_SLOTS(&schedule) && ELWB_GLOSSY_RX_CNT()) {
             /* forward the packet to the application task if the initiator was
              * the host or sink or if the custom forward filter is 'true' */
              if (ELWB_CONF_SRC_PKT_FILTER(payload)) {
                if (ELWB_QUEUE_PUSH(rx_queue, payload)) {
                  stats.pkt_fwd++;
                } else {
                  LOG_WARNING_CONST("RX queue full, message dropped");
                }
              }
              stats.pkt_rcv++;
            }
          }
          t_slot_ofs += (t_slot + ELWB_CONF_T_GAP);
        }
      }
      
  #if ELWB_CONF_DATA_ACK
      /* --- D-ACK SLOT --- */
      
      if (!(cfg.dbg_flags & 0x02) &&
         ELWB_SCHED_HAS_DATA_SLOTS(&schedule) &&
         !ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot = ELWB_CONF_T_DACK;
        payload_len = GLOSSY_UNKNOWN_PAYLOAD_LEN;
        ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
        ELWB_RCV_PACKET();                 /* receive data ack */
        payload_len = ELWB_GLOSSY_GET_PAYLOAD_LEN();
        /* only look into the D-ACK packet if we actually sent some data in the
         * previous round */
        if (first_slot != 0xff) {
          if (ELWB_GLOSSY_RX_CNT()) {
            ELWB_VERBOSE("D-ACK rcvd (%ub)", payload_len);
            uint8_t* data_acks = (uint8_t*)payload;
            for(i = 0; i < num_slots; i++) {
              /* bit not set? => not acknowledged */
              if (!(data_acks[(first_slot + i) >> 3] &
                  (1 << ((first_slot + i) & 0x07)))) {
                /* resend the packet (re-insert it into the output FIFO) */
                uint32_t addr = fifo_elem_addr_rel(&tx_queue,
                                                   (int16_t)i - num_slots);
                elwb_requeue_pkt(addr);
              } else {
                stats.pkt_ack++;
              }
            }
          } else {
            /* requeue all */
            fifo_restore(&tx_queue, num_slots);
            LOG_WARNING("D-ACK pkt missed, %u pkt requeued", num_slots);
          }
          first_slot  = 0xff;
          num_slots   = 0;
        }
        t_slot_ofs += (ELWB_CONF_T_DACK + ELWB_CONF_T_GAP);
      }
  #endif /* ELWB_CONF_DATA_ACK */
      
      /* --- CONTENTION SLOT --- */
      
      /* is there a contention slot in this round? */
      if (ELWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        t_slot      = ELWB_CONF_T_CONT;
        payload_len = ELWB_REQ_PKT_LEN;
        if ((ELWB_QUEUE_SIZE(tx_queue) >= ELWB_CONF_CONT_TH) && rand_backoff == 0) {
          /* if there is data in the output buffer, then request a slot */
          /* a slot request packet always looks the same */
          /* include the node ID in case this is the first request */
          payload[0] = 0;
          if (!node_registered) {
            payload[0] = NODE_ID;
            LOG_INFO_CONST("transmitting node ID");
          }
          /* contention slot requires precise timing: use HF timer for this
           * wake-up! */
  #if ELWB_CONF_CONT_USE_HFTIMER
          /* wait until the contention slot starts */
          ELWB_HFTIMER_SCHEDULE(t_ref_hf + t_slot_ofs * RTIMER_HF_LF_RATIO, elwb_notify);
          ELWB_SUSPENDED();
          ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
          ELWB_RESUMED();
  #else /* ELWB_CONF_CONT_USE_HFTIMER */
          /* wait until the contention slot starts */
          ELWB_WAIT_UNTIL(t_ref + t_slot_ofs);
  #endif /* ELWB_CONF_CONT_USE_HFTIMER */
          ELWB_SEND_PACKET();
          /* set random backoff time between 0 and 3 */
          rand_backoff = (rand() & 0x0003);
        } else {
          /* no request pending -> just receive / relay packets */
          ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
          ELWB_RCV_PACKET();
          if (rand_backoff) {
            rand_backoff--;
          }
        }
        t_slot_ofs += ELWB_CONF_T_CONT + ELWB_CONF_T_GAP;
        
        /* --- RECEIVE 2ND SCHEDULE --- */
      
        /* only rcv the 2nd schedule if there was a contention slot */
        payload_len = ELWB_2ND_SCHED_LEN;
        ELWB_WAIT_UNTIL(t_ref + t_slot_ofs - ELWB_CONF_T_GUARD_SLOT);
        ELWB_RCV_PACKET();
        if (ELWB_GLOSSY_RX_CNT()) {
          if (payload[0] != 0) {            /* zero means no change */
            schedule.period  = payload[0];  /* extract updated period */
            schedule.n_slots = 0;           /* clear! */
          } /* else: all good, no need to change anything */
        } else {
          LOG_WARNING_CONST("2nd schedule missed");
        }
      }
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */

    /* time for other computations */
    
    if (ELWB_SCHED_IS_STATE_IDLE(&schedule)) {
      /* print out some stats (note: takes ~2ms to compose this string!) */
      LOG_INFO("%s %llu | period: %us, slots: %u, pck cnt: %u, rcvd: %u, sent: %u, acks: %u, usyn: %u, boot: %u, drift: %d",
               elwb_syncstate_to_string[sync_state],
               schedule.time,
               schedule.period / ELWB_PERIOD_SCALE,
               ELWB_SCHED_N_SLOTS(&schedule),
               stats.pkt_rcv,
               stats.pkt_fwd,
               stats.pkt_snd,
               stats.pkt_ack,
               stats.unsynced_cnt,
               stats.bootstrap_cnt,
               stats.drift);

      /* poll the post process */
      if (post_task) {
        xTaskNotify(post_task, 0, eNoAction);
      }
    #if ELWB_CONF_T_PREPROCESS
      call_preprocess = true;
    #endif /* ELWB_CONF_T_PREPROCESS */
    }
    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));
    ELWB_SCHED_CLR_SLOTS(&schedule);
    
    uint32_t round_ticks = (uint32_t)schedule.period * ELWB_TIMER_SECOND / ELWB_PERIOD_SCALE;

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
    start_of_next_round = t_ref + round_ticks - ELWB_CONF_T_GUARD_ROUND + drift_comp;
    if (call_preprocess) {
      start_of_next_round -= ELWB_CONF_T_PREPROCESS;
    }
    if (lptimer_now() > start_of_next_round) {
      LOG_ERROR_CONST("wakeup time is in the past");
    }
    ELWB_WAIT_UNTIL(start_of_next_round);
  }
}


void elwb_start(void* elwb_task,
                void* pre_elwb_task,
                void* post_elwb_task,
                void* in_queue,
                void* out_queue)
{
  if (!in_queue || !out_queue || !elwb_task) {
    LOG_ERROR_CONST("invalid parameters");
    return;
  }

  task_handle = elwb_task;
  pre_task    = pre_elwb_task;
  post_task   = post_elwb_task;
  rx_queue    = in_queue;
  tx_queue    = out_queue;
  
  memset(&stats, 0, sizeof(elwb_stats_t));

  LOG_INFO("pkt_len=%u slots=%u n_tx=%u hops=%u",
           ELWB_CONF_MAX_PKT_LEN,
           ELWB_CONF_MAX_DATA_SLOTS,
           ELWB_CONF_N_TX,
           ELWB_CONF_N_HOPS);

  /* ceil the values (therefore + ELWB_TIMER_SECOND / 1000 - 1) */
  LOG_INFO("slots [ms]: sched=%lu data=%lu cont=%lu",
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_SCHED),
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_DATA),
           (uint32_t)ELWB_TICKS_TO_MS(ELWB_CONF_T_CONT));
  
  if (ELWB_IS_HOST) {
    LOG_INFO_CONST("is host node");
    elwb_host_run();
  } else {
    LOG_INFO_CONST("is source node");
    elwb_src_run();
  }
}

#endif /* ELWB_ENABLE */
