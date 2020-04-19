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

#ifndef PROTOCOL_ELWB_ELWB_H_
#define PROTOCOL_ELWB_ELWB_H_


#ifndef ELWB_ENABLE
#define ELWB_ENABLE               0
#endif /* ELWB_ENABLE */


/* values that need to be defined globally */
#ifndef HOST_ID
#warning "HOST_ID not defined, set to 0"
#define HOST_ID                   0
#endif


/* --------------- START OF CONFIG (default values) ------------------------ */

/* max. packet length */
#ifndef ELWB_CONF_MAX_PKT_LEN
#define ELWB_CONF_MAX_PKT_LEN     DPP_MSG_PKT_LEN
#endif /* ELWB_CONF_MAX_PKT_LEN */

/* additional header bytes that are added to a data packet by the radio driver */
#ifndef ELWB_CONF_RADIO_HDR_LEN
#define ELWB_CONF_RADIO_HDR_LEN   2
#endif /* ELWB_CONF_RADIO_HDR_LEN */

#ifndef ELWB_CONF_N_HOPS
#define ELWB_CONF_N_HOPS          3     /* average number of hops in the network */
#endif /* ELWB_CONF_N_HOPS */

#ifndef ELWB_CONF_N_TX
#define ELWB_CONF_N_TX            3     /* how many times a packet is retransmitted */
#endif /* ELWB_CONF_N_TX */

#ifndef ELWB_CONF_BOOTSTRAP_TIMEOUT
#define ELWB_CONF_BOOTSTRAP_TIMEOUT   (120 * ELWB_TIMER_SECOND)     /* in ticks */
#endif /* ELWB_CONF_BOOTSTRAP_TIMEOUT */

#ifndef ELWB_CONF_MAX_N_NODES
#define ELWB_CONF_MAX_N_NODES     10
#endif /* ELWB_CONF_MAX_N_NODES */

#ifndef ELWB_CONF_MAX_DATA_SLOTS
#define ELWB_CONF_MAX_DATA_SLOTS  10
#endif /* ELWB_CONF_MAX_DATA_SLOTS */

#ifndef ELWB_CONF_MAX_SLOTS_HOST
#define ELWB_CONF_MAX_SLOTS_HOST  (ELWB_CONF_MAX_DATA_SLOTS / 2)
#endif /* ELWB_CONF_MAX_SLOTS_HOST */

/* duration of a schedule slot in ticks, if not defined, the minimum
 * required slot time is calculated based on the network N_HOPS and N_TX */
#ifndef ELWB_CONF_T_SCHED
#define ELWB_CONF_T_SCHED         (ELWB_TIMER_SECOND / 50)      /* 20ms */
#endif /* ELWB_CONF_T_SCHED */

/* duration of a data slot in ticks, if not defined, the minimum required
 * slot time is calculated based on the network N_HOPS and N_TX */
#ifndef ELWB_CONF_T_DATA
#define ELWB_CONF_T_DATA          (ELWB_TIMER_SECOND / 50)      /* 20ms */
#endif /* ELWB_CONF_T_DATA */

/* duration of a contention slot in ticks */
#ifndef ELWB_CONF_T_CONT
#define ELWB_CONF_T_CONT          (ELWB_TIMER_SECOND / 100)     /* 10ms */
#endif /* ELWB_CONF_T_CONT */

/* duration of a data ACK slot in ticks */
#ifndef ELWB_CONF_T_DACK
#define ELWB_CONF_T_DACK          (ELWB_CONF_T_CONT * 2)
#endif /* ELWB_CONF_T_DACK */

/* gap time between 2 slots in ticks */
#ifndef ELWB_CONF_T_GAP
#define ELWB_CONF_T_GAP           (ELWB_TIMER_SECOND / 200)     /* 5ms */
#endif /* ELWB_CONF_T_GAP */

/* guard time before RX slots in ticks */
#ifndef ELWB_CONF_T_GUARD_SLOT
#define ELWB_CONF_T_GUARD_SLOT    (ELWB_TIMER_SECOND / 4000)    /* 0.25ms */
#endif /* ELWB_CONF_T_GUARD_SLOT */

/* guard time before a round in LF ticks */
#ifndef ELWB_CONF_T_GUARD_ROUND
#define ELWB_CONF_T_GUARD_ROUND   (ELWB_TIMER_SECOND / 1000)    /* 1ms */
#endif /* ELWB_CONF_T_GUARD_ROUND */

/* time reserved for the preprocess task (before a round) in LF ticks */
#ifndef ELWB_CONF_T_PREPROCESS
#define ELWB_CONF_T_PREPROCESS    (ELWB_TIMER_SECOND / 10)      /* 100ms */
#endif /* ELWB_CONF_T_PREPROCESS */

#ifndef ELWB_CONF_T_DEEPSLEEP
#define ELWB_CONF_T_DEEPSLEEP     (ELWB_TIMER_SECOND * 3600)    /* 1h */
#endif /* ELWB_CONF_T_DEEPSLEEP */

#ifndef ELWB_CONF_SCHED_PERIOD_IDLE
#define ELWB_CONF_SCHED_PERIOD_IDLE 15
#endif /* ELWB_CONF_SCHED_PERIOD_IDLE */

#ifndef ELWB_CONF_SCHED_PERIOD_MIN
#define ELWB_CONF_SCHED_PERIOD_MIN  3
#endif /* ELWB_CONF_SCHED_PERIOD_MIN */

#ifndef ELWB_CONF_SCHED_PERIOD_MAX
#define ELWB_CONF_SCHED_PERIOD_MAX  60
#endif /* ELWB_CONF_SCHED_PERIOD_MIN */

/* slack time for schedule computation, in ticks */
#ifndef ELWB_CONF_SCHED_COMP_TIME
#define ELWB_CONF_SCHED_COMP_TIME (ELWB_TIMER_SECOND / 50)          /* 20ms */
#endif /* ELWB_CONF_SCHED_COMP_TIME */

/* use a 'fair' scheduler which tries to assign slots to all nodes */
#ifndef ELWB_CONF_SCHED_FAIR
#define ELWB_CONF_SCHED_FAIR      1
#endif /* ELWB_CONF_SCHED_FAIR */

/* compress the schedule? */
#ifndef ELWB_CONF_SCHED_COMPRESS
#define ELWB_CONF_SCHED_COMPRESS  1
#endif /* ELWB_CONF_SCHED_COMPRESS */

/* append CRC to the schedule? */
#ifndef ELWB_CONF_SCHED_CRC
#define ELWB_CONF_SCHED_CRC       1
#endif /* ELWB_CONF_SCHED_ADD_CRC */

#ifndef ELWB_CONF_SCHED_NODE_TIMEOUT
#define ELWB_CONF_SCHED_NODE_TIMEOUT  3600    /* 1h */
#endif /* ELWB_CONF_SCHED_NODE_TIMEOUT */

#ifndef ELWB_CONF_MAX_CLOCK_DRIFT
#define ELWB_CONF_MAX_CLOCK_DRIFT 100    /* in ppm */
#endif /* ELWB_CONF_MAX_CLOCK_DRIFT */

#ifndef ELWB_CONF_CONT_TH
#define ELWB_CONF_CONT_TH         1
#endif /* ELWB_CONF_CONT_TH */

/* use more accurate high frequency reference clock to schedule the contention slot
 * (requires an implementation of ELWB_GLOSSY_GET_T_REF_HF()) */
#ifndef ELWB_CONF_CONT_USE_HFTIMER
#define ELWB_CONF_CONT_USE_HFTIMER  0
#endif /* ELWB_CONF_CONT_USE_HFTIMER */


/* --------------- END OF CONFIG, do not change values below --------------- */


#define ELWB_PERIOD_SCALE         100     // 1/scale = granularity (10ms -> allows for periods between 10 and 655350ms)
#define ELWB_REQ_PKT_LEN          2
#define ELWB_2ND_SCHED_LEN        2
#define ELWB_SCHED_CRC_LEN        (ELWB_CONF_SCHED_CRC ? 2 : 0)
#define ELWB_SCHED_PERIOD_MAX     (65535 / ELWB_PERIOD_SCALE)

#ifndef RF_CONF_MAX_PKT_LEN
#define RF_CONF_MAX_PKT_LEN       (ELWB_CONF_MAX_PKT_LEN + \
                                   GLOSSY_MAX_HEADER_LEN)
#endif /* RF_CONF_MAX_PKT_LEN */

#define ELWB_TICKS_TO_MS(t)       ((uint32_t)(t) * 1000UL / ELWB_TIMER_SECOND)
#define ELWB_MS_TO_TICKS(t)       ((uint32_t)(t) * ELWB_TIMER_SECOND / 1000UL)


/*---------------------------------------------------------------------------*/

/* parameter sanity checks */

#if RF_CONF_MAX_PKT_LEN < (LWB_CONF_MAX_PKT_LEN + GLOSSY_MAX_HEADER_LEN)
#error "LWB_CONF_MAX_PKT_LEN is too big"
#endif

#if ELWB_PERIOD_SCALE == 0 || ELWB_PERIOD_SCALE > 1000
#error "invalid ELWB_PERIOD_SCALE"
#endif

#if ELWB_CONF_SCHED_PERIOD_IDLE > ELWB_SCHED_PERIOD_MAX
#error "ELWB_CONF_SCHED_PERIOD_IDLE invalid"
#endif

#if ELWB_CONF_SCHED_PERIOD_IDLE < ELWB_CONF_SCHED_PERIOD_MIN
#error "ELWB_CONF_SCHED_PERIOD_IDLE < ELWB_CONF_SCHED_PERIOD_MIN"
#endif

#if ELWB_CONF_WRITE_TO_BOLT && !BOLT_CONF_ON
#error "ELWB_CONF_WRITE_TO_BOLT requires BOLT to be enabled!"
#endif

#if ELWB_CONF_MAX_N_NODES > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_N_NODES is invalid"
#endif

#if (ELWB_CONF_MAX_DATA_SLOTS * 2 + ELWB_SCHED_HDR_LEN) > ELWB_CONF_MAX_PKT_LEN
#error "ELWB_CONF_MAX_DATA_SLOTS exceeds the packet size limit"
#endif

#if ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_SLOTS_HOST > ELWB_CONF_MAX_DATA_SLOTS!"
#endif

#if ELWB_CONF_SCHED_FAIR
  /* make sure #slots is <= 100 to prevent an overflow in the calculations */
  #if ELWB_CONF_MAX_DATA_SLOTS > 100
  #error "ELWB_CONF_MAX_DATA_SLOTS > 100 not allowed"
  #endif
#endif

#if ELWB_CONF_CONT_TH == 0
#error "invalid value for ELWB_CONF_CONT_TH"
#endif /* ELWB_CONF_CONT_TH == 0 */

/*---------------------------------------------------------------------------*/

/* macros */

/* schedule related macros */
#define ELWB_SCHED_N_SLOTS(s)           ((s)->n_slots & 0x1fff)
#define ELWB_SCHED_CLR_SLOTS(s)         ((s)->n_slots &= ~0x1fff)
#define ELWB_SCHED_HAS_SLOTS(s)         (((s)->n_slots & 0x1fff) > 0)
#define ELWB_SCHED_HAS_DATA_SLOTS(s)    (((s)->n_slots & 0x8000) > 0)
#define ELWB_SCHED_HAS_CONT_SLOT(s)     (((s)->n_slots & 0x4000) > 0)
#define ELWB_SCHED_IS_FIRST(s)          ELWB_SCHED_HAS_CONT_SLOT(s)
#define ELWB_SCHED_IS_STATE_IDLE(s)     (((s)->n_slots & 0x2000) > 0)
#define ELWB_SCHED_SET_CONT_SLOT(s)     ((s)->n_slots |= 0x4000)
#define ELWB_SCHED_SET_DATA_SLOTS(s)    ((s)->n_slots |= 0x8000)
#define ELWB_SCHED_SET_STATE_IDLE(s)    ((s)->n_slots |= 0x2000)

/* timer */
#define ELWB_TIMER_SECOND               LPTIMER_SECOND
#define ELWB_TIMER_NOW()                lptimer_now()
#define ELWB_TIMER_SET(t, cb)           lptimer_set(t, cb)
#define ELWB_HFTIMER_SCHEDULE           //TODO

/* glossy-style communication primitive */
#define ELWB_GLOSSY_START(initiator, data, data_len, n_tx, is_schedule) \
                                        gloria_start(initiator, data, data_len, n_tx, is_schedule)
#define ELWB_GLOSSY_STOP()              gloria_stop()
#define ELWB_GLOSSY_GET_PAYLOAD_LEN()   gloria_get_payload_len()
#define ELWB_GLOSSY_SIGNAL_DETECTED()   gloria_get_rx_started_cnt()
#define ELWB_GLOSSY_RX_CNT()            gloria_get_rx_cnt()
#define ELWB_GLOSSY_GET_T_REF()         gloria_get_t_ref()
#define ELWB_GLOSSY_GET_T_REF_HF()      0  //TODO
#define ELWB_GLOSSY_IS_T_REF_UPDATED()  gloria_is_t_ref_updated()

/* message passing */
#define ELWB_QUEUE_SIZE(handle)         uxQueueMessagesWaiting(handle)          /* polls the queue size (# elements in queue) */
#define ELWB_QUEUE_SPACE(handle)        uxQueueSpacesAvailable(handle)          /* polls the empty queue space */
#define ELWB_QUEUE_POP(handle, data)    xQueueReceive(handle, data, 0)          /* don't block */
#define ELWB_QUEUE_PUSH(handle, data)   xQueueSend(handle, data, 0)

/* misc */
#ifndef ELWB_IS_HOST
#define ELWB_IS_HOST()                  (HOST_ID == NODE_ID)
#endif /* ELWB_IS_HOST */
#ifndef ELWB_IS_SINK
#define ELWB_IS_SINK()                  ELWB_IS_HOST()
#endif /* ELWB_IS_SINK */
/* a custom packet filter (if expression evaluates to 'true', the packet will be forwarded to the application layer) */
#ifndef ELWB_RCV_PKT_FILTER
#define ELWB_RCV_PKT_FILTER()    0
#endif /* ELWB_RCV_PKT_FILTER */
#define ELWB_PAYLOAD_LEN(msg)           (DPP_MSG_LEN((dpp_message_t*)msg))

/*---------------------------------------------------------------------------*/

/* structs and typedefs */

typedef struct {
  uint32_t bootstrap_cnt; /* #times bootstrap state was entered */
  uint32_t unsynced_cnt;  /* #times a schedule was missed */
  uint32_t sleep_cnt;     /* #times node went into LPM due to rf silence */
  int32_t  drift;         /* current estimated drift in ppm */
  uint32_t ref_ofs;       /* reference offset in timer ticks */
  uint32_t pkt_cnt;       /* total number of 'seen' data packets */
  uint32_t pkt_sent;      /* total number of sent data packets */
  uint32_t pkt_ack;       /* total number acknowledged data packets */
  uint32_t pkt_rcvd;      /* total number of received packets (forwarded to the application) */
  uint32_t pkt_dropped;   /* packets dropped due to input buffer full */
} elwb_stats_t;

#define ELWB_SCHED_HDR_LEN   12
#define ELWB_SCHED_MAX_SLOTS ((ELWB_CONF_MAX_PKT_LEN - ELWB_SCHED_HDR_LEN - \
                               ELWB_SCHED_CRC_LEN) / 2)
/* note: ELWB_SCHED_MAX_SLOTS != ELWB_CONF_MAX_DATA_SLOTS */
typedef struct {
  uint64_t time;        /* current time in microseconds */
  uint16_t period;
  /* store num. of data slots and last two bits to indicate whether there is
   * a contention or an s-ack slot in this round */
  uint16_t n_slots;
  uint16_t slot[ELWB_SCHED_MAX_SLOTS + ELWB_SCHED_CRC_LEN];
} elwb_schedule_t;

typedef uint64_t elwb_time_t;


/*---------------------------------------------------------------------------*/

/* global variables */


/*---------------------------------------------------------------------------*/

/* function prototypes */

void     elwb_notify(void);

void     elwb_start(void* elwb_task,
                    void* pre_elwb_task,
                    void* post_elwb_task,
                    void* in_queue_handle,
                    void* out_queue_handle);    /* queue data type must be dpp_message_t */

void     elwb_get_last_syncpoint(elwb_time_t* time, elwb_time_t* rx_timestamp);

/* if argument is given, converts the timestamp (in ELWB timer ticks) to global network time
 * returns the current network time if no argument is given */
elwb_time_t elwb_get_time(const uint64_t* timestamp);

const elwb_stats_t * const elwb_get_stats(void);

void     elwb_set_drift(int32_t drift_ppm);

/* scheduler functions */
uint32_t elwb_sched_get_period(void);
void     elwb_sched_set_period(uint32_t p);
elwb_time_t elwb_sched_get_time(void);
void     elwb_sched_set_time(elwb_time_t new_time);

/*---------------------------------------------------------------------------*/

#endif /* PROTOCOL_ELWB_ELWB_H_ */
