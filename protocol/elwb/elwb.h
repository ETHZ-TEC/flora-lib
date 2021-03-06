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

#ifndef PROTOCOL_ELWB_ELWB_H_
#define PROTOCOL_ELWB_ELWB_H_


#ifndef ELWB_ENABLE
#define ELWB_ENABLE               0
#endif /* ELWB_ENABLE */


/* --------------- START OF CONFIG (default values) ------------------------ */

#ifndef ELWB_CONF_NETWORK_ID
#define ELWB_CONF_NETWORK_ID      0x1111      /* custom 15-bit network ID */
#endif /* ELWB_CONF_NETWORK_ID */

/* max. payload size in bytes (data packets) */
#ifndef ELWB_CONF_MAX_PAYLOAD_LEN
#define ELWB_CONF_MAX_PAYLOAD_LEN (GLORIA_INTERFACE_MAX_PAYLOAD_LEN - ELWB_PKT_HDR_LEN)
#endif /* ELWB_CONF_MAX_PAYLOAD_LEN */

#ifndef ELWB_CONF_N_TX
#define ELWB_CONF_N_TX            3     /* how many times a packet is retransmitted */
#endif /* ELWB_CONF_N_TX */

#ifndef ELWB_CONF_NUM_HOPS
#define ELWB_CONF_NUM_HOPS        3     /* network diameter in number of hops */
#endif /* ELWB_CONF_NUM_HOPS */

#ifndef ELWB_CONF_BOOTSTRAP_TIMEOUT
#define ELWB_CONF_BOOTSTRAP_TIMEOUT   (120 * ELWB_TIMER_FREQUENCY)     /* in ticks */
#endif /* ELWB_CONF_BOOTSTRAP_TIMEOUT */

/* max. number of nodes in the network */
#ifndef ELWB_CONF_MAX_NODES
#define ELWB_CONF_MAX_NODES       10
#endif /* ELWB_CONF_MAX_NODES */

/* max. number of data or request slots per round */
#ifndef ELWB_CONF_MAX_DATA_SLOTS
#define ELWB_CONF_MAX_DATA_SLOTS  10
#endif /* ELWB_CONF_MAX_DATA_SLOTS */

/* how many slots the host may allocate for himself, per round */
#ifndef ELWB_CONF_MAX_SLOTS_HOST
#define ELWB_CONF_MAX_SLOTS_HOST  (ELWB_CONF_MAX_DATA_SLOTS / 2)
#endif /* ELWB_CONF_MAX_SLOTS_HOST */

/* gap time between 2 slots in ticks */
#ifndef ELWB_CONF_T_GAP
#define ELWB_CONF_T_GAP           (ELWB_TIMER_FREQUENCY / 200)     /* 5ms */
#endif /* ELWB_CONF_T_GAP */

/* guard time before RX slots in ticks */
#ifndef ELWB_CONF_T_GUARD_SLOT
#define ELWB_CONF_T_GUARD_SLOT    (ELWB_TIMER_FREQUENCY / 4000)    /* 0.25ms */
#endif /* ELWB_CONF_T_GUARD_SLOT */

/* guard time before a round in LF ticks */
#ifndef ELWB_CONF_T_GUARD_ROUND
#define ELWB_CONF_T_GUARD_ROUND   (ELWB_TIMER_FREQUENCY / 1000)    /* 1ms */
#endif /* ELWB_CONF_T_GUARD_ROUND */

/* time reserved for the preprocess task (before a round) in LF ticks */
#ifndef ELWB_CONF_T_PREPROCESS
#define ELWB_CONF_T_PREPROCESS    (ELWB_TIMER_FREQUENCY / 10)      /* 100ms */
#endif /* ELWB_CONF_T_PREPROCESS */

#ifndef ELWB_CONF_T_DEEPSLEEP
#define ELWB_CONF_T_DEEPSLEEP     (ELWB_TIMER_FREQUENCY * 3600)    /* 1h */
#endif /* ELWB_CONF_T_DEEPSLEEP */

#ifndef ELWB_CONF_DATA_ACK
#define ELWB_CONF_DATA_ACK        0
#endif /* ELWB_CONF_DATA_ACK */

#ifndef ELWB_CONF_SCHED_PERIOD
#define ELWB_CONF_SCHED_PERIOD    15      /* in seconds */
#endif /* ELWB_CONF_SCHED_PERIOD */

/* slack time for schedule computation, in ticks */
#ifndef ELWB_CONF_SCHED_COMP_TIME
#define ELWB_CONF_SCHED_COMP_TIME (ELWB_TIMER_FREQUENCY / 50)          /* 20ms */
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
#endif /* ELWB_CONF_SCHED_CRC */

#ifndef ELWB_CONF_SCHED_NODE_TIMEOUT
#define ELWB_CONF_SCHED_NODE_TIMEOUT  3600    /* 1h */
#endif /* ELWB_CONF_SCHED_NODE_TIMEOUT */

#ifndef ELWB_CONF_MAX_CLOCK_DRIFT
#define ELWB_CONF_MAX_CLOCK_DRIFT 100    /* in ppm */
#endif /* ELWB_CONF_MAX_CLOCK_DRIFT */

/* contention threshold, how many elements need to be in the queue before participating in a contention round */
#ifndef ELWB_CONF_CONT_TH
#define ELWB_CONF_CONT_TH         1
#endif /* ELWB_CONF_CONT_TH */

/* use more accurate high frequency reference clock to schedule the contention slot
 * (requires an implementation of ELWB_GLORIA_GET_T_REF_HF()) */
#ifndef ELWB_CONF_CONT_USE_HSTIMER
#define ELWB_CONF_CONT_USE_HSTIMER  0
#endif /* ELWB_CONF_CONT_USE_HSTIMER */

/* max. number of rounds for random backoff */
#ifndef ELWB_CONF_RAND_BACKOFF
#define ELWB_CONF_RAND_BACKOFF    4
#endif /* ELWB_CONF_RAND_BACKOFF */


/* --------------- END OF CONFIG, do not change values below --------------- */

#define ELWB_SCHED_CRC_LEN        (ELWB_CONF_SCHED_CRC ? 2 : 0)
#define ELWB_SCHED_PERIOD_MAX_S   (ULONG_MAX / ELWB_TIMER_FREQUENCY)  /* max period in seconds */
#define ELWB_NETWORK_ID_BITMASK   0x7fff
#define ELWB_PKT_TYPE_BITMASK     0x8000
#define ELWB_PKT_BUFFER_SIZE      GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#define ELWB_MAX_SCHED_PKT_LEN    (ELWB_SCHED_HDR_LEN + ELWB_CONF_MAX_DATA_SLOTS * sizeof(uint16_t) + ELWB_SCHED_CRC_LEN)
#define ELWB_MAX_DATA_PKT_LEN     (ELWB_PKT_HDR_LEN + ELWB_CONF_MAX_PAYLOAD_LEN)
#define ELWB_MAX_DACK_PKT_LEN     (ELWB_PKT_HDR_LEN + (ELWB_CONF_MAX_DATA_SLOTS + 7) / 8)


/*---------------------------------------------------------------------------*/

/* parameter sanity checks */

#if ELWB_CONF_WRITE_TO_BOLT && !BOLT_CONF_ON
#error "ELWB_CONF_WRITE_TO_BOLT requires BOLT to be enabled!"
#endif

#if ELWB_CONF_MAX_N_NODES > ELWB_CONF_MAX_DATA_SLOTS
#error "ELWB_CONF_MAX_N_NODES is invalid"
#endif

#if (ELWB_CONF_MAX_DATA_SLOTS * 2 + ELWB_SCHED_HDR_LEN + ELWB_SCHED_CRC_LEN) > GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#error "ELWB_CONF_MAX_DATA_SLOTS exceeds the packet size limit"
#endif

#if (ELWB_CONF_MAX_PAYLOAD_LEN + ELWB_PKT_HDR_LEN) > GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#error "ELWB_CONF_MAX_PAYLOAD_LEN must not exceed GLORIA_INTERFACE_MAX_PAYLOAD_LEN"
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

#if ELWB_CONF_MAX_DATA_SLOTS < ELWB_CONF_MAX_NODES
#error "ELWB_CONF_MAX_DATA_SLOTS must be larger than ELWB_CONF_MAX_NODES"
#endif

#if ELWB_PKT_BUFFER_SIZE < (ELWB_SCHED_HDR_LEN + ELWB_CONF_MAX_DATA_SLOTS * 2 + ELWB_SCHED_CRC_LEN)
#error "invalid schedule packet size"
#endif /* ELWB_PKT_BUFFER_SIZE */

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

#define ELWB_IS_SCHEDULE_PACKET(s)      ((s)->header.type != 0)
#define ELWB_IS_PKT_HEADER_VALID(p)     ((p)->header.net_id == (ELWB_CONF_NETWORK_ID & ELWB_NETWORK_ID_BITMASK))  // checks whether the packet header is valid
#define ELWB_SET_PKT_HEADER(p)          ((p)->header_raw = (ELWB_CONF_NETWORK_ID & ELWB_NETWORK_ID_BITMASK))      // set the header of a regular packet (all except schedule packets)

/* timer */
#define ELWB_TIMER_FREQUENCY            LPTIMER_SECOND
#define ELWB_TIMER_NOW()                lptimer_now()
#define ELWB_TIMER_LAST_EXP()           lptimer_get()
#define ELWB_TIMER_SET(t, cb)           lptimer_set(t, cb)                      /* low-power timer */
#define ELWB_TIMER_STOP()               lptimer_set(0, 0)
#define ELWB_TIMER_HS_SET(t, cb)        hs_timer_schedule_start(t, cb)          /* high-speed timer */
#define ELWB_TICKS_TO_S(t)              ((t) / ELWB_TIMER_FREQUENCY)
#define ELWB_TICKS_TO_MS(t)             ((t) * 1000ULL / ELWB_TIMER_FREQUENCY)
#define ELWB_TICKS_TO_US(t)             ((uint64_t)(t) * 1000000ULL / ELWB_TIMER_FREQUENCY)
#define ELWB_S_TO_TICKS(s)              ((uint64_t)(s) * ELWB_TIMER_FREQUENCY)
#define ELWB_MS_TO_TICKS(ms)            ((uint64_t)(ms) * ELWB_TIMER_FREQUENCY / 1000ULL)
#define ELWB_US_TO_TICKS(us)            ((uint64_t)(us) * ELWB_TIMER_FREQUENCY / 1000000ULL)

/* message passing */
#ifndef ELWB_QUEUE_SIZE
#define ELWB_QUEUE_SIZE(handle)         uxQueueMessagesWaiting(handle)          /* polls the queue size (# elements in queue) */
#endif /* ELWB_QUEUE_SIZE */
#ifndef ELWB_QUEUE_SPACE
#define ELWB_QUEUE_SPACE(handle)        uxQueueSpacesAvailable(handle)          /* polls the empty queue space */
#endif /* ELWB_QUEUE_SPACE */
#ifndef ELWB_QUEUE_POP
#define ELWB_QUEUE_POP(handle, data)    xQueueReceive(handle, data, 0)          /* receive / remove an element from the queue (non-blocking, return true on success) */
#endif /* ELWB_QUEUE_POP */
#ifndef ELWB_QUEUE_PUSH
#define ELWB_QUEUE_PUSH(handle, data)   xQueueSend(handle, data, 0)             /* append an element to the queue */
#endif /* ELWB_QUEUE_PUSH */
#ifndef ELWB_QUEUE_CLEAR
#define ELWB_QUEUE_CLEAR(handle)        xQueueReset(handle)                     /* empty a queue (drop all elements) */
#endif /* ELWB_QUEUE_CLEAR */

/* task related stuff */
#ifndef ELWB_TASK_YIELD
#define ELWB_TASK_YIELD()               xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY)  /* function used to yield the eLWB task (and allow other, lower priority tasks to run) */
#endif /* ELWB_TASK_YIELD */
#ifndef ELWB_TASK_NOTIFY
#define ELWB_TASK_NOTIFY(task)          xTaskNotify(task, 0, eNoAction)         /* function used to notify (poll) a task, i.e. giving it permission to run */
#endif /* ELWB_TASK_NOTIFY */
#ifndef ELWB_TASK_NOTIFY_FROM_ISR
#define ELWB_TASK_NOTIFY_FROM_ISR(task) xTaskNotifyFromISR(task, 0, eNoAction, 0);
#endif /* ELWB_TASK_NOTIFY_FROM_ISR */
#ifndef ELWB_RESUMED
  #define ELWB_RESUMED()
  #define ELWB_SUSPENDED()
#endif /* ELWB_RESUMED */
#ifndef ELWB_ON_WAKEUP
#define ELWB_ON_WAKEUP()
#endif /* ELWB_ON_WAKEUP */

/* whether the node is a sink (by default, a host node is a sink node) */
#ifndef ELWB_IS_SINK
#define ELWB_IS_SINK()                  is_host
#endif /* ELWB_IS_SINK */

/* a custom packet filter (if expression evaluates to 'true', the packet will be forwarded to the application layer) */
#ifndef ELWB_RCV_PKT_FILTER
#define ELWB_RCV_PKT_FILTER()           0
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
  uint32_t pkt_sent;      /* total number of sent data packets */
  uint32_t pkt_ack;       /* total number acknowledged data packets */
  uint32_t pkt_rcvd;      /* total number of received packets (forwarded to the application) */
  uint32_t pkt_dropped;   /* packets dropped due to input buffer full */
  uint32_t pkt_rx_all;    /* total number of received packets (including schedules, requests, ACK, but without contention) */
  uint32_t pkt_tx_all;    /* total number of transmitted packets (including schedules, requests, ACK, but without contention) */
  int_fast8_t rssi_avg;   /* average RSSI value */
  int_fast8_t snr_avg;    /* average SNR value */
} elwb_stats_t;

typedef enum {
  ELWB_PHASE_INIT = 0,  // 0
  ELWB_PHASE_PRE,       // 1
  ELWB_PHASE_SCHED1,    // 2
  ELWB_PHASE_CONT,      // 3
  ELWB_PHASE_SCHED2,    // 4
  ELWB_PHASE_REQ,       // 5
  ELWB_PHASE_DATA,      // 6
  ELWB_PHASE_DACK,      // 7
  ELWB_PHASE_POST,      // 8
  ELWB_PHASE_IDLE,      // 9
  NUM_ELWB_PHASES,
} elwb_phases_t;

#pragma pack(1)           /* force alignment to 1 byte for the following structs */

#define ELWB_PKT_HDR_LEN  2   // packet header length
typedef struct {
  uint16_t net_id : 15;   /* network ID and packet type indicator */
  uint16_t type   : 1;    /* MSB indicates the packet type (1 = schedule packet) */
} elwb_header_t;

#define ELWB_SCHED_HDR_LEN   18
typedef struct {
  elwb_header_t header;
  uint16_t      n_slots;
  uint32_t      period;
  uint64_t      time;          /* current time in microseconds */
  uint16_t      host_id;
  /* store num. of data slots and last two bits to indicate whether there is
   * a contention or an s-ack slot in this round */
  union {
    uint16_t    slot[ELWB_CONF_MAX_DATA_SLOTS + ELWB_SCHED_CRC_LEN / 2];
    uint8_t     rxbuffer[ELWB_PKT_BUFFER_SIZE - ELWB_SCHED_HDR_LEN];  /* to make sure that potentially larger received packets don't cause a buffer overflow */
  };
} elwb_schedule_t;

typedef uint64_t elwb_time_t;

#define ELWB_REQ_PKT_LEN    2         /* request packet length without header */
#define ELWB_2ND_SCHED_LEN  4         /* schedule length without header */

typedef struct {
  union {
    elwb_header_t header;
    uint16_t      header_raw;
  };
  union {
    struct {
      uint16_t    node_id;
    } cont;     // contention packet
    struct {
      uint32_t    period;
    } sched2;   // 2nd schedule
    struct {
      uint16_t    num_slots;
    } req;      // request packet
    uint8_t     payload[ELWB_CONF_MAX_PAYLOAD_LEN];
    uint16_t    payload16[ELWB_CONF_MAX_PAYLOAD_LEN / 2];
    uint8_t     rxbuffer[ELWB_PKT_BUFFER_SIZE - ELWB_PKT_HDR_LEN];    /* to make sure that potentially larger received packets don't cause a buffer overflow */
  };
} elwb_packet_t;

#pragma pack()


typedef void (* elwb_timeout_cb_t)(void);
typedef void (* elwb_slot_cb_t)(uint16_t, elwb_phases_t, elwb_packet_t*);      // initiator ID, eLWB phase, pointer to the packet buffer


/*---------------------------------------------------------------------------*/

/* function prototypes */

void     elwb_notify(void);

bool     elwb_init(void* elwb_task,
                   void* pre_elwb_task,
                   void* post_elwb_task,
                   void* in_queue_handle,
                   void* out_queue_handle,         /* queue data type must be dpp_message_t */
                   void* retransmit_queue_handle,  /* buffer to queue packets for retransmission */
                   elwb_timeout_cb_t listen_timeout_cb,
                   bool host);

void     elwb_start(void);
void     elwb_stop(void);

void     elwb_get_last_syncpoint(elwb_time_t* time, elwb_time_t* rx_timestamp);

/* register a custom callback function that will be executed after each communication slot, can e.g. be used to collect stats */
void     elwb_register_slot_callback(elwb_slot_cb_t cb);

/* if argument is given, converts the timestamp (in ELWB timer ticks) to global network time
 * returns the current network time if no argument is given */
elwb_time_t elwb_get_time(const uint64_t* timestamp);
uint32_t    elwb_get_time_sec(void);

const elwb_stats_t * const elwb_get_stats(void);

uint32_t elwb_get_max_round_duration(uint32_t t_sched_arg, uint32_t t_cont_arg, uint32_t t_data_arg);

void     elwb_set_drift(int32_t drift_ppm);

uint8_t  elwb_get_n_tx(void);                             /* gets the number of retransmissions */
bool     elwb_set_n_tx(uint8_t n_tx_arg);                 /* sets the number of retransmissions, returns true on success */
uint8_t  elwb_get_num_hops(void);                         /* gets the network size in number of hops */
bool     elwb_set_num_hops(uint8_t num_hops_arg);         /* sets the network size in number of hops (used to calculate slot sizes), returns true on success */

/* scheduler functions */
uint32_t elwb_sched_get_period(void);                     /* returns the current period in seconds */
bool     elwb_sched_set_period(uint32_t period_secs);     /* set the period in seconds */
bool     elwb_sched_check_params(uint32_t period_secs, uint32_t sched_slot, uint32_t cont_slot, uint32_t data_slot);  /* checks whether the given parameters are valid (slot lengths are in ticks) */
elwb_time_t elwb_sched_get_time(void);                    /* returns the current network time in microseconds */
void     elwb_sched_set_time(elwb_time_t time_us);        /* set the current network time in microseconds */
bool     elwb_sched_add_node(uint16_t node_id);

/*---------------------------------------------------------------------------*/

#endif /* PROTOCOL_ELWB_ELWB_H_ */
