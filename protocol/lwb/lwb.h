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

/*
 * a simplified version of the Low-Power Wireless Bus
 * (simplified state machine, no streams, 2nd schedule only contains period)
 */

#ifndef PROTOCOL_LWB_LWB_H_
#define PROTOCOL_LWB_LWB_H_


#ifndef LWB_ENABLE
#define LWB_ENABLE               0
#endif /* LWB_ENABLE */


/* --------------- START OF CONFIG (default values) ------------------------ */

#ifndef LWB_NETWORK_ID
#define LWB_NETWORK_ID            0x2222      /* custom 15-bit network ID */
#endif /* LWB_NETWORK_ID */

#ifndef LWB_NODE_ID
#define LWB_NODE_ID               NODE_ID
#endif /* LWB_NODE_ID */

/* the smallest node ID in the network (the host will ignore requests from nodes with smaller IDs) */
#ifndef LWB_MIN_NODE_ID
#define LWB_MIN_NODE_ID           1
#endif /* LWB_MIN_NODE_ID */

/* the largest node ID in the network (the host will ignore requests from nodes with larger IDs) */
#ifndef LWB_MAX_NODE_ID
#define LWB_MAX_NODE_ID           32
#endif /* LWB_MAX_NODE_ID */

/* max. payload size in bytes (data packets) */
#ifndef LWB_MAX_PAYLOAD_LEN
#define LWB_MAX_PAYLOAD_LEN       (GLORIA_INTERFACE_MAX_PAYLOAD_LEN - LWB_PKT_HDR_LEN)
#endif /* LWB_MAX_PAYLOAD_LEN */

#ifndef LWB_N_TX
#define LWB_N_TX                  3     /* how many times a packet is retransmitted */
#endif /* LWB_N_TX */

#ifndef LWB_NUM_HOPS
#define LWB_NUM_HOPS              3     /* network diameter in number of hops */
#endif /* LWB_NUM_HOPS */

#ifndef LWB_BOOTSTRAP_TIMEOUT
#define LWB_BOOTSTRAP_TIMEOUT     (120 * LWB_TIMER_FREQUENCY)     /* in ticks */
#endif /* LWB_BOOTSTRAP_TIMEOUT */

/* max. number of data or request slots per round */
#ifndef LWB_MAX_DATA_SLOTS
#define LWB_MAX_DATA_SLOTS        10
#endif /* LWB_MAX_DATA_SLOTS */

/* how many slots the host may allocate for itself, per round */
#ifndef LWB_MAX_SLOTS_HOST
#define LWB_MAX_SLOTS_HOST        (LWB_MAX_DATA_SLOTS / 2)
#endif /* LWB_MAX_SLOTS_HOST */

/* gap time between 2 slots in ticks */
#ifndef LWB_T_GAP
#define LWB_T_GAP                 (LWB_TIMER_FREQUENCY / 200)     /* 5ms */
#endif /* LWB_T_GAP */

/* guard time before RX slots in ticks */
#ifndef LWB_T_GUARD_SLOT
#define LWB_T_GUARD_SLOT          (LWB_TIMER_FREQUENCY / 4000)    /* 0.25ms */
#endif /* LWB_T_GUARD_SLOT */

/* guard time before a round in LF ticks */
#ifndef LWB_T_GUARD_ROUND
#define LWB_T_GUARD_ROUND         (LWB_TIMER_FREQUENCY / 1000)    /* 1ms */
#endif /* LWB_T_GUARD_ROUND */

/* longer guard time before a round in LF ticks, used if a schedule is missed or the drift has not yet been estimated */
#ifndef LWB_T_GUARD_ROUND_2
#define LWB_T_GUARD_ROUND_2       (LWB_TIMER_FREQUENCY / 5000)    /* 5ms */
#endif /* LWB_T_GUARD_ROUND_2 */

/* time reserved for the preprocess task (before a round) in LF ticks */
#ifndef LWB_T_PREPROCESS
#define LWB_T_PREPROCESS          (LWB_TIMER_FREQUENCY / 10)      /* 100ms */
#endif /* LWB_T_PREPROCESS */

#ifndef LWB_T_DEEPSLEEP
#define LWB_T_DEEPSLEEP           (LWB_TIMER_FREQUENCY * 3600)    /* 1h */
#endif /* LWB_T_DEEPSLEEP */

/* if set to 1, the host will append data ACKs to the 2nd schedule */
#ifndef LWB_DATA_ACK
#define LWB_DATA_ACK              0
#endif /* LWB_DATA_ACK */

#ifndef LWB_SCHED_PERIOD
#define LWB_SCHED_PERIOD          15      /* in seconds */
#endif /* LWB_SCHED_PERIOD */

/* slack time for schedule computation, in ticks */
#ifndef LWB_SCHED_COMP_TIME
#define LWB_SCHED_COMP_TIME       (LWB_TIMER_FREQUENCY / 50)          /* 20ms */
#endif /* LWB_SCHED_COMP_TIME */

/* append CRC to the schedule? */
#ifndef LWB_SCHED_ADD_CRC
#define LWB_SCHED_ADD_CRC         1
#endif /* LWB_SCHED_ADD_CRC */

#ifndef LWB_SCHED_NODE_TIMEOUT
#define LWB_SCHED_NODE_TIMEOUT    3600    /* 1h */
#endif /* LWB_SCHED_NODE_TIMEOUT */

#ifndef LWB_MAX_CLOCK_DRIFT
#define LWB_MAX_CLOCK_DRIFT       100    /* in ppm */
#endif /* LWB_MAX_CLOCK_DRIFT */

/* use more accurate high frequency reference clock to schedule the contention slot
 * (requires an implementation of LWB_GLORIA_GET_T_REF_HF()) */
#ifndef LWB_CONT_USE_HSTIMER
#define LWB_CONT_USE_HSTIMER      0
#endif /* LWB_CONT_USE_HSTIMER */

/* max. number of rounds for random backoff */
#ifndef LWB_RAND_BACKOFF
#define LWB_RAND_BACKOFF          4
#endif /* LWB_RAND_BACKOFF */

/* TX delay feature: allows the host to tell certain nodes to delay the retransmission of data packets by one Gloria slot */
#ifndef LWB_USE_TX_DELAY
#define LWB_USE_TX_DELAY          0
#endif /* LWB_USE_TX_DELAY */


/* --------------- END OF CONFIG, do not change values below --------------- */

#define LWB_MAX_NUM_NODES         (LWB_MAX_NODE_ID - LWB_MIN_NODE_ID + 1)
#define LWB_SCHED_CRC_LEN         (LWB_SCHED_ADD_CRC ? 2 : 0)
#define LWB_SCHED_PERIOD_MAX_S    (ULONG_MAX / LWB_TIMER_FREQUENCY)  /* max period in seconds */
#define LWB_NETWORK_ID_BITMASK    0x7fff
#define LWB_PKT_BUFFER_SIZE       GLORIA_INTERFACE_MAX_PAYLOAD_LEN    /* must be at least as large as the gloria interface buffer */
#define LWB_TX_DELAY_MASK_SIZE    (LWB_USE_TX_DELAY ? ((LWB_MAX_NUM_NODES + 7) / 8) : 0)
#define LWB_DATA_ACK_SIZE         (LWB_DATA_ACK ? ((LWB_MAX_DATA_SLOTS + 7) / 8) : 0)


/*---------------------------------------------------------------------------*/

/* parameter sanity checks */

#if LWB_WRITE_TO_BOLT && !BOLT_ON
#error "LWB_WRITE_TO_BOLT requires BOLT to be enabled!"
#endif

#if (LWB_MAX_DATA_SLOTS * 2 + LWB_SCHED_HDR_LEN + LWB_SCHED_CRC_LEN) > GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#error "LWB_MAX_DATA_SLOTS exceeds the packet size limit"
#endif

#if (LWB_MAX_PAYLOAD_LEN + LWB_PKT_HDR_LEN) > GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#error "LWB_MAX_PAYLOAD_LEN must not exceed GLORIA_INTERFACE_MAX_PAYLOAD_LEN"
#endif

#if LWB_MAX_SLOTS_HOST > LWB_MAX_DATA_SLOTS
#error "LWB_MAX_SLOTS_HOST > LWB_MAX_DATA_SLOTS!"
#endif

#if LWB_MAX_NODE_ID < LWB_MIN_NODE_ID || LWB_MAX_NODE_ID >= DPP_DEVICE_ID_BROADCAST
#error "invalid value for LWB_MAX_NODE_ID"
#endif

#if LWB_PKT_BUFFER_SIZE < (LWB_SCHED_HDR_LEN + LWB_MAX_DATA_SLOTS * 2 + LWB_SCHED_CRC_LEN + LWB_TX_DELAY_MASK_SIZE)
#error "invalid schedule packet size"
#endif /* LWB_PKT_BUFFER_SIZE */

/*---------------------------------------------------------------------------*/

/* macros */

/* schedule-related macros */
#define LWB_IS_SCHEDULE_PACKET(s)      ((s)->header.type != 0)
#define LWB_IS_PKT_HEADER_VALID(p)     ((p)->header.net_id == (LWB_NETWORK_ID & LWB_NETWORK_ID_BITMASK))  // checks whether the packet header is valid
#define LWB_SET_PKT_HEADER(p)          ((p)->header_raw = (LWB_NETWORK_ID & LWB_NETWORK_ID_BITMASK))      // set the header of a regular packet (all except schedule packets)

/* timer */
#define LWB_TIMER_FREQUENCY            LPTIMER_SECOND
#define LWB_TIMER_NOW()                lptimer_now()
#define LWB_TIMER_LAST_EXP()           lptimer_get()
#define LWB_TIMER_SET(t, cb)           lptimer_set(t, cb)                      /* low-power timer */
#define LWB_TIMER_STOP()               lptimer_set(0, 0)
#define LWB_TIMER_HS_SET(t, cb)        hs_timer_schedule_start(t, cb)          /* high-speed timer */
#define LWB_TICKS_TO_S(t)              ((t) / LWB_TIMER_FREQUENCY)
#define LWB_TICKS_TO_MS(t)             ((uint64_t)(t) * 1000UL / LWB_TIMER_FREQUENCY)
#define LWB_TICKS_TO_US(t)             ((uint64_t)(t) * 1000000ULL / LWB_TIMER_FREQUENCY)
#define LWB_MS_TO_TICKS(ms)            ((uint64_t)(ms) * LWB_TIMER_FREQUENCY / 1000UL)
#define LWB_S_TO_TICKS(s)              ((uint64_t)(s) * LWB_TIMER_FREQUENCY)

/* message passing */
#ifndef LWB_QUEUE_SIZE
#define LWB_QUEUE_SIZE(handle)         uxQueueMessagesWaiting(handle)          /* polls the queue size (# elements in queue) */
#endif /* LWB_QUEUE_SIZE */
#ifndef LWB_QUEUE_SPACE
#define LWB_QUEUE_SPACE(handle)        uxQueueSpacesAvailable(handle)          /* polls the empty queue space */
#endif /* LWB_QUEUE_SPACE */
#ifndef LWB_QUEUE_POP
#define LWB_QUEUE_POP(handle, data)    xQueueReceive(handle, data, 0)          /* receive / remove an element from the queue (non-blocking, return true on success) */
#endif /* LWB_QUEUE_POP */
#ifndef LWB_QUEUE_PUSH
#define LWB_QUEUE_PUSH(handle, data)   xQueueSend(handle, data, 0)             /* append an element to the queue */
#endif /* LWB_QUEUE_PUSH */
#ifndef LWB_QUEUE_CLEAR
#define LWB_QUEUE_CLEAR(handle)        xQueueReset(handle)                     /* empty a queue (drop all elements) */
#endif /* LWB_QUEUE_CLEAR */

/* task related stuff */
#ifndef LWB_TASK_YIELD
#define LWB_TASK_YIELD()               xTaskNotifyWait(0, ULONG_MAX, NULL, portMAX_DELAY)  /* function used to yield the LWB task (and allow other, lower priority tasks to run) */
#endif /* LWB_TASK_YIELD */
#ifndef LWB_TASK_NOTIFY
#define LWB_TASK_NOTIFY(task)          xTaskNotify(task, 0, eNoAction)         /* function used to notify (poll) a task, i.e. giving it permission to run */
#endif /* LWB_TASK_NOTIFY */
#ifndef LWB_TASK_NOTIFY_FROM_ISR
#define LWB_TASK_NOTIFY_FROM_ISR(task) xTaskNotifyFromISR(task, 0, eNoAction, 0);
#endif /* LWB_TASK_NOTIFY_FROM_ISR */
#ifndef LWB_RESUMED
  #define LWB_RESUMED()
  #define LWB_SUSPENDED()
#endif /* LWB_RESUMED */
#ifndef LWB_ON_WAKEUP
#define LWB_ON_WAKEUP()
#endif /* LWB_ON_WAKEUP */

/* whether the node is a sink (by default, a host node is a sink node) */
#ifndef LWB_IS_SINK
#define LWB_IS_SINK()                  is_host
#endif /* LWB_IS_SINK */

/* a custom packet filter (if expression evaluates to 'true', the packet will be forwarded to the application layer) */
#ifndef LWB_RCV_PKT_FILTER
#define LWB_RCV_PKT_FILTER()           0
#endif /* LWB_RCV_PKT_FILTER */

#define LWB_PAYLOAD_LEN(msg)           (DPP_MSG_LEN((dpp_message_t*)msg))

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
} lwb_stats_t;

typedef enum {
  LWB_PHASE_INIT = 0,
  LWB_PHASE_SCHED1,
  LWB_PHASE_DATA,
  LWB_PHASE_CONT,
  LWB_PHASE_SCHED2,
  NUM_LWB_PHASES,
} lwb_phases_t;

#pragma pack(1)           /* force alignment to 1 byte for the following structs */

#define LWB_PKT_HDR_LEN     2      /* packet header length */
typedef struct {
  uint16_t net_id : 15;   /* network ID and packet type indicator */
  uint16_t type   : 1;    /* MSB indicates the packet type (1 = schedule packet) */
} lwb_header_t;

#define LWB_SCHED_HDR_LEN   18
typedef struct {
  lwb_header_t  header;
  uint16_t      n_slots        : 15;  /* number of data slots in the schedule */
  uint16_t      has_delay_mask : 1;   /* whether a delay mask is included in the schedule */
  uint32_t      period;               /* round period in LWB ticks */
  uint64_t      time;                 /* current time in microseconds */
  uint16_t      host_id;              /* node ID of the host */
  union {
    uint16_t    slot[LWB_MAX_DATA_SLOTS + LWB_SCHED_ADD_CRC];
    uint8_t     rxbuffer[LWB_PKT_BUFFER_SIZE - LWB_SCHED_HDR_LEN];  /* to make sure that potentially larger received packets don't cause a buffer overflow */
  };
} lwb_schedule_t;

typedef uint64_t lwb_time_t;

#define LWB_CONT_PKT_LEN    4                         /* contention packet length without header */
#define LWB_2ND_SCHED_LEN   (6 + LWB_DATA_ACK_SIZE)   /* schedule packet length without header */

typedef struct {
  union {
    lwb_header_t header;
    uint16_t     header_raw;
  };
  union {
    struct {
      uint16_t   node_id;
      uint16_t   ipi;           /* inter-packet interval in seconds */
    } cont;
    struct {
      uint32_t   period;
      uint16_t   cont_winner;
#if LWB_DATA_ACK
      uint8_t    dack[LWB_DATA_ACK_SIZE];
#endif /* LWB_DATA_ACK */
    } sched2;
    uint8_t      payload[LWB_MAX_PAYLOAD_LEN];
    uint16_t     payload16[LWB_MAX_PAYLOAD_LEN / 2];
    uint8_t      rxbuffer[LWB_PKT_BUFFER_SIZE - LWB_PKT_HDR_LEN];    /* to make sure that potentially larger received packets don't cause a buffer overflow */
  };
} lwb_packet_t;

#pragma pack()


typedef void (* lwb_timeout_cb_t)(void);
typedef void (* lwb_slot_cb_t)(uint16_t, lwb_phases_t, lwb_packet_t*);      /* initiator ID, LWB phase, pointer to the packet buffer */


/*---------------------------------------------------------------------------*/

/* function prototypes */

void     lwb_notify(void);

bool     lwb_init(void* lwb_task,
                  void* pre_lwb_task,
                  void* post_lwb_task,
                  void* in_queue_handle,
                  void* out_queue_handle,         /* queue data type must be dpp_message_t */
                  void* retransmit_queue_handle,
                  lwb_timeout_cb_t listen_timeout_cb,
                  bool host);

void     lwb_start(void);
void     lwb_stop(void);

void     lwb_get_last_syncpoint(lwb_time_t* time, lwb_time_t* rx_timestamp);

/* register a custom callback function that will be executed after each communication slot, can e.g. be used to collect stats */
void     lwb_register_slot_callback(lwb_slot_cb_t cb);

/* if argument is given, converts the timestamp (in LWB timer ticks) to global network time
 * returns the current network time if no argument is given */
lwb_time_t lwb_get_time(const uint64_t* timestamp);
uint32_t lwb_get_time_sec(void);

const lwb_stats_t * const lwb_get_stats(void);

uint32_t lwb_get_max_round_duration(uint32_t t_sched_arg, uint32_t t_cont_arg, uint32_t t_data_arg);

void     lwb_set_drift(int32_t drift_ppm);

void     lwb_set_ipi(uint16_t ipi_secs);                /* set the inter-paket interval in seconds (source nodes only) */

uint8_t  lwb_get_n_tx(void);                            /* gets the number of retransmissions */
bool     lwb_set_n_tx(uint8_t n_tx_arg);                /* sets the number of retransmissions, returns true on success */
uint8_t  lwb_get_num_hops(void);                        /* gets the network size in number of hops */
bool     lwb_set_num_hops(uint8_t num_hops_arg);        /* sets the network size in number of hops (used to calculate slot sizes), returns true on success */

/* scheduler functions */
uint32_t lwb_sched_get_period(void);                    /* returns the current period in seconds */
bool     lwb_sched_set_period(uint32_t period_secs);    /* set the period in seconds */
bool     lwb_sched_check_params(uint32_t period_secs, uint32_t sched_slot, uint32_t cont_slot, uint32_t data_slot);  /* checks whether the given parameters are valid (slot lengths are in ticks) */
lwb_time_t lwb_sched_get_time(void);                    /* returns the current network time in microseconds */
void     lwb_sched_set_time(lwb_time_t time_us);        /* set the current network time in microseconds */
void     lwb_sched_set_delay_nodes(const uint16_t* node_list, uint8_t num_nodes);   /* define which source nodes should delay the retransmission of data packets */

/*---------------------------------------------------------------------------*/

#endif /* PROTOCOL_LWB_LWB_H_ */
