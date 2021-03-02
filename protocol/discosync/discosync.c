/*
 * discosync.c
 *
 * communication primitive for neighbor discovery & synchronization
 */

#include "main.h"


// Macros and defines

#define SCHEDULE_NEXT_SLOT()        hs_timer_schedule_start(start_time + DISCOSYNC_SLOT_TIME * (pkt.slot_idx + 1), &discosync_start_next_slot)


// Types

typedef struct __attribute__((__packed__, __aligned__(1)))  {
  uint8_t   protocol_id : 4;
  uint8_t   type : 4;
  uint8_t   slot_idx;
} discosync_packet_t;

_Static_assert(sizeof(discosync_packet_t) == DISCOSYNC_PKT_LEN, "discosync_packet_t is not DISCOSYNC_PKT_LEN bytes in size!");


// Private variables

static uint8_t            current_slot   = 0;
static uint8_t            total_slots    = 0;
static uint8_t            rx_done_cnt    = 0;
static uint64_t           start_time     = 0;
static bool               is_running     = false;
static discosync_cb_t     callback       = 0;
static uint8_t            tx_slots[DISCOSYNC_MAX_NUM_SLOTS]; // Slot assignment (0 == Rx, 1 == Tx)
static discosync_packet_t pkt;


// Private function prototypes

void discosync_start_rx(void);
void discosync_start_rx_tx(void);
void discosync_rx_timeout(bool crc_error);
void discosync_tx_done(void);
void discosync_rx_done(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error);
void discosync_start_next_slot(void);
void discosync_assign_probabilistic_slots(uint8_t nr_slots);
void discosync_assign_static_slots(uint8_t nr_slots);


// Functions

void discosync_start(uint8_t nr_slots, uint16_t rx_time_before_start_ms, discosync_cb_t _callback)
{
  if (nr_slots == 0) {
    return;
  }
  if (radio_wakeup()) {
    LOG_WARNING("radio was in sleep mode");
  }

  radio_reset_preamble_counter();
  radio_reset_sync_counter();

  // Set radio config
  radio_set_config_rxtx(radio_modulations[DISCOSYNC_MODULATION].modem,
                        DISCOSYNC_BAND,
                        radio_modulations[DISCOSYNC_MODULATION].datarate,
                        DISCOSYNC_TX_PWR,
                        radio_modulations[DISCOSYNC_MODULATION].bandwidth,
                        radio_modulations[DISCOSYNC_MODULATION].preambleLen,
                        radio_modulations[DISCOSYNC_MODULATION].coderate,
                        0,
                        false,
                        0,
                        true);
  radio_set_irq_mode(IRQ_MODE_RX_TX_CRC);
  radio_set_rx_gain(true);

  // Initialize state variables
  rx_done_cnt     = 0;
  current_slot    = 0;
  total_slots     = MIN(nr_slots, DISCOSYNC_MAX_NUM_SLOTS);
  start_time      = 0;
  is_running      = true;
  callback        = _callback;
  pkt.protocol_id = PROTOCOL_ID_DISCOSYNC;
  pkt.type        = DISCOSYNC_PKT_TYPE;
  pkt.slot_idx    = 0;
  memset(tx_slots, 0, DISCOSYNC_MAX_NUM_SLOTS);

  // Compute slot assignments
#if DISCOSYNC_PROBABILISTIC_ASSIGNMENT
  discosync_assign_probabilistic_slots(total_slots);
#else
  discosync_assign_static_slots(total_slots);
#endif /* DISCOSYNC_PROBABILISTIC_ASSIGNMENT */

  // Print slot assignment
  char array_representation[2 * DISCOSYNC_MAX_NUM_SLOTS + 1] = {0};
  for (uint8_t i = 0; i < total_slots; i++) {
    array_representation[2 * i    ] = '0' + tx_slots[i];
    array_representation[2 * i + 1] = ' ';
  }
  LOG_VERBOSE("slot assignment: %s", array_representation);

#if DISCOSYNC_DISABLE_INTERRUPTS
  // Disable other potentially interfering interrupts!
  HAL_SuspendTick();
  SUSPEND_SYSTICK();
#endif /* DISCOSYNC_DISABLE_INTERRUPTS */

  // Decide whether to do an RX phase before starting the actual discovery sync
  start_time = hs_timer_get_current_timestamp();
  if (rx_time_before_start_ms <= (DISCOSYNC_MIN_RX_TIME / HS_TIMER_FREQUENCY_MS)) {
    discosync_start_rx_tx();
  } else {
    start_time += rx_time_before_start_ms * HS_TIMER_FREQUENCY_MS;
    discosync_start_rx();
    hs_timer_schedule_start(start_time, &discosync_start_rx_tx);
  }
}


void discosync_stop(void)
{
  if (is_running) {
    is_running = false;
    hs_timer_schedule_stop();

    radio_standby();
    radio_set_rx_callback(0);
    radio_set_tx_callback(0);
    radio_set_cad_callback(0);
    radio_set_timeout_callback(0);

#if DISCOSYNC_DISABLE_INTERRUPTS
    // Re-enable other interrupts
    HAL_ResumeTick();
    RESUME_SYSTICK();
#endif /* DISCOSYNC_DISABLE_INTERRUPTS */

    LOG_VERBOSE("finished neighbor discovery with %i packets received", rx_done_cnt);
  }
}


void discosync_start_rx(void)
{
  // Set radio config and callbacks
  radio_set_rx_callback(&discosync_rx_done);
  radio_set_timeout_callback(&discosync_rx_timeout);
  radio_reset_sync_counter();
  radio_receive(0);
}


void discosync_start_rx_tx(void)
{
  if (is_running) {

    if (current_slot >= total_slots) {
      // End of discovery process
      discosync_stop();
      if (callback) {
        callback();
      }

    } else {
      radio_standby();

      // Set radio config and callbacks
      radio_set_rx_callback(&discosync_rx_done);
      radio_set_tx_callback(&discosync_tx_done);
      radio_set_timeout_callback(&discosync_rx_timeout);
      radio_reset_sync_counter();

      bool force_tx = false;
  #if DISCOSYNC_FIRST_SLOT_TX
      // if not yet received and first slot, then force TX
      if ((rx_done_cnt == 0) && (pkt.slot_idx == 0)) {
        force_tx = true;
      }
  #endif /* DISCOSYNC_FIRST_SLOT_TX */

      if (force_tx || tx_slots[current_slot]) {
        // Tx slot
        uint64_t tx_time = start_time + DISCOSYNC_SLOT_TIME * pkt.slot_idx + DISCOSYNC_TX_OFS - DISCOSYNC_TX_TRIGGER_DELAY;
  #if DISCOSYNC_MAX_RAND_TX_OFS
        tx_time -= rand() % DISCOSYNC_MAX_RAND_TX_OFS;
  #endif /* DISCOSYNC_MAX_RAND_TX_OFS */
        radio_transmit_scheduled((uint8_t*)&pkt, DISCOSYNC_PKT_LEN, tx_time);

      } else {
        // Rx slot
        radio_receive(0);
        SCHEDULE_NEXT_SLOT();
      }
      if (!force_tx) {
        current_slot++;
      }
    }
  }
}


#if DISCOSYNC_PROBABILISTIC_ASSIGNMENT

void discosync_assign_probabilistic_slots(uint8_t nr_slots)
{
  for (uint8_t i = 0; i < nr_slots; i++) {
    bool tx = ( (rand() % 10000) < (DISCOSYNC_TX_PROBABILITY * 10000) );
    if (tx) {
      tx_slots[i] = 1;
    }
  }
}

#else

void discosync_assign_static_slots(uint8_t nr_slots)
{
  uint8_t n_tx = nr_slots / 2;   // Use integer division for floor, as Tx costs are higher than Rx costs
  for (uint8_t i = 0; i < n_tx; i++) {
    uint8_t tx_idx = 0;
    do {
        tx_idx = rand() % nr_slots;
    } while (tx_slots[tx_idx]);

    // Found an empty slot
    tx_slots[tx_idx] = 1;
  }
}

#endif /* DISCOSYNC_PROBABILISTIC_ASSIGNMENT */


void discosync_start_next_slot(void)
{
  pkt.slot_idx++;

  // Make sure the radio is currently not receiving
  if (radio_get_sync_counter() > 0 && radio_get_status() == RF_RX_RUNNING) {
    // skip the next slot for a chance of successful packet reception
    radio_reset_sync_counter();
    SCHEDULE_NEXT_SLOT();
  } else {
    discosync_start_rx_tx();
  }
}


void discosync_tx_done(void)
{
  // Set the start time of the next slot
  SCHEDULE_NEXT_SLOT();
}


void discosync_rx_done(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error)
{
  discosync_packet_t* rcvd_pkt = (discosync_packet_t*)payload;

  if (crc_error) {
    //LOG_WARNING("packet with CRC error received");
    discosync_rx_timeout(true);
  } else if (size != DISCOSYNC_PKT_LEN || rcvd_pkt->protocol_id != PROTOCOL_ID_DISCOSYNC || rcvd_pkt->type != DISCOSYNC_PKT_TYPE) {
    //LOG_WARNING("invalid packet received");
    discosync_rx_timeout(false);
  } else {
    rx_done_cnt++;

    uint64_t estimated_start_time = radio_get_last_sync_timestamp() -
                                    gloria_timings[DISCOSYNC_MODULATION].txSync -
                                    DISCOSYNC_TX_OFS -
                                    DISCOSYNC_SLOT_TIME * rcvd_pkt->slot_idx;
    if (rcvd_pkt->slot_idx >= pkt.slot_idx) {
      // Sync to this node
      pkt.slot_idx = rcvd_pkt->slot_idx;
      start_time = estimated_start_time;
      // Adjust the start time of the next slot
      SCHEDULE_NEXT_SLOT();
    }
  }
}


void discosync_rx_timeout(bool crc_error)
{
  //LOG_WARNING("RX timeout (crc error: %u)", crc_error);
  uint32_t scheduled = hs_timer_get_schedule_timestamp();
  uint32_t current   = (uint32_t)hs_timer_get_current_timestamp();
  if ((scheduled - current) > DISCOSYNC_MIN_RX_TIME) {
    // restart RX
    discosync_start_rx();
  }
}


uint8_t discosync_get_rx_cnt(void)
{
  return rx_done_cnt;
}


uint64_t discosync_get_t_ref(void)
{
  return start_time;
}

