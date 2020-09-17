/*
 * radio.c
 *
 *  Created on: 01.05.2018
 *      Author: marku
 */

#include "flora_lib.h"


extern void (*RadioOnDioIrqCallback)(void);
extern const struct Radio_s Radio;
extern radio_message_t* last_message_list;


/* global state (shared) */
volatile bool radio_irq_direct = false;
volatile bool radio_process_irq_in_loop_once = false;

/* internal state */
static volatile radio_sleeping_t  radio_sleeping = false;
static volatile lora_irq_mode_t   radio_mode;
static volatile bool              radio_receive_continuous = false;
static volatile bool              radio_command_scheduled = false;
static uint64_t                   radio_last_sync_timestamp = 0;
static uint8_t                    preamble_detected_counter = 0;
static uint8_t                    sync_detected_counter = 0;
static uint32_t                   rx_started_counter = 0;       // used to determine the PRR
static uint32_t                   rx_successful_counter = 0;    // used to determine the PRR
static dcstat_t                   radio_dc_rx = { 0 };
static dcstat_t                   radio_dc_tx = { 0 };

/* function pointers */
static void (*radio_irq_callback) () = NULL;
static void (*radio_rx_callback)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error) = NULL;
static void (*radio_cad_callback)(bool success) = NULL;
static void (*radio_timeout_callback)(bool crc_error) = NULL;
static void (*radio_tx_callback) () = NULL;

/* private callback functions */
void radio_irq_capture_cb(void);
void radio_timeout_cb(void);
void radio_cad_done_cb(_Bool detected);
void radio_rx_done_cb(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr, bool crc_error);
void radio_rx_error_cb(void);
void radio_rx_timeout_cb(void);
void radio_tx_done_cb(void);
void radio_tx_timeout_cb(void);
void radio_rx_sync_cb(void);
void radio_rx_preamble_cb(void);


void radio_init(void)
{
  static RadioEvents_t radio_events;      // must be static

  // assign callback functions for radio driver
  radio_events.CadDone    = radio_cad_done_cb;
  radio_events.RxDone     = radio_rx_done_cb;
  radio_events.RxError    = radio_rx_error_cb;
  radio_events.RxTimeout  = radio_rx_timeout_cb;
  radio_events.TxDone     = radio_tx_done_cb;
  radio_events.TxTimeout  = radio_tx_timeout_cb;
  radio_events.RxSync     = radio_rx_sync_cb;
  radio_events.RxPreamble = radio_rx_preamble_cb;

  bool irq_set = RADIO_READ_DIO1_PIN();

  if (!irq_set) {
    radio_reset();
  }

  Radio.Init(&radio_events);

  if (irq_set) {
    (*RadioOnDioIrqCallback)();
    radio_process_irq_in_loop_once = true;
  }

  hs_timer_capture(&radio_irq_capture_cb);
  radio_set_irq_direct(true);

  radio_sleep(true);

  // initial configuration
  uint8_t modulation      = 3;
  uint8_t band            = 34;
  int8_t  power           = 0;
  int32_t bandwidth       = -1;
  int32_t fsk_bitrate     = -1;
  int     preamble_length = -1;
  bool    implicit        = false;
  uint8_t payload_length  = 0;
  bool    crc             = true;
  radio_set_config_rx(modulation, band, bandwidth, fsk_bitrate, preamble_length, 0, implicit, payload_length, crc, true);
  radio_set_config_tx(modulation, band, power, bandwidth, fsk_bitrate, preamble_length, implicit, crc);

  // reset duty cycle counters
  dcstat_reset(&radio_dc_rx);
  dcstat_reset(&radio_dc_tx);
  RADIO_TX_STOP_IND();
  RADIO_RX_STOP_IND();

  LOG_VERBOSE("initialized");
}


void radio_set_irq_callback(void (*callback)())
{
  radio_irq_callback = callback;
}


void radio_set_irq_mode(lora_irq_mode_t mode)
{
  radio_mode = mode;

  switch (mode)
  {
  case IRQ_MODE_ALL:
    SX126xSetDioIrqParams( IRQ_RADIO_ALL,
                           IRQ_RADIO_ALL,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_TX:
    SX126xSetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_RX:
    SX126xSetDioIrqParams( IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_RX_CRC:
    SX126xSetDioIrqParams( IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
      break;
  case IRQ_MODE_RX_CRC_PREAMBLE:
    SX126xSetDioIrqParams( IRQ_PREAMBLE_DETECTED | IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_PREAMBLE_DETECTED | IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
      break;
  case IRQ_MODE_RX_PREAMBLE:
    SX126xSetDioIrqParams( IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_RX_ONLY:
    SX126xSetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
  case IRQ_MODE_SYNC_RX_VALID:
    SX126xSetDioIrqParams( IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE,
                           IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_SYNC_ONLY:
    SX126xSetDioIrqParams( IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID,
                           IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_CAD:
    SX126xSetDioIrqParams( IRQ_CAD_ACTIVITY_DETECTED | IRQ_CAD_DONE,
                           IRQ_CAD_ACTIVITY_DETECTED | IRQ_CAD_DONE,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;
  case IRQ_MODE_CAD_RX:
    SX126xSetDioIrqParams( IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE,
                           IRQ_RADIO_NONE );
    break;

  default:
    break;
  }
}


void radio_set_irq_direct(bool direct)
{
  radio_irq_direct = direct;
}


void radio_set_cad_callback(void (*callback)(bool))
{
  radio_cad_callback = callback;
}


void radio_set_rx_callback(void (*callback)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error))
{
  radio_rx_callback = callback;
}


void radio_set_timeout_callback(void (*callback)(bool crc_error))
{
  radio_timeout_callback = callback;
}


void radio_set_tx_callback(void (*callback)())
{
  radio_tx_callback = callback;
}


void radio_sleep(bool warm)
{
  if (!radio_sleeping) {
    if (warm) {
      Radio.Sleep();
    }
    else {
      Radio.ColdSleep();
    }
    radio_sleeping = warm + 1;
    RADIO_TX_STOP_IND();
    RADIO_RX_STOP_IND();
    dcstat_stop(&radio_dc_rx);
    dcstat_stop(&radio_dc_tx);
  }
}


void radio_reset(void)
{
  SX126xReset();
  RADIO_RX_STOP_IND();
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_rx);
  dcstat_stop(&radio_dc_tx);
}


bool radio_wakeup(void)
{
  if (radio_sleeping) {
    SX126xWakeup();
    if (radio_sleeping == COLD) {
      SX126xSetDio2AsRfSwitchCtrl(true);
      SX126xSetRegulatorMode( USE_DCDC );
    }
    radio_sleeping = false;
    return true;
  }
  return false;
}


/* puts the radio into idle mode */
void radio_standby(void)
{
  Radio.Standby();
  radio_sleeping = false;

  RADIO_RX_STOP_IND();
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_rx);
  dcstat_stop(&radio_dc_tx);
}


void radio_irq_capture_cb(void)
{
  if (radio_mode != IRQ_MODE_SYNC_ONLY) {
    hs_timer_timeout_stop();
  }

  if (radio_irq_callback) {
    radio_irq_callback();
  }
  else {
    (*RadioOnDioIrqCallback)();

    if (radio_irq_direct) {
      Radio.IrqProcess();
    }
  }

#ifdef FLORA_DEBUG
  led_set_event_blink(0, 0);
#endif
}


void radio_timeout_cb(void)
{
  radio_set_rx_callback(NULL);

  if (radio_timeout_callback) {
      void (*tmp)(bool) = radio_timeout_callback;
      radio_set_timeout_callback(NULL);
      radio_standby();

      if(tmp) {
        tmp(false);
    }
  }

#ifdef FLORA_DEBUG
  CLI_LOG("MCU interrupt was triggered!", CLI_LOG_LEVEL_WARNING);
#endif
}


/**
 * SX1262 Callbacks
 */

void radio_cad_done_cb(bool detected)
{
  if (radio_cad_callback) {
    radio_set_timeout_callback(NULL);
    void (*tmp)(bool) = radio_cad_callback;
    radio_set_cad_callback(NULL);
    if(tmp) {
      tmp(detected);
    }
  }
  else if (radio_timeout_callback && !detected) {
    void (*tmp)(bool) = radio_timeout_callback;
    radio_set_timeout_callback(NULL);

    if(tmp) {
      tmp(false);
    }
  }

#ifdef FLORA_DEBUG
  if (detected) {
    CLI_LOG("CAD detected signal!", CLI_LOG_LEVEL_INFO);
  }
  else {
    CLI_LOG("CAD detected NO signal!", CLI_LOG_LEVEL_INFO);
  }
#endif
}


void radio_rx_done_cb(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error)
{
  RADIO_RX_STOP_IND();
  dcstat_stop(&radio_dc_rx);
  if (!crc_error) {
    rx_successful_counter++;
  }

  if (radio_rx_callback) {
    radio_set_timeout_callback(NULL);

    void (*tmp)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error) = radio_rx_callback;
    radio_set_rx_callback(NULL);

    if (tmp) {
      tmp(payload, size, rssi, snr, crc_error);
    }
  }
  else if(radio_receive_continuous) {
    SX126xSetRxBoosted(0);
    RADIO_RX_START_IND();
    dcstat_start(&radio_dc_rx);
  }

#ifdef FLORA_DEBUG
  if (!crc_error) {
    radio_message_t* message = malloc(sizeof(radio_message_t));

    if (message != NULL) {
      uint8_t* message_payload = malloc(size);
      if (message_payload != NULL) {
        memcpy(message_payload, payload, size);

        message->payload = message_payload;
        message->size = size;
        message->rssi = rssi;
        message->snr = snr;
        message->next = NULL;

        if (last_message_list == NULL) {
          last_message_list = message;
        }
        else {
          radio_message_t* tmp = last_message_list;

          while (tmp->next != NULL) {
            tmp = tmp->next;
          }

          tmp->next = message;
        }
      }
      else {
        free(message);
      }
    }
  }
  else {
    LOG_ERROR("CRC Error on message reception");
  }
#endif
}


void radio_rx_error_cb(void)
{
  RADIO_TX_STOP_IND();
  RADIO_RX_STOP_IND();

  if(!radio_rx_callback && radio_receive_continuous) {
    SX126xSetRxBoosted(0);
    RADIO_RX_START_IND();
  }
  else {
    dcstat_stop(&radio_dc_rx);
    void (*tmp)(bool) = radio_timeout_callback;
    radio_set_timeout_callback(NULL);

    if(tmp) {
      tmp(true);
    }
  }

#ifdef FLORA_DEBUG
  LOG_WARNING("CRC Error Timeout");
#endif
}


void radio_rx_timeout_cb(void)
{
  RADIO_RX_STOP_IND();
  dcstat_stop(&radio_dc_rx);

  void (*tmp)(bool) = radio_timeout_callback;
  radio_set_timeout_callback(NULL);

  if(tmp) {
    tmp(false);
  }

#ifdef FLORA_DEBUG
  LOG_WARNING("Rx Timeout");
#endif
}


void radio_rx_sync_cb(void)
{
  RADIO_RX_STOP_IND();
  RADIO_RX_START_IND();
  radio_last_sync_timestamp = hs_timer_get_capture_timestamp();
  if (sync_detected_counter < 255) {
    sync_detected_counter++;
  }
  rx_started_counter++;
}


void radio_rx_preamble_cb(void)
{
  RADIO_RX_STOP_IND();
  RADIO_RX_START_IND();
  if (preamble_detected_counter < 255) {
    preamble_detected_counter++;
  }
}


void radio_tx_done_cb(void)
{
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_tx);

  radio_set_timeout_callback(NULL);

  void (*tmp)() = radio_tx_callback;
  radio_set_tx_callback(NULL);

  if (tmp) {
    tmp();
  }
}


void radio_tx_timeout_cb(void)
{
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_tx);

  void (*tmp)(bool) = radio_timeout_callback;
  radio_set_timeout_callback(NULL);

  if (tmp) {
    tmp(false);
  }
}


/**
 * RX / TX functions
 */

static void radio_execute(void)
{
  RADIO_SET_NSS_PIN();

  switch (Radio.GetStatus()) {
    case RF_RX_RUNNING:
      RADIO_RX_START_IND();
      dcstat_start(&radio_dc_rx);
      break;
    case RF_TX_RUNNING:
      RADIO_TX_START_IND();
      dcstat_start(&radio_dc_tx);
      break;
    default:
      break;
  }
  hs_timer_schedule_stop();
  radio_command_scheduled = false;
}


void radio_transmit(uint8_t* buffer, uint8_t size, bool schedule)
{
  radio_set_payload(buffer, 0, size);
  if (schedule) {
    radio_command_scheduled = true;
    SX126xSetTxWithoutExecute(0);
  }
  else {
    SX126xSetTx(0);
    hs_timer_set_schedule_timestamp((uint32_t) hs_timer_get_current_timestamp());
    RADIO_TX_START_IND();
    dcstat_start(&radio_dc_tx);
  }
}


void radio_transmit_at_precise_moment(uint8_t* buffer, uint8_t size, uint32_t time)
{
  radio_command_scheduled = true;
  radio_set_payload(buffer, 0, size);
  SX126xSetTxWithoutExecute(0);

  hs_timer_schedule(time, &radio_execute);
}


void radio_retransmit_at_precise_moment(uint8_t* overwrite_buffer, uint8_t overwrite_size, uint8_t size, uint64_t time)
{
  radio_command_scheduled = true;
  radio_set_packet_params_and_size(size);
  SX126xSetPayload(overwrite_buffer, overwrite_size);
  SX126xSetTxWithoutExecute(0);

  hs_timer_schedule(time, &radio_execute);
}


void radio_execute_manually(int64_t timer)
{
  if (radio_command_scheduled) {
    if (timer == -1) {
      hs_timer_set_schedule_timestamp((uint32_t) hs_timer_get_current_timestamp());
      radio_execute();
    }
    else {
      hs_timer_schedule(timer, &radio_execute);
    }
  }
}


void radio_set_tx(uint64_t timestamp)
{
  radio_command_scheduled = true;
  SX126xSetTxWithoutExecute(0);
  hs_timer_schedule(timestamp, &radio_execute);
}


void radio_set_rx(uint64_t timestamp, uint32_t timeout)
{
  radio_command_scheduled = true;
  SX126xSetRxBoostedWithoutExecute(timeout);
  hs_timer_schedule(timestamp, &radio_execute);

  if (radio_mode != IRQ_MODE_SYNC_ONLY) {
    hs_timer_timeout_start(timestamp);
  }
  else {
    hs_timer_timeout_start(0);
  }
}


void radio_receive_and_execute(bool boost, uint32_t schedule_timer)
{
  radio_receive_continuous = false;
  hs_timer_timeout(radio_calculate_timeout(false), &radio_timeout_cb);
  radio_command_scheduled = true;

  uint32_t hardware_timeout = radio_calculate_timeout(true);

  if (boost) {
    SX126xSetRxBoostedWithoutExecute(hardware_timeout);
  }
  else {
    SX126xSetRxWithoutExecute(hardware_timeout);
  }

  hs_timer_schedule(schedule_timer, &radio_execute);
}


void radio_receive(bool schedule, bool boost, uint32_t timeout, uint32_t rx_timeout)
{
  // default value for "rx_timeout": 0
  radio_receive_continuous = !schedule;

  if (schedule) {
    if (timeout) {
      hs_timer_timeout(timeout  + (uint32_t) (85.2 * HS_TIMER_FREQUENCY_US), &radio_timeout_cb);
    }
    else {
      hs_timer_timeout(radio_calculate_timeout(false), &radio_timeout_cb);
    }

    if (boost) {
      if(rx_timeout) {
        SX126xSetRxBoostedWithoutExecute(rx_timeout);
      }
      else {
        SX126xSetRxBoostedWithoutExecute(radio_calculate_timeout(true));
      }

    }
    else {
      if(rx_timeout) {
        SX126xSetRxWithoutExecute(rx_timeout);
      }
      else {
        SX126xSetRxWithoutExecute(radio_calculate_timeout(true));
      }
    }

    radio_command_scheduled = true;
  }
  else {
    if (boost) {
      SX126xSetRxBoosted(timeout);
    }
    else {
      SX126xSetRx(timeout);
    }
    RADIO_RX_START_IND();
    dcstat_start(&radio_dc_rx);
  }
}


void radio_sync_receive(void)
{
  radio_receive_continuous = false;
  SX126xSetRxBoosted(0);
  RADIO_RX_START_IND();
  dcstat_start(&radio_dc_rx);
}


void radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule)
{
  radio_receive_continuous = false;

  rx = (uint64_t) rx; // in 15.625 us steps
  sleep = (uint64_t) sleep; // in 15.625 us steps (see Figure 13-2: "RX Duty Cycle Energy Profile" in SX1262 datasheet)

  if (schedule) {
    SX126xSetRxDutyCycleWithoutExecute(rx, sleep);
    radio_command_scheduled = true;
  }
  else {
    Radio.SetRxDutyCycle(rx, sleep);
  }

}


uint64_t radio_get_last_sync_timestamp(void)
{
  return radio_last_sync_timestamp;
}


void radio_reset_preamble_counter(void)
{
  preamble_detected_counter = 0;
}


uint8_t radio_get_preamble_counter(void)
{
  return preamble_detected_counter;
}

void radio_reset_sync_counter(void)
{
  sync_detected_counter = 0;
}


uint8_t radio_get_sync_counter(void)
{
  return sync_detected_counter;
}


uint32_t radio_get_rx_dc(void)
{
  return dcstat_get_dc(&radio_dc_rx);
}


uint32_t radio_get_tx_dc(void)
{
  return dcstat_get_dc(&radio_dc_tx);
}


void radio_dc_counter_reset(void)
{
  dcstat_reset(&radio_dc_rx);
  dcstat_reset(&radio_dc_tx);
}


uint32_t radio_get_prr(bool reset)
{
  uint32_t prr = 0;
  if (rx_started_counter) {
    prr = 10000 * rx_successful_counter / rx_started_counter;
  }
  if (reset) {
    rx_successful_counter = rx_started_counter = 0;
  }
  return prr;
}
