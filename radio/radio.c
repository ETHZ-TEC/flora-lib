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

/* shared state */
volatile bool radio_irq_direct = false;

/* internal state */
static volatile radio_sleeping_t  radio_sleeping = RADIO_SLEEPING_FALSE;
static bool                       rx_boosted = true;
static uint64_t                   radio_last_sync_timestamp = 0;
static uint8_t                    preamble_detected_counter = 0;
static uint8_t                    sync_detected_counter = 0;
static uint32_t                   rx_started_counter = 0;       // used to determine the PRR
static uint32_t                   rx_successful_counter = 0;    // used to determine the PRR
static dcstat_t                   radio_dc_rx = { 0 };
static dcstat_t                   radio_dc_tx = { 0 };

/* function pointers */
static radio_irq_cb_t     radio_irq_callback;
static radio_rx_cb_t      radio_rx_callback      = 0;
static radio_cad_cb_t     radio_cad_callback     = 0;
static radio_timeout_cb_t radio_timeout_callback = 0;
static radio_tx_cb_t      radio_tx_callback      = 0;

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


// restore basic radio configuration (e.g. after cold sleep)
static void radio_restore_config(bool reset)
{
  static RadioEvents_t radio_events;

  // perform a full radio reset?
  if (reset) {

    // assign callback functions for radio driver
    radio_events.CadDone    = radio_cad_done_cb;
    radio_events.RxDone     = radio_rx_done_cb;
    radio_events.RxError    = radio_rx_error_cb;
    radio_events.RxTimeout  = radio_rx_timeout_cb;
    radio_events.TxDone     = radio_tx_done_cb;
    radio_events.TxTimeout  = radio_tx_timeout_cb;
    radio_events.RxSync     = radio_rx_sync_cb;
    radio_events.RxPreamble = radio_rx_preamble_cb;

    // NOTE: Radio.Init() performs a hard reset of the SX1262 chip
    Radio.Init(&radio_events);

    // sets the center frequency and performs image calibration if necessary -> can take up to 2ms (see datasheet p.56)
    // NOTE: Image calibration is valid for the whole frequency band (863 - 870MHz), no recalibration necessary if another frequency within this band is selected later on
    Radio.SetChannel(radio_bands[RADIO_DEFAULT_BAND].centerFrequency);

  } else {
    // make sure the radio is in STDBY_XOSC mode and set crystal trim values
    SX126xSetXoscTrim();
  }

  // max LNA gain, increase current by ~2mA for around ~3dB in sensitivity
  if (rx_boosted) {
    radio_set_rx_gain(true);
  }
}


void radio_init(void)
{
  if (RADIO_READ_DIO1_PIN()) {
    LOG_WARNING("SX1262 DIO1 pin is high");
  }
  radio_restore_config(true);

  hs_timer_capture(&radio_irq_capture_cb);
  radio_set_irq_direct(true);

  // note: RX/TX config has to be set by the user / application

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
  uint16_t radio_irq_mask = IRQ_RADIO_ALL;

  switch (mode)
  {
  case IRQ_MODE_ALL:
    radio_irq_mask = IRQ_RADIO_ALL;
    break;
  case IRQ_MODE_TX:
    radio_irq_mask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT;
    break;
  case IRQ_MODE_RX:
    radio_irq_mask = IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0);
    break;
  case IRQ_MODE_RX_CRC:
    radio_irq_mask = IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0) | IRQ_CRC_ERROR;
    break;
  case IRQ_MODE_RX_CRC_PREAMBLE:
    radio_irq_mask = IRQ_PREAMBLE_DETECTED | IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0) | IRQ_CRC_ERROR;
    break;
  case IRQ_MODE_RX_PREAMBLE:
    radio_irq_mask = IRQ_PREAMBLE_DETECTED | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0);
    break;
  case IRQ_MODE_RX_ONLY:
    radio_irq_mask = IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0);
    break;
  case IRQ_MODE_SYNC_RX_VALID:
    radio_irq_mask = IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE;
    break;
  case IRQ_MODE_SYNC_ONLY:
    radio_irq_mask = IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID;
    break;
  case IRQ_MODE_CAD:
    radio_irq_mask = IRQ_CAD_ACTIVITY_DETECTED | IRQ_CAD_DONE;
    break;
  case IRQ_MODE_CAD_RX:
    radio_irq_mask = IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_RX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0);
    break;
  case IRQ_MODE_RX_TX:
    radio_irq_mask = IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_TX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0);
    break;
  case IRQ_MODE_RX_TX_CRC:
    radio_irq_mask = IRQ_HEADER_ERROR | IRQ_HEADER_VALID | IRQ_SYNCWORD_VALID | IRQ_RX_DONE | IRQ_TX_DONE | (RADIO_USE_HW_TIMEOUT ? IRQ_RX_TX_TIMEOUT : 0) | IRQ_CRC_ERROR;
    break;
  default:
    break;
  }

  SX126xSetDioIrqParams(radio_irq_mask,
                        radio_irq_mask,
                        IRQ_RADIO_NONE,
                        IRQ_RADIO_NONE);
}


void radio_set_irq_direct(bool direct)
{
  radio_irq_direct = direct;
}


void radio_set_cad_callback(radio_cad_cb_t callback)
{
  radio_cad_callback = callback;
}


void radio_set_rx_callback(radio_rx_cb_t callback)
{
  radio_rx_callback = callback;
}


void radio_set_timeout_callback(radio_timeout_cb_t callback)
{
  radio_timeout_callback = callback;
}


void radio_set_tx_callback(radio_tx_cb_t callback)
{
  radio_tx_callback = callback;
}


void radio_sleep(bool warm)
{
  if (!radio_sleeping) {
    if (warm) {
      Radio.Sleep();
      radio_sleeping = RADIO_SLEEPING_WARM;
    }
    else {
      Radio.ColdSleep();
      radio_sleeping = RADIO_SLEEPING_COLD;
    }
    RADIO_TX_STOP_IND();
    RADIO_RX_STOP_IND();
    dcstat_stop(&radio_dc_rx);
    dcstat_stop(&radio_dc_tx);
  }
}


void radio_reset(void)
{
  RADIO_RX_STOP_IND();
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_rx);
  dcstat_stop(&radio_dc_tx);
  radio_restore_config(true);   // calls Radio.Init() and performs a radio chip reset
}


bool radio_wakeup(void)
{
  if (radio_sleeping) {
    if (radio_sleeping == RADIO_SLEEPING_COLD) {
      /* radio config is lost and must be restored */
      radio_restore_config(true);
    } else {
      SX126xWakeup();
      radio_restore_config(false);
    }
    radio_sleeping = RADIO_SLEEPING_FALSE;
    return true;
  }
  return false;
}


/* puts the radio into idle mode */
void radio_standby(void)
{
  if (radio_sleeping) {
    radio_wakeup();       // wake radio if it is still in sleep mode
  }

  //ENTER_CRITICAL_SECTION( );
  // temporarily force into RC mode (bug workaround -> 10us offset for TX start)
  SX126xSetStandby( STDBY_RC );
  SX126xSetXoscTrim();
  Radio.Standby();
  //LEAVE_CRITICAL_SECTION();

  RADIO_RX_STOP_IND();
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_rx);
  dcstat_stop(&radio_dc_tx);
}


void radio_irq_capture_cb(void)
{
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
  radio_rx_callback = 0;

  if (radio_timeout_callback) {
    radio_timeout_cb_t tmp = radio_timeout_callback;
    radio_timeout_callback = 0;
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
    radio_cad_cb_t tmp = radio_cad_callback;
    radio_cad_callback = 0;
    if(tmp) {
      tmp(detected);
    }
  }
  else if (radio_timeout_callback && !detected) {
    radio_timeout_cb_t tmp = radio_timeout_callback;
    radio_timeout_callback = 0;
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
#if !RADIO_USE_HW_TIMEOUT
  hs_timer_timeout_stop();
#endif /* RADIO_USE_HW_TIMEOUT */

  if (radio_rx_callback) {
    radio_set_timeout_callback(NULL);

    radio_rx_cb_t tmp = radio_rx_callback;
    if (SX126xGetOperatingMode() != MODE_RX_CONTINUOUS) {
      radio_rx_callback = 0;
    }

    if (tmp) {
      tmp(payload, size, rssi, snr, crc_error);
    }
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
  RADIO_RX_STOP_IND();
  dcstat_stop(&radio_dc_rx);

#if !RADIO_USE_HW_TIMEOUT
  hs_timer_timeout_stop();
#endif /* RADIO_USE_HW_TIMEOUT */

  radio_timeout_cb_t tmp = radio_timeout_callback;
  if (SX126xGetOperatingMode() != MODE_RX_CONTINUOUS) {
    radio_timeout_callback = 0;
  }
  if(tmp) {
    tmp(true);
  }

#ifdef FLORA_DEBUG
  LOG_WARNING("CRC Error Timeout");
#endif
}


void radio_rx_timeout_cb(void)
{
  RADIO_RX_STOP_IND();
  dcstat_stop(&radio_dc_rx);

#if !RADIO_USE_HW_TIMEOUT
  hs_timer_timeout_stop();
#endif /* RADIO_USE_HW_TIMEOUT */

  radio_timeout_cb_t tmp = radio_timeout_callback;
  radio_timeout_callback = 0;
  if(tmp) {
    tmp(false);
  }

#ifdef FLORA_DEBUG
  LOG_WARNING("Rx Timeout");
#endif
}


void radio_rx_sync_cb(void)
{
  radio_last_sync_timestamp = hs_timer_get_capture_timestamp();
  if (sync_detected_counter < 255) {
    sync_detected_counter++;
  }
  rx_started_counter++;
}


void radio_rx_preamble_cb(void)
{
  if (preamble_detected_counter < 255) {
    preamble_detected_counter++;
  }
}


void radio_tx_done_cb(void)
{
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_tx);

  radio_set_timeout_callback(NULL);

  radio_tx_cb_t tmp = radio_tx_callback;
  radio_tx_callback = 0;
  if (tmp) {
    tmp();
  }
}


void radio_tx_timeout_cb(void)
{
  RADIO_TX_STOP_IND();
  dcstat_stop(&radio_dc_tx);

  radio_timeout_cb_t tmp = radio_timeout_callback;
  radio_timeout_callback = 0;
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
      RADIO_RX_STOP_IND();
      RADIO_TX_START_IND();
      dcstat_start(&radio_dc_tx);
      break;
    default:
      break;
  }
  hs_timer_schedule_stop();
}


void radio_transmit(uint8_t* buffer, uint8_t size)
{
  if (radio_sleeping) return;      // abort if radio is still in sleep mode

  if (buffer) {
    radio_set_payload(buffer, size);
  }
  Radio.Tx(0, false);
  hs_timer_set_schedule_timestamp(hs_timer_get_counter());
  RADIO_RX_STOP_IND();
  RADIO_TX_START_IND();
  dcstat_start(&radio_dc_tx);
}


void radio_transmit_scheduled(uint8_t* buffer, uint8_t size, uint64_t schedule_timestamp_hs)
{
  if (radio_sleeping) return;      // abort if radio is still in sleep mode

  if (buffer) {
    radio_set_payload(buffer, size);
  }
  Radio.Tx(0, true);
  hs_timer_schedule_start(schedule_timestamp_hs, &radio_execute);
}


void radio_execute_manually(int64_t timestamp_hs)
{
  if (RADIO_READ_NSS_PIN() == 0) {    // command scheduled?
    if (timestamp_hs < 0) {
      hs_timer_set_schedule_timestamp(hs_timer_get_counter());
      radio_execute();
    }
    else {
      hs_timer_schedule_start(timestamp_hs, &radio_execute);
    }
  }
}


void radio_receive_scheduled(uint64_t schedule_timestamp_hs, uint32_t timeout_hs)
{
  uint32_t timeout_ms = 0;

  if (radio_sleeping) return;      // abort if radio is still in sleep mode

#if RADIO_USE_HW_TIMEOUT
  // convert to milliseconds
  timeout_ms = HS_TIMER_TICKS_TO_MS(timeout_hs);
  if (timeout_ms > RADIO_TIMER_MAX_TIMEOUT_MS) {
    timeout_ms = RADIO_TIMER_MAX_TIMEOUT_MS;        // set to max. allowed value
  }
#else  /* RADIO_USE_HW_TIMEOUT */
  if (timeout_hs) {
    hs_timer_timeout_start(schedule_timestamp_hs + timeout_hs, &radio_timeout_cb);
  }
#endif /* RADIO_USE_HW_TIMEOUT */

  Radio.Rx(timeout_ms, false, true);

  hs_timer_schedule_start(schedule_timestamp_hs, &radio_execute);
}


void radio_receive(uint32_t timeout_hs)
{
  uint32_t timeout_ms = 0;

  if (radio_sleeping) return;      // abort if radio is still in sleep mode

#if RADIO_USE_HW_TIMEOUT
  // convert to milliseconds
  timeout_ms = HS_TIMER_TICKS_TO_MS(timeout_hs);
  if (timeout_ms > RADIO_TIMER_MAX_TIMEOUT_MS) {
    timeout_ms = RADIO_TIMER_MAX_TIMEOUT_MS;        // set to max. allowed value
  }
#else  /* RADIO_USE_HW_TIMEOUT */
  if (timeout_hs) {
    hs_timer_timeout_start(hs_timer_get_current_timestamp() + timeout_hs, &radio_timeout_cb);
  }
#endif /* RADIO_USE_HW_TIMEOUT */

  Radio.Rx(timeout_ms, false, false);
  RADIO_RX_START_IND();
  dcstat_start(&radio_dc_rx);
}


void radio_receive_continuously(void)
{
  if (radio_sleeping) return;      // abort if radio is still in sleep mode

  Radio.Rx(0, true, false);
  RADIO_RX_START_IND();
  dcstat_start(&radio_dc_rx);
}


void radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule)
{
  if (radio_sleeping) return;      // abort if radio is still in sleep mode

  Radio.SetRxDutyCycle(rx, sleep, schedule);
}


void radio_set_rx_gain(bool rx_boost)
{
  rx_boosted = rx_boost;
  if (rx_boosted) {
    SX126xWriteRegister( REG_RX_GAIN, 0x96 ); // max LNA gain, increase current by ~2mA for around ~3dB in sensitivity
  } else {
    SX126xWriteRegister( REG_RX_GAIN, 0x94 ); // default gain
  }
}


/**
 * functions to query / reset stats
 */

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
