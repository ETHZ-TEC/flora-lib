/*
 * lora.c
 *
 *  Created on: 01.05.2018
 *      Author: marku
 */

#include "flora_lib.h"


extern void (*RadioOnDioIrqCallback)(void);

extern volatile bool    cli_initialized;

/* global state (shared) */
RadioEvents_t           radio_RadioEvents;
volatile bool           radio_command_scheduled = false;
volatile bool           radio_receive_continuous = false;
volatile bool           radio_initialized = false;
volatile bool           radio_irq_direct = false;
volatile bool           radio_process_irq_in_loop_once = false;
bool                    radio_mcu_timeout_flag = false;
uint64_t                radio_last_sync_timestamp;
dcstat_t                radio_dc_rx = { 0 };
dcstat_t                radio_dc_tx = { 0 };

/* internal state */
static radio_sleeping_t radio_sleeping = FALSE;
static lora_irq_mode_t  radio_mode;
static radio_message_t* last_message_list = NULL;
static uint8_t          preamble_detected_counter = 0;
static uint8_t          sync_detected_counter = 0;


void radio_irq_capture_callback();
void radio_schedule_callback();
void radio_mcu_timeout_callback();

void (*radio_irq_callback) () = NULL;
void (*radio_rx_callback)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error) = NULL;
void (*radio_cad_callback)(bool success) = NULL;
void (*radio_timeout_callback)(bool crc_error) = NULL;
void (*radio_tx_callback) () = NULL;

void radio_cad_done_cb(_Bool detected);
void radio_rx_done_cb(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr, bool crc_error);
void radio_rx_error_cb(void);
void radio_rx_timeout_cb(void);
void radio_tx_done_cb(void);
void radio_tx_timeout_cb(void);
void radio_rx_sync_cb(void);
void radio_rx_preamble_cb(void);


void radio_init()
{
  radio_initialized = false;

  radio_RadioEvents.CadDone = radio_cad_done_cb;
  radio_RadioEvents.RxDone = radio_rx_done_cb;
  radio_RadioEvents.RxError = radio_rx_error_cb;
  radio_RadioEvents.RxTimeout = radio_rx_timeout_cb;
  radio_RadioEvents.TxDone = radio_tx_done_cb;
  radio_RadioEvents.TxTimeout = radio_tx_timeout_cb;
  radio_RadioEvents.RxSync = radio_rx_sync_cb;
  radio_RadioEvents.RxPreamble = radio_rx_preamble_cb;

  bool irq_set = RADIO_READ_DIO1_PIN();

  if (!irq_set) {
    radio_reset();
  }

  Radio.Init(&radio_RadioEvents);

  if (irq_set) {
    (*RadioOnDioIrqCallback)();
    radio_process_irq_in_loop_once = true;
  }

  hs_timer_capture(&radio_irq_capture_callback);
  radio_set_irq_direct(true);

  radio_sleep(true);

  // initial configuration
  uint8_t modulation = 3;
  uint8_t band = 34;
  int8_t power = 0;
  int32_t bandwidth = -1;
  int32_t fsk_bitrate = -1;
  int preamble_length = -1;
  bool implicit = false;
  uint8_t payload_length = 0;
  bool crc = true;
  radio_set_config_rx(modulation, band, bandwidth, fsk_bitrate, preamble_length, 0, implicit, payload_length, crc, true);
  radio_set_config_tx(modulation, band, power, bandwidth, fsk_bitrate, preamble_length, implicit, crc);

  // reset duty cycle counters
  dcstat_reset(&radio_dc_rx);
  dcstat_reset(&radio_dc_tx);

  radio_initialized = true;
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

void radio_update()
{
  if ((!radio_irq_direct || radio_process_irq_in_loop_once) && cli_initialized) {
    Radio.IrqProcess();
    radio_process_irq_in_loop_once = false;
  }

  radio_message_t* tmp = last_message_list;

  while (tmp != NULL) {
    radio_message_t* tmp_next;
    radio_print_message(tmp);
    tmp_next = tmp->next;
    free(tmp->payload);
    free(tmp);
    tmp = tmp_next;
  }

  last_message_list = NULL;
}

void radio_sleep(bool warm)
{
  if(radio_initialized && !radio_sleeping) {
    SleepParams_t params = { 0 };
    if (warm) {
      params.Fields.WarmStart = 1;
    }
    else {
      params.Fields.WarmStart = 0;
    }
    SX126xSetSleep( params );

    radio_sleeping = warm + 1;

    dcstat_stop(&radio_dc_rx);
    dcstat_stop(&radio_dc_tx);
  }
}

void radio_reset(void)
{
  SX126xReset();
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
    radio_sleeping = FALSE;
    return true;
  }
  return false;
}

/* puts the radio into idle mode */
void radio_standby(void)
{
  Radio.Standby();
  radio_sleeping = FALSE;
}

void radio_irq_capture_callback(void)
{
  if (radio_mode != IRQ_MODE_SYNC_ONLY) {
    radio_stop_mcu_timeout();
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


void radio_schedule_callback(void)
{
  RADIO_SET_NSS_PIN(); // Execute radio command
  radio_command_scheduled = false;
}

void radio_mcu_timeout_callback(void)
{
  radio_mcu_timeout_flag = true;

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
  if (radio_rx_callback) {
    dcstat_stop(&radio_dc_rx);
    radio_set_timeout_callback(NULL);

    void (*tmp)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error) = radio_rx_callback;
    radio_set_rx_callback(NULL);

    if (tmp) {
      tmp(payload, size, rssi, snr, crc_error);
    }
  }
  else if(radio_receive_continuous) {
    SX126xSetRxBoosted(0);
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
  if(!radio_rx_callback && radio_receive_continuous) {
    SX126xSetRxBoosted(0);
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
  radio_last_sync_timestamp = hs_timer_get_capture_timestamp();
  if (sync_detected_counter < 255) {
    sync_detected_counter++;
  }
}


void radio_rx_preamble_cb(void)
{
  if (preamble_detected_counter < 255) {
    preamble_detected_counter++;
  }
}


void radio_tx_done_cb(void)
{
  radio_set_timeout_callback(NULL);

  void (*tmp)() = radio_tx_callback;
  radio_set_tx_callback(NULL);

  if (tmp) {
    tmp();
  }
  dcstat_stop(&radio_dc_tx);
}


void radio_tx_timeout_cb(void)
{
  void (*tmp)(bool) = radio_timeout_callback;
  radio_set_timeout_callback(NULL);

  if (tmp) {
    tmp(false);
  }
  dcstat_stop(&radio_dc_tx);
}


void radio_set_mcu_timeout(uint64_t offset)
{
  hs_timer_timeout(offset, &radio_mcu_timeout_callback);
}


void radio_start_mcu_timeout(uint64_t compare_timeout)
{
  if (radio_mode != IRQ_MODE_SYNC_ONLY) {
    hs_timer_timeout_start(compare_timeout);
  }
  else {
    hs_timer_timeout_start(0);
  }
}


void radio_stop_mcu_timeout(void)
{
  hs_timer_timeout_stop();
}


void radio_stop_schedule(void)
{
  hs_timer_schedule_stop();
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
