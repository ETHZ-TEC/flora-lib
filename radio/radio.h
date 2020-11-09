/*
 * radio.h
 *
 * Core flora radio functions.
 *
 *  Created on: 01.05.2018
 *      Author: marku
 */

#ifndef RADIO_FLORA_RADIO_H_
#define RADIO_FLORA_RADIO_H_


typedef enum {
  FALSE = 0,
  COLD = 1,
  WARM = 2,
} radio_sleeping_t;

typedef enum {
  IRQ_MODE_ALL,
  IRQ_MODE_TX,
  IRQ_MODE_RX,
  IRQ_MODE_RX_CRC,
  IRQ_MODE_RX_CRC_PREAMBLE,
  IRQ_MODE_RX_PREAMBLE,
  IRQ_MODE_RX_ONLY,
  IRQ_MODE_SYNC_RX_VALID,
  IRQ_MODE_SYNC_ONLY,
  IRQ_MODE_CAD,
  IRQ_MODE_CAD_RX,
} lora_irq_mode_t;

typedef struct lora_message_s {
  uint8_t* payload;
  uint8_t size;
  int8_t rssi;
  int8_t snr;
  struct lora_message_s* next;
} radio_message_t;


/* include all required radio drivers */
#include "radio/semtech/sx126x-radio.h"
#include "radio/radio_constants.h"
#include "radio/radio_helpers.h"
#include "radio/radio_platform.h"


void radio_init(void);
void radio_sleep(bool warm);  /* note: radio_wakeup() must be called before using the radio again after calling radio_sleep() */
void radio_reset(void);
bool radio_wakeup(void);      /* returns true if the radio was in sleep mode and has been woken successfully, false otherwise */
void radio_standby(void);

void radio_set_irq_callback(void (*callback)());
void radio_set_irq_mode(lora_irq_mode_t mode);
void radio_set_irq_direct(bool direct);

void radio_set_rx_callback(void (*callback)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error));
void radio_set_cad_callback(void (*callback)(bool));
void radio_set_timeout_callback(void (*callback)(bool crc_error));
void radio_set_tx_callback(void (*callback)());

void radio_transmit(uint8_t* buffer, uint8_t size, bool schedule);
void radio_transmit_at_precise_moment(uint8_t* buffer, uint8_t size, uint32_t time);
void radio_retransmit_at_precise_moment(uint8_t* overwrite_buffer, uint8_t overwrite_size, uint8_t size, uint64_t time);
void radio_receive_and_execute(bool boost, uint32_t schedule_timer);
void radio_receive(bool schedule, bool boost, uint32_t timeout, uint32_t rx_timeout);
void radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule);
void radio_sync_receive();
void radio_execute_manually(int64_t timer);

uint64_t  radio_get_last_sync_timestamp(void);
void      radio_reset_preamble_counter(void);
uint8_t   radio_get_preamble_counter(void);
void      radio_reset_sync_counter(void);
uint8_t   radio_get_sync_counter(void);
uint32_t  radio_get_rx_dc(void);
uint32_t  radio_get_tx_dc(void);
void      radio_dc_counter_reset(void);
uint32_t  radio_get_prr(bool reset);      /* returns the packet reception rate in [% * 10^2] */


#endif /* RADIO_FLORA_RADIO_H_ */
