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

#ifndef RADIO_RADIO_H_
#define RADIO_RADIO_H_


/* whether to use the hardware timeout of the radio chip in receive mode; if set to 0, a software timeout will be used instead (hs timer) */
#ifndef RADIO_USE_HW_TIMEOUT
#define RADIO_USE_HW_TIMEOUT      1
#endif /* RADIO_USE_HW_TIMEOUT */


typedef enum {
  RADIO_SLEEPING_FALSE = 0,
  RADIO_SLEEPING_COLD = 1,
  RADIO_SLEEPING_WARM = 2,
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
  IRQ_MODE_RX_TX,
  IRQ_MODE_RX_TX_CRC,
} lora_irq_mode_t;

typedef struct lora_message_s {
  uint8_t* payload;
  uint8_t size;
  int8_t rssi;
  int8_t snr;
  struct lora_message_s* next;
} radio_message_t;

typedef void (* radio_irq_cb_t)(void);
typedef void (* radio_rx_cb_t)(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error);
typedef void (* radio_cad_cb_t)(bool);
typedef void (* radio_timeout_cb_t)(bool crc_error);
typedef void (* radio_tx_cb_t)(void);


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

void radio_set_rx_callback(radio_rx_cb_t callback);
void radio_set_cad_callback(radio_cad_cb_t callback);
void radio_set_timeout_callback(radio_timeout_cb_t callback);
void radio_set_tx_callback(radio_tx_cb_t callback);

void radio_set_rx_gain(bool rx_boost);

void radio_transmit(uint8_t* buffer, uint8_t size);
void radio_transmit_scheduled(uint8_t* buffer, uint8_t size, uint64_t schedule_timestamp_hs); // timestamp in hs ticks (absolute)
void radio_receive(uint32_t timeout_hs);                                                      // timeout in hs_timer ticks (relative), set timeout to 0xFFFFFF for continuous RX
void radio_receive_continuously(void);                                                        // radio will stay in RX mode even after packet reception
void radio_receive_scheduled(uint64_t schedule_timestamp_hs, uint32_t timeout_hs);            // timestamp (absolute) and timeout (relative) in hs ticks
void radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule);
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


#endif /* RADIO_RADIO_H_ */
