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

#ifndef RADIO_RADIO_CONSTANTS_H_
#define RADIO_RADIO_CONSTANTS_H_


// SX1262 Timings from datasheet in us
#define RADIO_TIME_SLEEP_COLD_TO_STBY   3500 // cold start (no data retention)
#define RADIO_TIME_SLEEP_WARM_TO_STBY   340 // warm start (with data retention)
#define RADIO_TIME_STBY_RC_TO_STBY_XOSC 31
#define RADIO_TIME_STBY_RC_TO_FS        50
#define RADIO_TIME_STBY_RC_TO_RX        83
#define RADIO_TIME_STBY_RC_TO_TX        126
#define RADIO_TIME_STBY_XOSC_TO_FS      40
#define RADIO_TIME_STBY_XOSC_TO_RX      62
#define RADIO_TIME_STBY_XOSC_TO_TX      105
#define RADIO_TIME_FS_TO_RX             41
#define RADIO_TIME_FS_TO_TX             76
#define RADIO_TIME_RX_TO_FS             15
#define RADIO_TIME_RX_TO_TX             92

#define RADIO_TIME_PA_RAMP_UP           40 // us
#define RADIO_SPI_SPEED                 12580000 // 12 MeBit/s

// Radio complete Wake-up Time with margin for temperature compensation
#define RADIO_WAKEUP_TIME               3 // [ms]

// SX1262 power values
#define RADIO_MAX_POWER                 22 // dBm (SX1262)
#define RADIO_MIN_POWER                 -9 // dBm (SX1262)

#define RADIO_NUM_MODULATIONS           11
#define RADIO_NUM_CAD_PARAMS            8
#define RADIO_NUM_BANDS                 52

#ifndef US915
#define RADIO_DEFAULT_BAND              40
#else
#define RADIO_DEFAULT_BAND              0
#endif

#define RADIO_MAX_PAYLOAD_SIZE          255

#define RADIO_CLOCK_DRIFT_PPM           10      // +-10ppm
#define RADIO_TIMER_PERIOD_NS           15625U
#define RADIO_TIMER_FREQUENCY           64000   // Hz
#define RADIO_TIMER_NO_TIMEOUT          0x0
#define RADIO_TIMER_MAX_TIMEOUT         0xFFFFFE
#define RADIO_TIMER_MAX_TIMEOUT_MS      RADIO_TICKS_TO_MS(RADIO_TIMER_MAX_TIMEOUT)
#define RADIO_TIMER_RX_CONTINUOUS       0xFFFFFF

#define RADIO_LORA_SF_TO_MODULATION_INDEX(sf)     ((sf >= 5 && sf <= 12) ? (12 - sf) : 0)

#define RADIO_HSTICKS_TO_RADIOTIMER(t)            (uint32_t)((uint64_t)(t) * RADIO_TIMER_FREQUENCY / HS_TIMER_FREQUENCY)  // convert from hs ticks to radio timer ticks
#define RADIO_TICKS_TO_MS(t)                      ((t) / (RADIO_TIMER_FREQUENCY / 1000))
#define RADIO_MS_TO_TICKS(ms)                     ((ms) * (RADIO_TIMER_FREQUENCY / 1000))


typedef enum
{
  LORA_SYNCWORD_PUBLIC = LORA_MAC_PUBLIC_SYNCWORD,
  LORA_SYNCWORD_PRIVATE = LORA_MAC_PRIVATE_SYNCWORD,
  LORA_SYNCWORD_PERMASENSE = (LORA_MAC_PUBLIC_SYNCWORD ^ 0xdada)
} radio_lora_syncword_t;

typedef struct
{
  RadioModems_t modem;
  uint32_t bandwidth;
  uint32_t datarate;
  uint8_t coderate;
  uint8_t preambleLen; // LoRa: in symbols; FSK: in Bytes (not bits!)
  uint32_t fdev;
} radio_config_t;

typedef struct
{
  uint32_t centerFrequency;
  uint32_t bandwidth;
  uint8_t dutyCycle; // in 0.1 %
  int8_t maxPower;
} radio_band_t;

/**
 * Identifies a group of bands which can be merged (for higher throughput)
 */
typedef struct
{
  // Indices
  uint8_t lower;
  uint8_t upper;
} radio_band_group_t;

typedef struct lora_cad_params_s
{
  // Indices
  RadioLoRaCadSymbols_t symb_num;
  uint8_t cad_det_peak;
  uint8_t cad_det_min;
} radio_cad_params_t;

extern const radio_config_t radio_modulations[];
extern const radio_band_t radio_bands[];
extern const radio_band_group_t radio_band_groups[];
extern const radio_cad_params_t radio_cad_params[];

#endif /* RADIO_RADIO_CONSTANTS_H_ */
