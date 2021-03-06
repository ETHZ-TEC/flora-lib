/*
 * Copyright (c) 2018 - 2022, ETH Zurich, Computer Engineering Group (TEC)
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

#ifndef PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_
#define PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_

#define GLORIA_PHY_MAX_PAYLOAD        255       // max length of the physical layer payload (given by the hardware)
#define GLORIA_HEADER_LENGTH          4         // length of the gloria header (without timestamp)
#define GLORIA_HEADER_LENGTH_MIN      2         // length of the gloria header in ACK mode (without timestamp)
#define GLORIA_MAX_PAYLOAD_LENGTH     (GLORIA_PHY_MAX_PAYLOAD - GLORIA_HEADER_LENGTH - GLORIA_TIMESTAMP_LENGTH)
#define GLORIA_ACK_LENGTH             2         // length of the gloria ack message
#define GLORIA_TIMESTAMP_LENGTH       8         // length of the timestamp to send with sync floods

#ifndef GLORIA_SCHEDULE_GRANULARITY
#define GLORIA_SCHEDULE_GRANULARITY   1         // sync timestamps get divided by the granularity. can be used to send fewer bytes for the sync ts.
#endif /* GLORIA_SCHEDULE_GRANULARITY */

#define GLORIA_RADIO_SLEEP_TIME       2000      // 250us  TODO: calculate min sleep time for which it's worth going into warm sleep mode
#define GLORIA_RADIO_SLEEP_TIME_COLD  240000000 // 30s    TODO: calculate min sleep time for which it's worth going into cold sleep mode
#define GLORIA_RADIO_WAKEUP_TIME      4180      // 522.500 us; time needed to wake up from warm sleep
#define GLORIA_RADIO_WAKEUP_TIME_COLD 36000     // 4.5 ms; time needed to wake up from cold sleep
#define GLORIA_MIN_RX_TIME            2000      // 250 us; min time left before rx timeout for which radio is still set in receive mode

#define GLORIA_HSTIMER_TRIGGER_DELAY  36        // ~4.5 us  implementation specific time between the timer compare trigger and the actual NSS pin actuation (assumes ~18 CPU cycles for ISR entry + register saving + ISR_IND setting)
#define GLORIA_TIME_BUFFER            800       // 100 us, time buffer for RX / TX radio setup
#define GLORIA_FLOOD_FINISH_OVERHEAD  152       // 19.000 us
#define GLORIA_RX_TRIGGER_DELAY       682       // 85.250 us   delay after the rx command has been sent to the radio until it is executed
#define GLORIA_TX_TRIGGER_DELAY       1010      // 126.250 us  delay after the tx command has been sent to the radio until it is executed
#define GLORIA_RX_SETUP               5326      // 665.750 us  time needed for the rx radio setup
#define GLORIA_TX_SETUP               10083     // 1.260 ms    time needed for the tx radio setup

typedef struct {
  uint32_t slotOverhead;
  uint32_t slotAckOverhead;
  uint32_t floodInitOverhead;
  uint32_t rxOffset;
  uint32_t txSync;
} gloria_timings_t;

extern const gloria_timings_t gloria_timings[];
extern const uint8_t          gloria_modulations[];
extern const int8_t           gloria_powers[];
extern const uint8_t          gloria_default_power_levels[];
extern const uint8_t          gloria_default_retransmissions[];
extern const uint8_t          gloria_default_acks[];
extern const uint8_t          gloria_default_data_slots[];

#endif /* PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_ */
