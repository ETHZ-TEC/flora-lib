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

#ifndef PROTOCOL_DISCOSYNC_DISCOSYNC_H_
#define PROTOCOL_DISCOSYNC_DISCOSYNC_H_


// Config

#ifndef DISCOSYNC_BAND
#define DISCOSYNC_BAND              16
#endif /* DISCOSYNC_BAND */

#ifndef DISCOSYNC_TX_PWR
#define DISCOSYNC_TX_PWR            14
#endif /* DISCOSYNC_TX_PWR */

#ifndef DISCOSYNC_MODULATION
#define DISCOSYNC_MODULATION        10
#endif /* DISCOSYNC_MODULATION */

#ifndef DISCOSYNC_TX_OFS
#define DISCOSYNC_TX_OFS            6000    // TX offset from the beginning of the slot (in hstimer ticks), must be larger than the expected RX to TX switching time
#endif /* DISCOSYNC_TX_OFS */

#ifndef DISCOSYNC_MAX_NUM_SLOTS
#define DISCOSYNC_MAX_NUM_SLOTS     16
#endif /* DISCOSYNC_MAX_NUM_SLOTS */

#ifndef DISCOSYNC_PROBABILISTIC_ASSIGNMENT
#define DISCOSYNC_PROBABILISTIC_ASSIGNMENT    0 // Configures Discosync to either use static (== 0) or probabilistic (==1) slot assignment
#endif /* DISCOSYNC_PROBABILISTIC_ASSIGNMENT */

#if DISCOSYNC_PROBABILISTIC_ASSIGNMENT
#ifndef DISCOSYNC_TX_PROBABILITY
#define DISCOSYNC_TX_PROBABILITY    0.6321
#endif /* DISCOSYNC_TX_PROBABILITY */
#endif /* DISCOSYNC_PROBABILISTIC_ASSIGNMENT */

#ifndef DISCOSYNC_MIN_RX_TIME
#define DISCOSYNC_MIN_RX_TIME       8000    // minimum time required to enter RX mode
#endif /* DISCOSYNC_MIN_RX_TIME */

#ifndef DISCOSYNC_FIRST_SLOT_TX
#define DISCOSYNC_FIRST_SLOT_TX     1       // enforce TX in the first slot
#endif /* DISCOSYNC_FIRST_SLOT_TX */

#ifndef DISCOSYNC_DISABLE_INTERRUPTS
#define DISCOSYNC_DISABLE_INTERRUPTS  1        // disable potentially interfering interrupts
#endif /* DISCOSYNC_DISABLE_INTERRUPTS */

#ifndef DISCOSYNC_MAX_RAND_TX_OFS
#define DISCOSYNC_MAX_RAND_TX_OFS   0       // if != 0, a random offset between 0 to the specified value in hs ticks will be subtracted from the transmission start
#endif /* DISCOSYNC_MAX_RAND_TX_OFS */

#if DISCOSYNC_MAX_RAND_TX_OFS > DISCOSYNC_TX_OFS
#error "invalid DISCOSYNC_MAX_RAND_TX_OFS"
#endif

#ifndef DISCOSYNC_TX_TRIGGER_DELAY
#define DISCOSYNC_TX_TRIGGER_DELAY  36      // TX trigger delay in hs timer ticks, implementation dependent
#endif /* DISCOSYNC_TX_TRIGGER_DELAY */


// Defines & macros

#define DISCOSYNC_PKT_LEN           2
#define DISCOSYNC_PKT_TYPE          0xb     // 4 random bits

#define DISCOSYNC_SLOT_TIME         (2 * DISCOSYNC_TX_OFS + radio_get_toa_hs(DISCOSYNC_PKT_LEN, DISCOSYNC_MODULATION) + gloria_timings[DISCOSYNC_MODULATION].txSync)  // Slot duration in hstimer ticks
#define DISCOSYNC_SLOT_TIME_MS      (DISCOSYNC_SLOT_TIME / HS_TIMER_FREQUENCY_MS)


// Typedefs

typedef void (* discosync_cb_t)(void);


// Functions

void     discosync_start(uint8_t nr_slots, uint16_t rx_time_before_start_ms, discosync_cb_t _callback);
void     discosync_stop(void);
uint64_t discosync_get_t_ref(void);       // returns the reference time (start of the disco sync round) in hs ticks
uint8_t  discosync_get_rx_cnt(void);


#endif /* PROTOCOL_DISCOSYNC_DISCOSYNC_H_ */
