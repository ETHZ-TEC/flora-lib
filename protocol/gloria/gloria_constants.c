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

#include "flora_lib.h"

const uint8_t gloria_modulations[]             = { 3,  5,  7, 9 };
const int8_t  gloria_powers[]                  = { 0, 10, 22 };                                 // dBm
const uint8_t gloria_default_power_levels[]    = { 0,  0,  0,  0,  0,  0,  0,  0,  2,  2 };     // see radio_powers
const uint8_t gloria_default_retransmissions[] = { 3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3 };
const uint8_t gloria_default_acks[]            = { 3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3 };
const uint8_t gloria_default_data_slots[]      = { 4,  4,  4,  4,  8,  8, 12, 12, 16, 16, 16 };

const gloria_timings_t gloria_timings[] = {
    { // 0 (SF12)
        .slotOverhead       = 4179556,  // 522.444 ms
        .slotAckOverhead    = 4965988,  // 620.749 ms
        .floodInitOverhead  = 795938,   // 99.492 ms
        .rxOffset           = 786432,   // 98.304 ms
        .txSync             = 6078848,  // 759.856 ms (updated 20200812)
    },
    { // 1 (SF11)
        .slotOverhead       = 2132075,  // 266.509 ms
        .slotAckOverhead    = 2525291,  // 315.661 ms
        .floodInitOverhead  = 402722,   // 50.340 ms
        .rxOffset           = 393216,   // 49.152 ms
        .txSync             = 3032500,  // 379.063 ms (updated 20200812)
    },
    { // 2 (SF10)
        .slotOverhead       = 1050062,  // 131.258 ms
        .slotAckOverhead    = 1246670,  // 155.834 ms
        .floodInitOverhead  = 206114,   // 25.764 ms
        .rxOffset           = 196608,   // 24.576 ms
        .txSync             = 1513412,  // 189.177 ms (updated 20200812)
    },
    { // 3 (SF9)
        .slotOverhead       = 537151,   // 67.144 ms
        .slotAckOverhead    = 635455,   // 79.432 ms
        .floodInitOverhead  = 107810,   // 13.476 ms
        .rxOffset           = 98304,    // 12.288 ms
        .txSync             = 755928,   // 94.491 ms (updated 20200812)
    },
    { // 4 (SF8)
        .slotOverhead       = 283375,   // 35.422 ms
        .slotAckOverhead    = 332527,   // 41.566 ms
        .floodInitOverhead  = 58658,    // 7.332 ms
        .rxOffset           = 49152,    // 6.144 ms
        .txSync             = 378216,   // 47.277 ms (updated 20200812)
    },
    { // 5 (SF7)
        .slotOverhead       = 153639,   // 19.205 ms
        .slotAckOverhead    = 178215,   // 22.277 ms
        .floodInitOverhead  = 34082,    // 4.260 ms
        .rxOffset           = 24576,    // 3.072 ms
        .txSync             = 189864,   // 23.733 ms (updated 20200812)
    },
    { // 6 (SF6)
        .slotOverhead       = 79827,    // 9.978 ms
        .slotAckOverhead    = 102827,   // 12.853 ms
        .floodInitOverhead  = 32507,    // 4.063 ms
        .rxOffset           = 23001,    // 2.875 ms
        .txSync             = 112356,   // 14.045 ms (updated 20200812)
    },
    { // 7 (SF5)
        .slotOverhead       = 50171,    // 6.271 ms
        .slotAckOverhead    = 61672,    // 7.709 ms
        .floodInitOverhead  = 21007,    // 2.626 ms
        .rxOffset           = 11500,    // 1.438 ms
        .txSync             = 57316,    // 7.165 ms (updated 20200812)
    },
    { // 8 (FSK 125k)
        .slotOverhead       = 28000,    // 3.5 ms
        .slotAckOverhead    = 28000,    // 3.5 ms
        .floodInitOverhead  = 18000,    // 2.25 ms
        .rxOffset           = 4096,     // 512.000 us
        .txSync             = 4137,     // 517.125 us (updated 20200812)
    },
    { // 9 (FSK 200k)
        .slotOverhead       = 26400,    // 3.3 ms
        .slotAckOverhead    = 26400,    // 3.3 ms
        .floodInitOverhead  = 18000,    // 2.25 ms
        .rxOffset           = 2560,     // 320.000 us
        // .txSync          = 3206,     // 400.75 us
        .txSync             = 3034,     // 379.25 us (updated 20200812)
    },
    { // 10 (FSK 250k)
        .slotOverhead       = 14000,    // 1.75 ms
        .slotAckOverhead    = 14000,    // 1.75 ms
        .floodInitOverhead  = 18000,    // 2.25 ms
        .rxOffset           = 2560,     // 320.000 us (copy from 9)
        .txSync             = 3140,     // 392.5 us (updated 20200814)
    },
};
