/* THIS FILE HAS BEEN AUTOGENERATED BY FLORA-TOOLS */

#include "flora_lib.h"

const uint8_t gloria_modulations[] = {3,5,7,9};
const int8_t gloria_powers[] = {0,10,22}; // dBm
const uint8_t gloria_default_power_levels[] = {0,0,0,0,0,0,0,0,2,2}; // see radio_powers
const uint8_t gloria_default_retransmissions[] = {3,3,3,3,3,3,3,3,3,3,3};
const uint8_t gloria_default_acks[] = {3,3,3,3,3,3,3,3,3,3,3};
const uint8_t gloria_default_data_slots[] = {4,4,4,4,8,8,12,12,16,16,16};

const gloria_timings_t gloria_timings[] = {
    { // 0 (SF12)
        .slotOverhead = 4179556, // 522.444 ms
        .slotAckOverhead = 4965988, // 620.749 ms
        .floodInitOverhead = 795938, // 99.492 ms
        .rxOffset = 786432, // 98.304 ms
        .txSync = 6079087, // 759.886 ms (adapted by kelmicha)
    },
    { // 1 (SF11)
        .slotOverhead = 2132075, // 266.509 ms
        .slotAckOverhead = 2525291, // 315.661 ms
        .floodInitOverhead = 402722, // 50.340 ms
        .rxOffset = 393216, // 49.152 ms
        .txSync = 3032703, // 379.088 ms (adapted by kelmicha)
    },
    { // 2 (SF10)
        .slotOverhead = 1050062, // 131.258 ms
        .slotAckOverhead = 1246670, // 155.834 ms
        .floodInitOverhead = 206114, // 25.764 ms
        .rxOffset = 196608, // 24.576 ms
        .txSync = 1513613, // 189.202 ms (adapted by kelmicha)
    },
    { // 3 (SF9)
        .slotOverhead = 537151, // 67.144 ms
        .slotAckOverhead = 635455, // 79.432 ms
        .floodInitOverhead = 107810, // 13.476 ms
        .rxOffset = 98304, // 12.288 ms
        .txSync = 756071, // 94.509 ms (adapted by kelmicha)
    },
    { // 4 (SF8)
        .slotOverhead = 283375, // 35.422 ms
        .slotAckOverhead = 332527, // 41.566 ms
        .floodInitOverhead = 58658, // 7.332 ms
        .rxOffset = 49152, // 6.144 ms
        .txSync = 378255, // 47.282 ms
    },
    { // 5 (SF7)
        .slotOverhead = 153639, // 19.205 ms
        .slotAckOverhead = 178215, // 22.277 ms
        .floodInitOverhead = 34082, // 4.260 ms
        .rxOffset = 24576, // 3.072 ms
        .txSync = 189986, // 23.748 ms (adapted by kelmicha)
    },
    { // 6 (SF6)
        .slotOverhead = 79827, // 9.978 ms
        .slotAckOverhead = 102827, // 12.853 ms
        .floodInitOverhead = 32507, // 4.063 ms
        .rxOffset = 23001, // 2.875 ms
        .txSync = 112533, // 14.066 ms (adapted by kelmicha)
    },
    { // 7 (SF5)
        .slotOverhead = 50171, // 6.271 ms
        .slotAckOverhead = 61672, // 7.709 ms
        .floodInitOverhead = 21007, // 2.626 ms
        .rxOffset = 11500, // 1.438 ms
        .txSync = 57493, // 7.186 ms (adapted by kelmicha)
    },
    { // 8 (FSK 125k)
        .slotOverhead = 28000, // 3.5 ms (adapted by kelmicha)
        .slotAckOverhead = 28000, // 3.5 ms (adapted by kelmicha)
        .floodInitOverhead = 14263, // 1.783 ms
        .rxOffset = 4096, // 512.000 us
        .txSync = 4309, // 538.625 us (adapted by kelmicha)
    },
    { // 9 (FSK 200k)
        .slotOverhead = 26400, // 3.3 ms (adapted by kelmicha)
        .slotAckOverhead = 26400, // 3.3 ms (adapted by kelmicha)
        .floodInitOverhead = 14263, // 1.783 ms
        .rxOffset = 2560, // 320.000 us
        .txSync = 3206, // 400.75 us (adapted by kelmicha)
    },
    { // 10 (FSK 250k)
        .slotOverhead = 26400, // 3.3 ms (copy from 9)
        .slotAckOverhead = 26400, // 3.3 ms (copy from 9)
        .floodInitOverhead = 14263, // 2.125 ms
        .rxOffset = 2560, // 320.000 us (copy from 9)
        .txSync = 3206, // 400.75 us (copy from 9)
    },
};
