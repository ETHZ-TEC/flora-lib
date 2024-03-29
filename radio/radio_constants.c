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

#include "flora_lib.h"

#if RADIO_ENABLE

const radio_config_t radio_modulations[RADIO_NUM_MODULATIONS] =
{
    // NOTE: bandwidth is the DSB
    {   // 0
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 12,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 1
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 11,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 2
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 10,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 3
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 9,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 4
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 8,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 5
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 7,
        .coderate = 1,
        .preambleLen = 10,
    },
    {   // 6
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 6,
        .coderate = 1,
        .preambleLen = 12,
    },
    {   // 7
        .modem = MODEM_LORA,
        .bandwidth = 0,
        .datarate = 5,
        .coderate = 1,
        .preambleLen = 12,
    },
    {   // 8
        .modem = MODEM_FSK,
        .bandwidth = 234300,
        .datarate = 125000,
        .fdev = 50000,
        .preambleLen = 2,
    },
    {   // 9
        .modem = MODEM_FSK,
        .bandwidth = 234300,
        .datarate = 200000,
        .fdev = 10000,
        .preambleLen = 2,
    },
    {   // 10
        .modem = MODEM_FSK,
        .bandwidth = 312000,
        .datarate = 250000,
        .fdev = 23500,
        .preambleLen = 4,
    },
};

#ifndef US915
const radio_band_t radio_bands[RADIO_NUM_BANDS] =
{
    /* Notes:
     * - duty cycle is in 0.1% steps (e.g. 100 = 10%)
     * - general rule for non-specific SRD without any collision avoidance for the 863 - 870 MHz band: max. 0.1% duty cycle at 14dBm (25mW)
     */
    { .centerFrequency = 863062500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 0
    { .centerFrequency = 863187500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 1
    { .centerFrequency = 863312500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 2
    { .centerFrequency = 863437500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 3
    { .centerFrequency = 863562500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 4
    { .centerFrequency = 863687500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 5
    { .centerFrequency = 863812500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 6
    { .centerFrequency = 863937500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 7
    { .centerFrequency = 864062500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 8
    { .centerFrequency = 864187500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 9
    { .centerFrequency = 864312500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 10
    { .centerFrequency = 864437500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 11
    { .centerFrequency = 864562500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 12
    { .centerFrequency = 864687500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 13
    { .centerFrequency = 864812500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 14
    { .centerFrequency = 864937500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 15
    { .centerFrequency = 865062500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 16
    { .centerFrequency = 865187500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 17
    { .centerFrequency = 865312500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 18
    { .centerFrequency = 865437500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 19
    { .centerFrequency = 865562500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 20
    { .centerFrequency = 865687500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 21
    { .centerFrequency = 865812500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 22
    { .centerFrequency = 865937500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 23
    { .centerFrequency = 866062500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 24
    { .centerFrequency = 866187500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 25
    { .centerFrequency = 866312500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 26
    { .centerFrequency = 866437500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 27
    { .centerFrequency = 866562500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 28
    { .centerFrequency = 866687500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 29
    { .centerFrequency = 866812500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 30
    { .centerFrequency = 866937500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 31
    { .centerFrequency = 867062500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 32
    { .centerFrequency = 867187500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 33
    { .centerFrequency = 867312500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 34
    { .centerFrequency = 867437500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 35
    { .centerFrequency = 867562500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 36
    { .centerFrequency = 867687500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 37
    { .centerFrequency = 867812500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 38
    { .centerFrequency = 867937500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 39
    { .centerFrequency = 868062500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 40
    { .centerFrequency = 868187500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 41
    { .centerFrequency = 868312500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 42
    { .centerFrequency = 868437500, .bandwidth = 125000, .dutyCycle = 10, .maxPower = 14 }, // 43
    { .centerFrequency = 868762500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 44
    { .centerFrequency = 868887500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 45
    { .centerFrequency = 869012500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 46
    { .centerFrequency = 869137500, .bandwidth = 125000, .dutyCycle = 1, .maxPower = 14 }, // 47
    { .centerFrequency = 869462500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 48
    { .centerFrequency = 869587500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 49
    { .centerFrequency = 869762500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 7 }, // 50       up to 100% duty cycle @7dBm or 1% @14dBm
    { .centerFrequency = 869887500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 7 }, // 51       up to 100% duty cycle @7dBm or 1% @14dBm
};

const radio_band_group_t lora_band_groups[] = {
    { .lower = 0, .upper = 15 },
    { .lower = 16, .upper = 39 },
    { .lower = 40, .upper = 43 },
    { .lower = 44, .upper = 47 },
    { .lower = 48, .upper = 49 },
    { .lower = 50, .upper = 51 },
};

#else
const lora_band_t radio_bands[] =
{
    { .centerFrequency = 914812500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 0
    { .centerFrequency = 914937500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 1
    { .centerFrequency = 915062500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 2
    { .centerFrequency = 915187500, .bandwidth = 125000, .dutyCycle = 100, .maxPower = 26 }, // 3
};

const radio_band_group_t radio_band_groups[] = {
    { .lower = 0, .upper = 3 },
};

#endif

// CAD with 1 Symbol is totally random!
const radio_cad_params_t radio_cad_params[RADIO_NUM_CAD_PARAMS] = {
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 25, .cad_det_min = 10}, // SF12
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 24, .cad_det_min = 10}, // SF11
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 23, .cad_det_min = 10}, // SF10
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 22, .cad_det_min = 10}, // SF9
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 21, .cad_det_min = 10}, // SF8
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 20, .cad_det_min = 10}, // SF7
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 19, .cad_det_min = 10}, // SF6
    {.symb_num = LORA_CAD_04_SYMBOL, .cad_det_peak = 18, .cad_det_min = 10}, // SF5
};

#endif /* RADIO_ENABLE */
