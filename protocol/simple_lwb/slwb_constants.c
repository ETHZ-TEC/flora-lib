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

#if SLWB_ENABLE

const uint8_t slwb_max_flood_slots[] = {1, 1, 2, 2, 3, 3, 3, 3, 10, 10};
const uint8_t slwb_default_power_levels[] =  {0,0,0,0,0,0,0,0,22,22};
const slwb_slot_times_t slwb_slot_times[10] = {
  {    // 0
    .schedule_slot_time = 26936990, // 3367ms
    .contention_slot_time = 25939202, // 3242ms
    .ack_slot_time = 13829790, // 1729ms
    .data_slot_time = 19072670, // 2384ms
    .lr_schedule_slot_time = 15140510, // 1893ms
    .lr_cont_slot_time = 25939202, // 3242ms
    .lr_data_slot_time = 19072670, // 2384ms
  },
  {    // 1
    .schedule_slot_time = 14370981, // 1796ms
    .contention_slot_time = 13259024, // 1657ms
    .ack_slot_time = 7162021, // 895ms
    .data_slot_time = 9783461, // 1223ms
    .lr_schedule_slot_time = 7817381, // 977ms
    .lr_cont_slot_time = 13259024, // 1657ms
    .lr_data_slot_time = 9783461, // 1223ms
  },
  {    // 2
    .schedule_slot_time = 13487062, // 1686ms
    .contention_slot_time = 12998514, // 1625ms
    .ack_slot_time = 6933462, // 867ms
    .data_slot_time = 9554902, // 1194ms
    .lr_schedule_slot_time = 4097544, // 512ms
    .lr_cont_slot_time = 6802390, // 850ms
    .lr_data_slot_time = 5080584, // 635ms
  },
  {    // 3
    .schedule_slot_time = 7627960, // 953ms
    .contention_slot_time = 6752566, // 844ms
    .ack_slot_time = 3695800, // 462ms
    .data_slot_time = 5334200, // 667ms
    .lr_schedule_slot_time = 2429561, // 304ms
    .lr_cont_slot_time = 3630264, // 454ms
    .lr_data_slot_time = 2921081, // 365ms
  },
  {    // 4
    .schedule_slot_time = 6088967, // 761ms
    .contention_slot_time = 5476820, // 685ms
    .ack_slot_time = 2894087, // 362ms
    .data_slot_time = 4368647, // 546ms
    .lr_schedule_slot_time = 1434409, // 179ms
    .lr_cont_slot_time = 2131480, // 266ms
    .lr_data_slot_time = 1762089, // 220ms
  },
  {    // 5
    .schedule_slot_time = 3653807, // 457ms
    .contention_slot_time = 3137828, // 392ms
    .ack_slot_time = 1810607, // 226ms
    .data_slot_time = 2547887, // 318ms
    .lr_schedule_slot_time = 974945, // 122ms
    .lr_cont_slot_time = 1335432, // 167ms
    .lr_data_slot_time = 1138785, // 142ms
  },
  {    // 6
    .schedule_slot_time = 2284940, // 286ms
    .contention_slot_time = 1994381, // 249ms
    .ack_slot_time = 1179020, // 147ms
    .data_slot_time = 1609100, // 201ms
    .lr_schedule_slot_time = 742886, // 93ms
    .lr_cont_slot_time = 953233, // 119ms
    .lr_data_slot_time = 824806, // 103ms
  },
  {    // 7
    .schedule_slot_time = 1500952, // 188ms
    .contention_slot_time = 1294288, // 162ms
    .ack_slot_time = 855832, // 107ms
    .data_slot_time = 1101592, // 138ms
    .lr_schedule_slot_time = 596770, // 75ms
    .lr_cont_slot_time = 712202, // 89ms
    .lr_data_slot_time = 647970, // 81ms
  },
  {    // 8
    .schedule_slot_time = 950455, // 119ms
    .contention_slot_time = 984735, // 123ms
    .ack_slot_time = 684215, // 86ms
    .data_slot_time = 791735, // 99ms
    .lr_schedule_slot_time = 444979, // 56ms
    .lr_cont_slot_time = 471447, // 59ms
    .lr_data_slot_time = 452147, // 57ms
  },
  {    // 9
    .schedule_slot_time = 881615, // 110ms
    .contention_slot_time = 1009615, // 126ms
    .ack_slot_time = 715215, // 89ms
    .data_slot_time = 782415, // 98ms
    .lr_schedule_slot_time = 446735, // 56ms
    .lr_cont_slot_time = 473935, // 59ms
    .lr_data_slot_time = 451215, // 56ms
  },
};

#endif /* SLWB_ENABLE */
