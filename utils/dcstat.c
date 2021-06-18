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


void dcstat_start(dcstat_t* dc)
{
  if (dc) {
    /* only overwrite if not already started */
    if (!dc->start_time) {
      dc->start_time = lptimer_now();
    }
  }
}


void dcstat_stop(dcstat_t* dc)
{
  if (dc) {
    /* start time valid? */
    if (dc->start_time) {
      dc->active_time += (lptimer_now() - dc->start_time);
      dc->start_time   = 0;
    }
  }
}


void dcstat_reset(dcstat_t* dc)
{
  if (dc) {
    dc->start_time  = 0;
    dc->active_time = 0;
    dc->reset_time  = lptimer_now();
  }
}


uint32_t dcstat_get_dc(const dcstat_t* const dc)
{
  if (dc) {
    uint64_t delta  = lptimer_now() - dc->reset_time;
    uint64_t active = dc->active_time;
    if (dc->start_time) {
      active += (lptimer_now() - dc->start_time);
    }
    if (delta) {
      return active * 1000000 / delta;
    }
  }
  return 1000000;
}
