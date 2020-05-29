/*
 * dcstat.c
 *
 *  Created on: May 29, 2020
 *      Author: rdaforno
 */

#include "flora_lib.h"


void dcstat_on(dcstat_t* dc)
{
  if (dc) {
    /* only overwrite if not already started */
    if (!dc->start_time) {
      dc->start_time = lptimer_now();
    }
  }
}


void dcstat_off(dcstat_t* dc)
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


uint32_t dcstat_get_dc(dcstat_t* dc)
{
  if (dc) {
    uint64_t delta = lptimer_now() - dc->reset_time;
    if (delta) {
      return dc->active_time * 1000000 / delta;
    }
  }
  return 1000000;
}
