/*
 * dcstat.h
 * duty-cycle stats
 *
 *  Created on: May 29, 2020
 *      Author: rdaforno
 */

#ifndef UTILS_DCSTAT_H_
#define UTILS_DCSTAT_H_


typedef struct
{
  uint64_t  reset_time;
  uint64_t  start_time;
  uint64_t  active_time;
} dcstat_t;


#define DCSTAT_ACTIVE_TIME_MS(dc)       (((dc)->active_time * 1000) / LPTIMER_SECOND)


void dcstat_start(dcstat_t* dc);        /* continue counting */
void dcstat_stop(dcstat_t* dc);         /* stop counting */
void dcstat_reset(dcstat_t* dc);        /* reset counter */
uint32_t dcstat_get_dc(dcstat_t* dc);   /* get duty cycle in ppm */


#endif /* UTILS_DCSTAT_H_ */
