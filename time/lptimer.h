/*
 * lptimer.h
 *
 *  Created on: 16.08.2019
 *      Author: rdaforno
 */

#ifndef TIME_LPTIMER_H_
#define TIME_LPTIMER_H_


#define LPTIMER_SECOND      32768UL       /* ticks per second */

#ifndef LPTIMER_RESET_WDG_ON_OVF
#define LPTIMER_RESET_WDG_ON_OVF  0     /* reset watchdog inside the timer overflow interrupt? (only works if watchdog configured and enabled) */
#endif /* LPTIMER_RESET_WDG_ON_OVF */

#ifndef LPTIMER_RESET_WDG_ON_EXP
#define LPTIMER_RESET_WDG_ON_EXP  0
#endif /* LPTIMER_RESET_WDG_ON_EXP */

#ifndef LPTIMER_CHECK_EXP_TIME
#define LPTIMER_CHECK_EXP_TIME    1     /* check expiration time inside lptimer_set() and issue a warning if it is in the past or far in the future */
#endif /* LPTIMER_CHECK_EXP_TIME */

/* returns relative time (elapsed time since MCU start) in milliseconds */
#define LPTIMER_NOW_MS()    (uint32_t)(lptimer_now() * 1000 / LPTIMER_SECOND)

/* returns relative time (elapsed time since MCU start) in seconds */
#define LPTIMER_NOW_SEC()   (uint32_t)(lptimer_now() / LPTIMER_SECOND)


typedef void (* lptimer_cb_func_t)(void);


/*
 * Set a timer, i.e. schedule the execution of a callback function.
 * @param timer     ID of the timer (of type lptimer_id_t)
 * @param t_exp     expiration time in ticks
 * @param cb        callback function that is execution once the timer expires
 */
void lptimer_set(uint64_t t_exp, lptimer_cb_func_t cb);

/*
 * Get the last (or current) expiration time of the lptimer.
 */
uint64_t lptimer_get(void);

/*
 * Returns the current timestamp in ticks.
 */
uint64_t lptimer_now(void);

/*
 * Get synchronized timestamps of both timers, lptimer and hs_timer.
 * Mustn't be called from an interrupt context.
 */
bool lptimer_now_synced(uint64_t* lp_timestamp, uint64_t* hs_timestamp);

/*
 * Clear the timer value.
 */
void lptimer_clear(void);


#endif /* TIME_LPTIMER_H_ */
