/*
 * lptimer.h
 *
 *  Created on: 16.08.2019
 *      Author: rdaforno
 */

#ifndef TIME_LPTIMER_H_
#define TIME_LPTIMER_H_


#define LPTIMER_SECOND      32768       /* ticks per second */

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
 * Clear the timer value.
 */
void lptimer_clear(void);


#endif /* TIME_LPTIMER_H_ */
