/*
 * bolt.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef BOLT_BOLT_H_
#define BOLT_BOLT_H_

/* include app config to override default parameters */
#include "main.h"

/* DEFAULT CONFIG */

#ifndef BOLT_ENABLE
#define BOLT_ENABLE                     0
#endif /* BOLT_ENABLE */

#ifndef BOLT_TRQ_ENABLE
#define BOLT_TRQ_ENABLE                 0
#endif /* BOLT_TRQ_ENABLE */

#ifndef BOLT_MAX_MSG_LEN
#define BOLT_MAX_MSG_LEN                128  /* bytes */
#endif /* BOLT_MAX_MSG_LEN */

#ifndef BOLT_DEBUG_ON
#define BOLT_DEBUG_ON                   0
#endif /* BOLT_DEBUG_ON */

/* interface to SPI HAL functions */
#ifndef BOLT_SPI_WRITE
#define BOLT_SPI_WRITE(data, len)       HAL_SPI_Transmit(&hspi1, data, len, 100)
#define BOLT_SPI_READ(data, len)        HAL_SPI_Receive(&hspi1, data, len, 100)
#endif /* BOLT_SPI_WRITE */

#define BOLT_DATA_AVAILABLE             (PIN_GET(BOLT_IND) > 0)


bool bolt_init(void);

/*
 * returns true if BOLT is active/ready (= responds to a write request)
 */
bool bolt_status(void);

/*
 * flush the BOLT queue (drop all incoming messages)
 */
void bolt_flush(void);

/*
 * read a message from BOLT (max. length: BOLT_MAX_MSG_LEN)
 */
uint32_t bolt_read(uint8_t* out_data);

/*
 * write a message to BOLT (max. length: BOLT_MAX_MSG_LEN)
 */
bool bolt_write(uint8_t* data, uint32_t len);


#if BOLT_TRQ_ENABLE
void    bolt_set_timereq_callback(void (*func)(void));
uint8_t bolt_handle_timereq(lptimer_clock_t* timestamp);
#endif /* BOLT_TRQ_ENABLE */


#endif /* BOLT_BOLT_H_ */
