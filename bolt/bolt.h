/*
 * bolt.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef __DPP_LIB_BOLT_H__
#define __DPP_LIB_BOLT_H__


/* DEFAULT CONFIG */

#ifndef BOLT_ENABLE
#define BOLT_ENABLE                     0
#endif /* BOLT_ENABLE */

/* note: the time request feature is configured outside of this driver */

#ifndef BOLT_MAX_MSG_LEN
#define BOLT_MAX_MSG_LEN                128  /* bytes */
#endif /* BOLT_MAX_MSG_LEN */

/* interface to SPI HAL functions */
#ifndef BOLT_SPI_WRITE
#define BOLT_SPI_WRITE(data, len)       HAL_SPI_Transmit(&hspi1, data, len, pdMS_TO_TICKS(10))
#define BOLT_SPI_READ(data, len)        HAL_SPI_Receive(&hspi1, data, len, pdMS_TO_TICKS(10))
#endif /* BOLT_SPI_WRITE */

#define BOLT_DATA_AVAILABLE             (PIN_GET(BOLT_IND) > 0)
#define BOLT_DATA_IN_OUTPUT_QUEUE       (PIN_GET(APP_IND) > 0)


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


#endif /* __DPP_LIB_BOLT_H__ */
