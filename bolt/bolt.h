/*
 * bolt.h
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#ifndef BOLT_BOLT_H_
#define BOLT_BOLT_H_


/* DEFAULT CONFIG */

#ifndef BOLT_ENABLE
#define BOLT_ENABLE                     0
#endif /* BOLT_ENABLE */

/* note: the time request feature is configured outside of this driver */

#ifndef BOLT_MAX_MSG_LEN
#define BOLT_MAX_MSG_LEN                128  /* bytes */
#endif /* BOLT_MAX_MSG_LEN */

#if BASEBOARD
  /* baseboard-specific features */
  #ifndef BOLT_WAKE_BASEBOARD_THRESHOLD
  #define BOLT_WAKE_BASEBOARD_THRESHOLD 0   /* if set to a value != 0, the baseboard will be woken (enable pin high) if the #written messages exceeds this value */
  #endif /* BOLT_WAKE_BASEBOARD_THRESHOLD */
#else  /* BASEBOARD */
  #define BOLT_WAKE_BASEBOARD_THRESHOLD 0   /* force disable feature */
#endif /* BASEBOARD */

/* interface to SPI HAL functions */
#ifndef BOLT_SPI_WRITE
#define BOLT_SPI_WRITE(data, len)       HAL_SPI_Transmit(&hspi1, data, len, pdMS_TO_TICKS(10))
#define BOLT_SPI_READ(data, len)        HAL_SPI_Receive(&hspi1, data, len, pdMS_TO_TICKS(10))
#endif /* BOLT_SPI_WRITE */

#define BOLT_DATA_AVAILABLE             (PIN_GET(BOLT_IND) != 0)
#define BOLT_DATA_IN_OUTPUT_QUEUE       (PIN_GET(APP_IND) != 0)


bool bolt_init(void);

/*
 * returns true if BOLT is active/ready (= responds to a write request)
 */
bool bolt_status(void);

/*
 * return true if the BOLT queue is full (write access denied)
 */
bool bolt_full(void);

/*
 * flush the BOLT queue (drop all incoming messages)
 */
void bolt_flush(void);

/*
 * read a message from BOLT
 * @param   out_data: pointer to a buffer to hold the read data, must be at least BOLT_MAX_MSG_LEN bytes long
 * @return  the number of read bytes
 */
uint32_t bolt_read(uint8_t* out_data);

/*
 * write a message to BOLT
 * @param   data: pointer to a buffer containing the data to write
 * @param   len:  the number of bytes to write (max. length: BOLT_MAX_MSG_LEN)
 * @return  true if the operation was successful, false otherwise
 */
bool bolt_write(uint8_t* data, uint32_t len);

/*
 * get the write counter
 * @param   reset: set to true to reset the counter
 * @return  the number of messages that have been successfully written to BOLT since the last reset;
 *          in case BOLT_WAKE_BASEBOARD_WRITE_TH is used, this value represents the number of written messages since the BOLT output queue was empty the last time (~= estimated # messages currently in the output queue)
 */
uint32_t bolt_get_write_cnt(bool reset);

/*
 * get the write-failed counter
 * @param   reset: set to true to reset the counter
 * @return  the number of messages that could not be written to BOLT (due to queue full) since the last reset
 */
uint32_t bolt_get_write_failed_cnt(bool reset);


#endif /* BOLT_BOLT_H_ */
