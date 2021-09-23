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

#ifndef BOLT_BOLT_H_
#define BOLT_BOLT_H_


/* DEFAULT CONFIG */

#ifndef BOLT_ENABLE
#define BOLT_ENABLE                     0
#endif /* BOLT_ENABLE */

#ifndef BOLT_MAX_MSG_LEN
#define BOLT_MAX_MSG_LEN                128  /* bytes */
#endif /* BOLT_MAX_MSG_LEN */

/* note: the time request feature is configured outside this driver */

#if BOLT_ENABLE

#if BOLT_MAX_MSG_LEN < DPP_MSG_PKT_LEN
#warning "BOLT_MAX_MSG_LEN is smaller than DPP_MSG_PKT_LEN"
#endif

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
#define BOLT_SPI_WRITE(data, len)       HAL_SPI_Transmit(&hspi1, data, len, pdMS_TO_TICKS(10))        /* SPI transmit function, must return 0 on success */
#define BOLT_SPI_READ(data, len)        HAL_SPI_Receive(&hspi1, data, len, pdMS_TO_TICKS(10))         /* SPI receive function, must return 0 on success */
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
bool bolt_write(uint8_t* data, uint16_t len);

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


#endif /* BOLT_ENABLE */


#endif /* BOLT_BOLT_H_ */
