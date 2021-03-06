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


#if BOLT_ENABLE

#define BOLT_ACK_STATUS                 (PIN_GET(BOLT_ACK) != 0)
#define BOLT_REQ_STATUS                 (PIN_STATE(BOLT_REQ) != 0)

static uint32_t bolt_write_cnt        = 0;
static uint32_t bolt_write_failed_cnt = 0;


bool bolt_init(void)
{
  /* note: control signals and SPI must be initialized before calling bolt_init! */
  if (!bolt_status()) {
    /* wait 50ms, then try again (maybe BOLT is still starting up) */
    LOG_VERBOSE("not ready, will try again in 50ms...");
    delay_us(50000);
    if (!bolt_status()) {
      LOG_ERROR("not accessible, init failed");
      return false;
    }
  }
  LOG_VERBOSE("initialized");

  return true;
}


void bolt_release(void)
{
  uint32_t count = 100;
  PIN_CLR(BOLT_REQ);                  /* set REQ low */
  while (BOLT_ACK_STATUS && count) {  /* wait until ACK is low or timeout */
    delay_us(10);
    count--;
  }
  if (count == 0) {
    LOG_ERROR("release timeout");
  }
}


bool bolt_acquire(bool mode_write)
{
  /* check status */
  if (BOLT_ACK_STATUS || BOLT_REQ_STATUS) {
    LOG_ERROR("request failed (REQ or ACK still high)");
    return false;
  }
  /* set MODE */
  if (!mode_write) {
    if (!BOLT_DATA_AVAILABLE) {
      LOG_WARNING("no data available");
      return false;
    }
    PIN_CLR(BOLT_MODE); /* 0 = READ */
  } else {
    PIN_SET(BOLT_MODE); /* 1 = WRITE */
  }

  /* set REQ high */
  PIN_SET(BOLT_REQ);
  /* wait for a rising edge on the ACK line */
  uint8_t cnt = 10;
  while (!BOLT_ACK_STATUS && cnt) {
    delay_us(10);
    cnt--;
  }
  if (!BOLT_ACK_STATUS) {
    /* ACK line is still low -> failed */
    PIN_CLR(BOLT_REQ);
    return false;
  }

  return true;
}


uint32_t bolt_read(uint8_t* out_data)
{
  uint32_t rcvd_bytes = 0;

  /* parameter check */
  if (!out_data) {
    return 0;
  }
  if (!bolt_acquire(false)) {
    return 0;
  }

  while ((rcvd_bytes < BOLT_MAX_MSG_LEN) && BOLT_ACK_STATUS) {
    /* read one byte at a time */
    if (BOLT_SPI_READ(out_data, 1) != 0) {
      LOG_ERROR("SPI read failed");
      break;
    }
    out_data++;
    rcvd_bytes++;
  }
  delay_us(10);   /* a small delay may be needed in case the message is as long as the max. supported size */
  if (BOLT_ACK_STATUS) {
    /* ACK is still high -> packet is too long */
    LOG_WARNING("received packet is too long");
    rcvd_bytes = 0;   /* error condition */
  }

  bolt_release();

  return rcvd_bytes;
}


bool bolt_write(uint8_t* data, uint16_t len)
{
  /* parameter check */
  if (!data || !len || len > BOLT_MAX_MSG_LEN) {
    return false;
  }
#if BOLT_WAKE_BASEBOARD_THRESHOLD
  /* if IND_OUT is low, reset the write counter */
  if (!BOLT_DATA_IN_OUTPUT_QUEUE) {
    bolt_write_cnt = 0;
  }
#endif /* BOLT_WAKE_BASEBOARD_THRESHOLD */

  if (!bolt_acquire(true)) {
    bolt_write_failed_cnt++;
    return false;
  }
  if (BOLT_SPI_WRITE(data, len) != 0) {
    LOG_ERROR("SPI write failed");
    bolt_release();
    return false;
  }
  bolt_release();

  bolt_write_cnt++;

#if BOLT_WAKE_BASEBOARD_THRESHOLD
  /* if the number of written messages reaches the configured threshold, then make sure the baseboard is woken / enabled */
  if ((bolt_write_cnt >= BOLT_WAKE_BASEBOARD_THRESHOLD) && !BASEBOARD_IS_ENABLED()) {
    BASEBOARD_ENABLE();
    LOG_INFO("baseboard woken (BOLT queue threshold exceeded)");
  }
#endif /* BOLT_WAKE_BASEBOARD_THRESHOLD */

  return true;
}


bool bolt_status(void)
{
  if (bolt_acquire(true)) {
    bolt_release();
    return true;
  }
  /* check if data in output queue */
  if (BOLT_DATA_IN_OUTPUT_QUEUE) {
    LOG_WARNING("can't verify status, output queue is probably full");
    return true;
  }
  return false;
}


bool bolt_full(void)
{
  if (bolt_acquire(true)) {
    bolt_release();
    return false;
  }
  return true;
}


void bolt_flush(void)
{
  uint32_t cnt = 0;
  while (BOLT_DATA_AVAILABLE) {
    if (!bolt_acquire(false)) {
      break;
    }
    uint8_t read_byte;
    while (BOLT_ACK_STATUS) {
      if (BOLT_SPI_READ(&read_byte, 1) != 0) {
        LOG_ERROR("SPI read failed");
        break;
      }
    }
    bolt_release();
    cnt++;
  }
  LOG_VERBOSE("queue cleared (%lu messages removed)", cnt);
}


uint32_t bolt_get_write_cnt(bool reset)
{
  uint32_t cnt = bolt_write_cnt;
  if (reset) {
    bolt_write_cnt = 0;
  }
  return cnt;
}


uint32_t bolt_get_write_failed_cnt(bool reset)
{
  uint32_t cnt = bolt_write_failed_cnt;
  if (reset) {
    bolt_write_failed_cnt = 0;
  }
  return cnt;
}

#endif /* BOLT_CONF_ON */
