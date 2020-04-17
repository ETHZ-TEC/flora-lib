/*
 * bolt.c
 *
 *  Created on: Aug 22, 2018
 *      Author: rdaforno
 */

#include "flora_lib.h"

#if BOLT_ENABLE

/* private typedefs */
typedef enum {
  BOLT_STATE_IDLE = 0,
  BOLT_STATE_READ,
  BOLT_STATE_WRITE,
  BOLT_STATE_INVALID,
  NUM_OF_STATES
} bolt_state_t;

typedef enum {
  BOLT_OP_READ = 0,
  BOLT_OP_WRITE,
  NUM_OF_OPS
} bolt_op_mode_t;

/* private helper macros */
#define BOLT_ACK_STATUS                 PIN_GET(BOLT_ACK)
#define BOLT_REQ_STATUS                 PIN_GET(BOLT_REQ)
#define BOLT_WAIT_TILL_COMPLETED        while (BOLT_STATE_IDLE != bolt_state)

#if BOLT_DEBUG_ON
#define BOLT_DEBUG(...)                 DEBUG_PRINT(...)
#else
#define BOLT_DEBUG(...)
#endif /* BOLT_CONF_DEBUG_ON */

/* private variables */
static volatile bolt_state_t bolt_state = BOLT_STATE_INVALID;


bool bolt_init(void)
{
  /* note: control signals and SPI must be initialized before calling bolt_init! */

  if (bolt_status()) {
    BOLT_DEBUG("[BOLT] not accessible, init failed");
    return 0;
  }
  BOLT_DEBUG("[BOLT] initialized");
  bolt_state = BOLT_STATE_IDLE;

  return 1;
}


void bolt_release(void)
{
  PIN_CLR(BOLT_REQ);
  while (PIN_GET(BOLT_ACK));
  bolt_state = BOLT_STATE_IDLE;
  BOLT_DEBUG("[BOLT] back in idle state");
}

uint8_t bolt_acquire(bolt_op_mode_t mode)
{
  if (BOLT_STATE_INVALID == bolt_state) {
    BOLT_DEBUG("[BOLT] not initialized!");
    return 0;
  }
  if (BOLT_ACK_STATUS || BOLT_REQ_STATUS) {
    BOLT_DEBUG("[BOLT] request failed (REQ or ACK still high)");
    return 0;
  }
  if (BOLT_STATE_IDLE != bolt_state) {
    BOLT_DEBUG("[BOLT] not in idle state, operation skipped");
    return 0;
  }

  if (BOLT_OP_READ == mode) {
    if (!BOLT_DATA_AVAILABLE) {
      BOLT_DEBUG("[BOLT] no data available");
      return 0;
    }
    PIN_CLR(BOLT_MODE); /* 0 = READ */
    BOLT_DEBUG("[BOLT] requesting read access");

  } else {
    PIN_SET(BOLT_MODE); /* 1 = WRITE */
    BOLT_DEBUG("[BOLT] requesting write access");
  }

  PIN_SET(BOLT_REQ);
  /* now wait for a rising edge on the ACK line */
  uint8_t cnt = 5;
  while (!BOLT_ACK_STATUS && cnt) {
    delay_us(10);
    cnt--;
  }
  if (!BOLT_ACK_STATUS) {
    /* ACK line is still low -> failed */
    bolt_state = BOLT_STATE_IDLE;
    PIN_CLR(BOLT_REQ);
    BOLT_DEBUG("[BOLT] access denied");
    return 0;
  }

  /* update state */
  bolt_state = (mode == BOLT_OP_READ) ? BOLT_STATE_READ : BOLT_STATE_WRITE;

  return 1;
}

uint32_t bolt_read(uint8_t* out_data)
{
  uint32_t rcvd_bytes = 0;

  /* parameter check */
  if (!out_data) {
    BOLT_DEBUG("[BOLT] invalid parameter");
    return 0;
  }
  if (!bolt_acquire(BOLT_OP_READ)) {
    return 0;
  }
  BOLT_DEBUG("[BOLT] starting data transfer");

  while ((rcvd_bytes < BOLT_MAX_MSG_LEN) && BOLT_ACK_STATUS) {
    /* read one byte at a time */
    BOLT_SPI_READ(out_data, 1);
    out_data++;
    rcvd_bytes++;
  }
  if (BOLT_ACK_STATUS) {
    /* ACK is still high -> packet is too long */
    BOLT_DEBUG("[BOLT] received packet is too long");
    rcvd_bytes = 0;   /* error condition */
  }
  BOLT_DEBUG("[BOLT] %d bytes received", rcvd_bytes);

  bolt_release();

  return rcvd_bytes;
}

bool bolt_write(uint8_t* data, uint32_t len)
{
  /* parameter check */
  if (!data || !len || len > BOLT_MAX_MSG_LEN) {
    BOLT_DEBUG("[BOLT] invalid parameter");
    return false;
  }
  if (!bolt_acquire(BOLT_OP_WRITE)) {
    return false;
  }
  BOLT_DEBUG("[BOLT] starting data transfer");
  BOLT_SPI_WRITE(data, len);
  bolt_release();

  return true;
}

bool bolt_status(void)
{
  if (bolt_acquire(BOLT_OP_WRITE)) {
    bolt_release();
    return 1;
  }
  return 0;
}

void bolt_flush(void)
{
  uint8_t buffer[BOLT_MAX_MSG_LEN];
  while (BOLT_DATA_AVAILABLE && bolt_read(buffer));
}

#endif /* BOLT_CONF_ON */