/*
 * dozer_utils.c
 *
 *  Created on: Jul 9, 2018
 *      Author: kelmicha
 */

#include "flora_lib.h"

#if DOZER_ENABLE

/*
 * returns the absolute value of a 16 bit int as an uint16_t
 */
uint16_t abs16(int16_t value) {
  uint16_t ret;

  if ((value & 0x8000) == 0){  // the first bit is 0 --> it's a positive number
      ret = value;
  } else {
      ret = ~value;   // bitwise invert
      ret++;
  }

  return ret;
}


/*
 * returns the absolute value of a 32 bit int as an uint32_t
 */
uint32_t abs32(int32_t value) {
  uint32_t ret;

  if((value & 0x80000000) == 0) {  // the first bit is 0 --> it's a positive number
    ret = value;
  }
  else {
    ret = ~value;   // bitwise invert
    ret++;
  }

  return ret;
}


/*
 * get next random value with the specified seed
 */
uint32_t next_rand(uint32_t seed) {
  srand(seed);
  return (uint32_t) rand();
}



/*
 * serial print the buffer if the priority is high enough
 */
void dozer_print(uint8_t prio, char* buf) {
  if(prio >= 6) {
    cli_println(buf);
  }
}

/*
 * print the payload of the given message, if the given priority is high enough
 * supports beacons, hanshake and data messages
 * for the data messages, the payload is not printed as it is currently just random
 */
void print_msg(dozer_message_t* msg, uint8_t prio) {
  dozer_header_t header = msg->header;



  dozer_print(prio, "*****");

  sprintf(char_buff, "Source:\t%d", header.source);
  dozer_print(prio, char_buff);
  sprintf(char_buff, "Dest:\t%d", header.dest);
  dozer_print(prio, char_buff);

  switch (header.type) {
    case BEACON_MSG:

      dozer_print(prio, "Beacon:");

      beacon_msg_t bm = msg->payload.beacon_msg;

      sprintf(char_buff, "seed:\t\t%lu", bm.seed);
      dozer_print(prio, char_buff);
      sprintf(char_buff, "hop_count:\t%d", bm.hop_count);
      dozer_print(prio, char_buff);
      sprintf(char_buff, "load:\t\t%d", bm.load);
      dozer_print(prio, char_buff);
      sprintf(char_buff, "lsd:\t\t%d", bm.lsd);
      dozer_print(prio, char_buff);

      break;

    case HANDSHAKE_MSG:

      dozer_print(prio, "Handshake:");

      handshake_msg_t hs = msg->payload.handshake_msg;
      sprintf(char_buff, "slot:\t\t%d", hs.slot);
      dozer_print(prio, char_buff);

      break;

    case DATA_MSG:

      dozer_print(prio, "Data:");

      data_msg_t dm = msg->payload.data_msg;
      sprintf(char_buff, "seq nr:\t\t%d", dm.seqNr);
      dozer_print(prio, char_buff);
      sprintf(char_buff, "orig:\t\t%d", dm.originatorID);
      dozer_print(prio, char_buff);

      break;

    default:
      sprintf(char_buff, "Message type does not match! %d", header.type);
      dozer_print(prio, char_buff);
      break;

  }
  dozer_print(prio, "*****");
}

/*
 * serial print the current timestamp
 */
void print_ts(uint8_t prio) {
  uint64_t timestamp = hs_timer_get_current_timestamp();

  sprintf(char_buff, "Time:\t%llu", timestamp);
  dozer_print(prio, char_buff);
}

#endif /* DOZER_ENABLE */
