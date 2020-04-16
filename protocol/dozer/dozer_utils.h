/*
 * dozer_utils.h
 *
 *  Created on: Jul 9, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_DOZER_UTILS_H_
#define PROTOCOL_DOZER_DOZER_UTILS_H_

// returns the absolute value of a 16 bit int as an uint16_t
uint16_t abs16(int16_t value);
uint32_t abs32(int32_t value);


uint32_t next_rand(uint32_t seed);

void dozer_print(uint8_t prio, char* buf);


void print_msg(dozer_message_t* msg, uint8_t prio);

void print_ts(uint8_t prio);



#endif /* PROTOCOL_DOZER_DOZER_UTILS_H_ */
