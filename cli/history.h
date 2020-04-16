/*
 * history.h
 *
 *  Created on: 30.04.2018
 *      Author: marku
 */

#ifndef CLI_HISTORY_H_
#define CLI_HISTORY_H_


#define HISTORY_COUNT 4


typedef struct history_entry_s {
  char input[CLI_MAX_INPUT+1];
} history_entry_t;

typedef struct history_state_s {
  uint8_t count;
  uint8_t newest;
  uint8_t selected;
} history_state_t;

void history_init();

void history_insert(char* buffer, uint16_t length);
uint16_t history_get(char* buffer, uint16_t length, uint8_t index);
uint16_t history_get_previous(char* buffer, uint16_t length);
uint16_t history_get_next(char* buffer, uint16_t length);



#endif /* CLI_HISTORY_H_ */
