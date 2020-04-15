/*
 * history.c
 *
 *  Created on: 30.04.2018
 *      Author: marku
 */

#include "history.h"

#include <string.h>

static history_entry_t history[HISTORY_COUNT] = {0};
static history_state_t state = { .count = 1, .newest = 0, .selected = 0};

void history_init() {
  state.newest = 0;
}

void history_insert(char* buffer, uint16_t length) {
  buffer[length] = '\0';

  if (strcmp(buffer, history[((unsigned) state.newest + 1) % HISTORY_COUNT].input) != 0) {

    strcpy(history[state.newest].input, buffer);

    if (state.newest == 0)
    {
      state.newest = HISTORY_COUNT - 1;
    }
    else {
      state.newest--;
    }

    state.selected = 0;

    if(state.count < HISTORY_COUNT) {
      state.count++;
    }
  }
  else {
    history[state.newest].input[0] = '\0';
    state.selected = 0;
  }
}

uint16_t history_get(char* buffer, uint16_t length, uint8_t index) {
  uint8_t actual_index = ((unsigned) state.newest + index) % HISTORY_COUNT;

  if(length != 0) {
    buffer[length] = '\0';
    strcpy(history[((unsigned) state.newest + (unsigned) state.selected) % HISTORY_COUNT].input, buffer);
  }

  if(index >= 0 && index < state.count)
  {
    state.selected = index;

    strcpy(buffer, history[actual_index].input);
    return strlen(history[actual_index].input);
  }
  else
  {
    return strlen(history[actual_index].input);
  }
}


uint16_t history_get_previous(char* buffer, uint16_t length) {
  if (state.selected < (state.count - 1)) {
    return history_get(buffer, length, state.selected + 1);
  } else {
    return history_get(buffer, length, state.selected);
  }

}

uint16_t history_get_next(char* buffer, uint16_t length) {
  if (state.selected > 0) {
    return history_get(buffer, length, state.selected - 1);
  } else {
    return history_get(buffer, length, state.selected);
  }
}





