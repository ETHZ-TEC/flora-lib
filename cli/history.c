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

#if CLI_ENABLE

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

#endif /* CLI_ENABLE */
