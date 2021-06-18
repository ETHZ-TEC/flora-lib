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

#ifndef PROTOCOL_SIMPLE_LWB_LINKED_LIST_H_
#define PROTOCOL_SIMPLE_LWB_LINKED_LIST_H_


typedef struct list_element_t list_element_t;

struct __attribute__((__packed__, __aligned__(1))) list_element_t {
  list_element_t* next_element;
  uint8_t data[];
};

typedef struct __attribute__((__packed__, __aligned__(1))) {
  uint8_t list_size;
  uint8_t data_size;
  list_element_t* head;
  list_element_t* tail;
} linked_list_t;


linked_list_t* ll_get_new_list(uint8_t data_size);

void* ll_find_element(linked_list_t* list, void* data);
uint8_t ll_free_element(linked_list_t* list, void* data);

void ll_append_element(linked_list_t* list, void* data);
void ll_prepend_element(linked_list_t* list, void* data);

void* ll_get_new_element_head(linked_list_t* list);
void* ll_get_new_element_tail(linked_list_t* list);

list_element_t* ll_get_head(linked_list_t* list);
list_element_t* ll_get_tail(linked_list_t* list);

void* ll_get_head_data(linked_list_t* list);
void* ll_get_tail_data(linked_list_t* list);

void ll_free_head(linked_list_t* list);
void ll_free_tail(linked_list_t* list);

#endif /* PROTOCOL_SIMPLE_LWB_LINKED_LIST_H_ */
