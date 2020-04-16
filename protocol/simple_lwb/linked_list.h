/*
 * linked_list.h
 *
 *  Created on: Mar 13, 2019
 *      Author: kelmicha
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
