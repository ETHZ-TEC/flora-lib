/*
 * linked_list.c
 *
 *  Created on: Mar 13, 2019
 *      Author: kelmicha
 */

#include "flora_lib.h"


/*
 * allocate memory for new list and initialize it
 * data_size: size in bytes of the data that should be stored in the list
 */
linked_list_t* ll_get_new_list(uint8_t data_size) {
  linked_list_t* list = malloc(sizeof(linked_list_t));
  list->list_size = 0;
  list->data_size = data_size;
  list->head = NULL;
  list->tail = NULL;
  return list;
}


/*
 * allocate memory for new element and append it at the end of the list
 * copy the given data to the element
 * data: the data to copy to the element
 */
void ll_append_element(linked_list_t* list, void* data) {
  list_element_t* new_element = malloc(sizeof(void*) + list->data_size);

  if (new_element) {
    memcpy(new_element->data, data, list->data_size);
    new_element->next_element = NULL;

    if (list->list_size) {
      list->tail->next_element = new_element;
      list->tail = new_element;
    }
    else {
      list->head = new_element;
      list->tail = new_element;
    }

    list->list_size++;
  }
}


/*
 * allocate memory for new element and prepend it at the beginning of the list
 * copy the given data to the element
 * data: the data to copy to the element
 */
void ll_prepend_element(linked_list_t* list, void* data) {
  list_element_t* new_element = malloc(sizeof(void*) + list->data_size);

  if (new_element) {
    memcpy(new_element->data, data, list->data_size);

    if (list->list_size) {
      new_element->next_element = list->head;
      list->head = new_element;
    }
    else {
      new_element->next_element = NULL;
      list->head = new_element;
      list->tail = new_element;
    }

    list->list_size++;
  }
}


/*
 * allocate memory for new element and append it to the end of the list
 * returns a pointer to the new elements data
 */
void* ll_get_new_element_tail(linked_list_t* list) {
  list_element_t* new_element = malloc(sizeof(void*) + list->data_size);

  if (new_element) {
    new_element->next_element = NULL;

    if (list->list_size) {
      list->tail->next_element = new_element;
      list->tail = new_element;
    }
    else {
      list->head = new_element;
      list->tail = new_element;
    }

    list->list_size++;
  }

  return new_element->data;
}


/*
 * allocate memory for new element and append it to the beginning of the list
 * returns a pointer to the new elements data
 */
void* ll_get_new_element_head(linked_list_t* list) {
  list_element_t* new_element = malloc(sizeof(void*) + list->data_size);

  if (new_element) {
    new_element->next_element = list->head;
    list->head = new_element;

    // set head to the new element if the list is empty
    if (!list->list_size) {
      list->tail = new_element;
    }

    list->list_size++;
  }

  return new_element->data;
}


/*
 * check if an element with data is in the list
 * return a pointer to the elements data if found
 * return NULL if element not found
 */
void* ll_find_element(linked_list_t* list, void* data) {
  list_element_t* elem = list->head;
  while (elem) {
    if (!memcmp(elem->data, data, list->data_size)) {
      return elem->data;
    }
    elem = elem->next_element;
  }
  return NULL;
}


/*
 * find element which corresponds to data in list and remove it from list
 * returns 1 if succesfull 0 otherwise
 */
uint8_t ll_free_element(linked_list_t* list, void* data) {
  if (list->list_size) {
    list_element_t* elem = list->head;

    // check first element
    if (!memcmp(elem->data, data, list->data_size)) {
      list->head = elem->next_element;
      free(elem);
      list->list_size--;
      return 1;
    }

    // iterate through the remaining elements
    while (elem->next_element) {
      if (!memcmp(elem->next_element->data, data, list->data_size)) {
        list_element_t* tmp = elem->next_element;
        elem->next_element = tmp->next_element;

        if (tmp == list->tail) {
          list->tail = elem;
        }

        free(tmp);
        list->list_size--;
        return 1;
      }
      elem = elem->next_element;
    }
  }

  return 0;
}

/*
 * get a pointer to the data of the first element in the list
 * returns NULL if the list is empty
 */
void* ll_get_head_data(linked_list_t* list) {
  if (list->list_size) {
    return list->head->data;
  }
  else {
    return NULL;
  }
}


/*
 * get a pointer to the first element in the list
 * returns NULL if the list is empty
 */
inline list_element_t* ll_get_head(linked_list_t* list) {
  return list->head;
}


/*
 * remove first element from list and free the allocated memory
 */
void ll_free_head(linked_list_t* list) {
  void* tmp = list->head;
  if (tmp) {
    list->head = list->head->next_element;
    list->list_size--;
    free(tmp);
  }
}


/*
 * get a pointer to the data of the last element in the list
 * returns NULL if the list is empty
 */
void* ll_get_tail_data(linked_list_t* list) {
  if (list->list_size) {
    return list->tail->data;
  }
  else {
    return NULL;
  }
}


/*
 * get a pointer to the last element in the list
 * returns NULL if the list is empty
 */
inline list_element_t* ll_get_tail(linked_list_t* list) {
  return list->tail;
}


/*
 * remove last element from list and free the allocated memory
 */
void ll_free_tail(linked_list_t* list) {
  if (list->list_size > 1) {
    void* tmp = list->tail;
    list_element_t* elem = list->head;
    while(elem->next_element != tmp) {
      elem = elem->next_element;
    }

    elem->next_element = NULL;
    list->tail = elem;
    list->list_size--;
    free(tmp);
  }
  else if (list->list_size == 1) {
    void* tmp = list->tail;
    list->head = NULL;
    list->tail = NULL;
    list->list_size--;
    free(tmp);
  }
}

