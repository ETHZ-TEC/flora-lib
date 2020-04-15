/*
 * timer_queue.c
 *
 *  Created on: Jul 17, 2018
 *      Author: kelmicha
 */
#include "timer_queue.h"
#include "dozer.h"

#ifdef DOZER

timer_queue_element_t* first_element = NULL;
timer_queue_element_t* current_timer = NULL;

static void (*tq_callback)() = NULL;

/*
 * add element to timer queue
 * timer_name: name of the timer that should be added
 * ts: timestamp, when the timer should fire
 * callback: callback function, that gets called if the timer fired
 * returns 1 if the element was added at the first position
 */
uint8_t tq_add_element(timer_name_t timer_name, uint64_t ts, void (*callback)) {
  uint8_t ret = 0;

  timer_queue_element_t* new_elem = malloc(sizeof(timer_queue_element_t));
  new_elem->timer_name = timer_name;
  new_elem->timestamp = ts;
  new_elem->callback = callback;
  new_elem->next = NULL;

  if(!first_element) { // empty queue
    first_element = new_elem;
    ret = 1;
  }
  else {
    timer_queue_element_t* p_elem = NULL;
    timer_queue_element_t* c_elem = first_element;

    while(c_elem) {
      if(c_elem->timestamp > ts) {
        if(p_elem) {
          p_elem->next = (struct timer_queue_element_t*) new_elem;
          new_elem->next = (struct timer_queue_element_t*) c_elem;
        }
        else { // if ts is smaller than the one of the first element
          // insert element at the beginning
          first_element = new_elem;
          new_elem->next = (struct timer_queue_element_t*) c_elem;
          ret = 1;
        }
        return ret;
      }
      else {
        p_elem = c_elem;
        c_elem = (timer_queue_element_t*) c_elem->next;
      }
    }
    // insert element at the end
    p_elem->next = (struct timer_queue_element_t*) new_elem;
  }
  return ret;
}

/*
 * removes the first element with timer_name
 * returns 1 if the first element was removed
 */
uint8_t tq_remove_element(timer_name_t timer_name) {
  uint8_t ret = 0;

  timer_queue_element_t* p_elem = NULL;
  timer_queue_element_t* c_elem = first_element;

  while(c_elem) {
    if(c_elem->timer_name == timer_name) {
      if(p_elem) {
        p_elem->next = c_elem->next;
      }
      else { // if the first element is removed
        first_element = (timer_queue_element_t*) c_elem->next;
        ret = 1;
      }
      free(c_elem);
      return ret;
    }
    else {
      p_elem = c_elem;
      c_elem = (timer_queue_element_t*) c_elem->next;
    }
  }
  return ret;
}

/*
 * print the names and timestamps of the elements in the queue
 */
void tq_print_queue() {
  timer_queue_element_t* c_elem = first_element;

  while(c_elem) {
    sprintf(char_buff, "Timer: %d, Timestamp: %d", c_elem->timer_name, (int) c_elem->timestamp);
    cli_println(char_buff);
    rtc_delay(100);

    c_elem = (timer_queue_element_t*) c_elem->next;
  }
  return;
}

/*
 * start a new timer
 * timer_name: name of the timer that should be added
 * ts: timestamp, when the timer should fire
 * callback: callback function, that gets called if the timer fired
 */
void tq_start_timer(timer_name_t timer_name, uint64_t ts, void (*callback)) {
  if(tq_add_element(timer_name, ts, callback)) { // if first element added restart timer
    tq_run_timer(true);
  }
}

/*
 * stop timer with name timer_name
 */
void tq_stop_timer(timer_name_t timer_name) {
  if(tq_remove_element(timer_name)) { // if first element removed restart timer
    tq_run_timer(true);
  }
}

/*
 * if a timer fired or no timer is running, start next timer
 * set_callback: only set the new callback if the old one was executed
 */
void tq_run_timer(bool set_callback) {
  // check if queue is not empty
  if(first_element) {
    // get next timer
    current_timer = first_element;

    if(set_callback) {
      // set callback if the timer was restarted
      // if the timer fired the new callback gets set after the old was executed
      tq_callback = current_timer->callback;
    }

    // start timer
    if(hs_timer_get_current_timestamp() < current_timer->timestamp) {
      hs_timer_timeout2_start(current_timer->timestamp, &tq_timer_fired);
    }
    else {
//      print_ts();
      sprintf(char_buff, "tq timestamp already passed! %d, %llu, %llu", (int) current_timer->timer_name, hs_timer_get_current_timestamp(), current_timer->timestamp);
      cli_println(char_buff);

      // execute callback of previous timer
      if(tq_callback) {
        tq_callback();
      }

      // set callback for the already passed timer (will be executed in tq_timer_fired())
      tq_callback = current_timer->callback;

      // remove timer and get next one
      tq_remove_element(current_timer->timer_name);
      tq_run_timer(false);
    }
  }
  else {
    // if queue is empty stop timer
    hs_timer_timeout2_stop();
  }
}


/*
 * callback function if a timer fired
 */
void tq_timer_fired() {
  // remove timer that just fired
  tq_remove_element(current_timer->timer_name);

  // start next timer
  tq_run_timer(false);

  // execute callback
  if(tq_callback) {
    tq_callback();
  }
  // set new callback after execution of the old one
  tq_callback = current_timer->callback;

  if(in_rec_fct()) {
    sprintf(char_buff, "timer infered in rec fct: %d", current_timer->timer_name);
    dozer_print(5, char_buff);
  }
}


/*
 * check if timer is already running
 */
bool tq_is_running(timer_name_t timer_name) {
  timer_queue_element_t* c_elem = first_element;

  while(c_elem) {
    if(c_elem->timer_name == timer_name) {
      return true;
    }
    else {
      c_elem = (timer_queue_element_t*) c_elem->next;
    }
  }

  return false;
}


/*
 * return the scheduled timestamp for the timer with name timer_name
 */
uint64_t tq_get_fire_ts(timer_name_t timer_name) {
  timer_queue_element_t* c_elem = first_element;

  while(c_elem) {
    if(c_elem->timer_name == timer_name) {
      return c_elem->timestamp;
    }
    else {
      c_elem = (timer_queue_element_t*) c_elem->next;
    }
  }

  return 0;
}

#endif // DOZER

