/*
 * circular_queue.c
 *
 *  Created on: Jul 19, 2018
 *      Copy of the circular queue used in the TinyOS implementation of Dozer
 *
 */


#define MAX_TRANSMISSION_LENGTH 100

#include "circular_queue.h"



CQueue_t queue;
uint8_t lock;      // prevent deletion of a newer message if it arrived between a getFirst(...) and dropFirst call
bool QueueClosed=false;
bool print_queue = true;


/*************************************** StdControl *****************************************************/

/**
 * Initialize the queue struct
 */
uint8_t cq_init() {
  queue.firstUsed = 0;
  queue.firstFree = 0;
  queue.isFull = 0;
  lock = 0;
  return SUCCESS;
}

/************************************ CircularQueue *******************************************************/

/**
 * Append a new message to the queue.
 * The function first checks if the queue already contains
 * a DataMsg from the same originator and if so, replaces such a
 * message with the new one.
 */
uint8_t cq_append(uint8_t forceAdd, uint8_t dataType, uint8_t payloadLength, data_msg_t *msg){
  //uint8_t _index;
  uint64_t ts;

  if (QueueClosed) {
    dozer_print(1, "queue close");
    return ERROR;
  }

//  atomic { // TODO: atomic
  //_index = queue.firstUsed;
  ts = hs_timer_get_current_timestamp();
  if (queue.isFull != 0) {
    dozer_print(1, "queue full");
    if (print_queue) {
      for (int i = 0; i < QUEUE_SIZE; ++i) {
        sprintf(char_buff, "orig: %d, seqNr: %d", queue.buffer[i].originatorID, queue.buffer[i].seqNr);
        dozer_print(1, char_buff);
      }
      print_queue = false;
    }

    return ERROR;
  }

  // loop through the buffer and check if there
  // is a stored message from the same originator as
  // the new message
  /*while(_index != queue.firstFree){
  if(queue.buffer[_index].originatorID == msg->originatorID) {
    // if the previous message was forceAdded and the new message is not then return
    if (queue.forceAdd[_index] == 1 && (forceAdd == 0)){
    return ERROR;
    }
    // if this message is newer than the previously stored one
    if (queue.buffer[_index].seqNr - msg->seqNr < 0x8000)  {
    // if the first message in the queue is overwritten, make sure it does not get deleted by a pending dropMessag()
    if (_index == queue.firstUsed) {
      lock = 0;
    }
    queue.buffer[_index] = *msg;
    queue.dType[_index] = dataType;
    queue.payloadLength[_index] = payloadLength;
    queue.arrivalTime[_index] = ts;
    queue.forceAdd[_index] = forceAdd;
    return SUCCESS;
    }
    // the new message is older than the stored one -> drop it
    // TODO maybe return ERROR?
    // we overwrite.. changed 07.10.23
    return ERROR;
  }
  // move to the next message in the queue
  _index = (_index+1) % QUEUE_SIZE;
  }*/
  // there is no message from the same originator in the queue -> append it
  queue.buffer[queue.firstFree] = *msg;
  queue.dType[queue.firstFree] = dataType;
  queue.payloadLength[queue.firstFree] = payloadLength;
  queue.arrivalTime[queue.firstFree] = ts;
  queue.forceAdd[queue.firstFree] = forceAdd;

  queue.firstFree = (queue.firstFree+1) % QUEUE_SIZE;

  // check if the queue is full now
  if (queue.firstFree == queue.firstUsed) {
  queue.isFull = 1;
  }
//  }
  return SUCCESS;

}


/**
 * Copy the first message in the queue to buf and removes it from the queue
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is ERROR
 */
uint8_t cq_removeFirst(data_msg_t *buf){
  uint64_t curTime;
  if ((queue.firstFree == queue.firstUsed) && (queue.isFull == 0)){
    return ERROR;
  }
  curTime = hs_timer_get_current_timestamp();
  *buf = queue.buffer[queue.firstUsed];
  //*arrival = queue.arrivalTime[queue.firstUsed];

  //buf->aTime += curTime.low - queue.arrivalTime[queue.firstUsed];
//  buf->aTime = add(buf->aTime, sub(curTime, queue.arrivalTime[queue.firstUsed]));
  buf->aTime = buf->aTime + curTime - queue.arrivalTime[queue.firstUsed];
  queue.firstUsed = (queue.firstUsed+1) % QUEUE_SIZE;
  queue.isFull = 0;

  return SUCCESS;
}


/**
 * Deletes the first element in the queue.
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is ERROR
 */
uint8_t cq_dropFirst(){
//  atomic { // TODO: atomic
  if ((queue.firstFree == queue.firstUsed) && (queue.isFull == 0)){
    return ERROR;
  }
  if (lock == 1){
    queue.firstUsed = (queue.firstUsed+1) % QUEUE_SIZE;
    queue.isFull = 0;
    lock = 0;
  }
//  }
  return SUCCESS;
}

/**
 * Copy the first message in the queue to buf.
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is ERROR
 */
uint8_t cq_getFirst(data_msg_t *buf, uint8_t *dataType, uint8_t *payloadLength){
  uint64_t curTime;
//  atomic { // TODO:atomic

  if ((queue.firstFree == queue.firstUsed) && (queue.isFull == 0)){
    return ERROR;
  }

  curTime = hs_timer_get_current_timestamp();
  *buf = queue.buffer[queue.firstUsed];
  *dataType = queue.dType[queue.firstUsed];
  *payloadLength = queue.payloadLength[queue.firstUsed];
  //buf->aTime += curTime.low-queue.arrivalTime[queue.firstUsed];
//  buf->aTime = add(buf->aTime, sub (curTime, queue.arrivalTime[queue.firstUsed]));
  buf->aTime = buf->aTime + curTime - queue.arrivalTime[queue.firstUsed];
  lock = 1;
//  }
  return SUCCESS;
}


/**
 * Returns the number of buffered Messages
 */
uint8_t cq_cntUsedBuffer(){
  if (queue.firstFree > queue.firstUsed){
  // normal mode
  return (queue.firstFree-queue.firstUsed);
  } else if(queue.firstFree < queue.firstUsed){
  // wrapped mode
  return (queue.firstFree-queue.firstUsed+QUEUE_SIZE);
  } else if (queue.isFull == 0){
  // queue is empty
  return 0;
  } else{
  // queue is full
  return QUEUE_SIZE;
  }
}


/**
 * Returns the number of available buffer space
 */
uint8_t cq_cntFreeBuffer(){
  if (QueueClosed) {
    return 0;
  }
  else if (queue.firstFree > queue.firstUsed){
  // normal mode
  return (QUEUE_SIZE-(queue.firstFree-queue.firstUsed));
  } else if(queue.firstFree < queue.firstUsed){
  // wrapped mode
  return (queue.firstUsed-queue.firstFree);
  } else if (queue.isFull == 0){
  // queue is empty
  return QUEUE_SIZE;
  } else{
  // queue is full
  return 0;
  }
}

void cq_close(){
  QueueClosed=true;
}

void cq_open(){
  QueueClosed=false;
}

bool cq_isClosed(){
  return QueueClosed;
}
