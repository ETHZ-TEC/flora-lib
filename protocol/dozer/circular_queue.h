/*
 * circular_queue.h
 *
 *  Created on: Jul 19, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_CIRCULAR_QUEUE_H_
#define PROTOCOL_DOZER_CIRCULAR_QUEUE_H_


#ifndef CQUEUEH_BUFFER_SIZE
  // TODO migrate to constants
  #define CQUEUEH_BUFFER_SIZE (20)
#endif


enum{
  QUEUE_SIZE = CQUEUEH_BUFFER_SIZE,
};

typedef struct CQueue{
    data_msg_t buffer[QUEUE_SIZE];
    uint64_t arrivalTime[QUEUE_SIZE];
    uint8_t dType[QUEUE_SIZE];
    uint8_t payloadLength[QUEUE_SIZE];
    uint8_t forceAdd[QUEUE_SIZE];
    uint8_t firstUsed;
    uint8_t firstFree;
    uint8_t isFull;
} CQueue_t;




/**
 * Initialize the queue struct
 */
uint8_t cq_init();

/**
 * Append a new message to the queue.
 * The function first checks if the queue already contains
 * a DataMsg from the same originator and if so, replaces such a
 * message with the new one.
 */
uint8_t cq_append(uint8_t forceAdd, uint8_t dataType, uint8_t payloadLength, data_msg_t *msg);


/**
 * Copy the first message in the queue to buf and removes it from the queue
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is FAIL
 */
uint8_t cq_removeFirst(data_msg_t *buf);


/**
 * Deletes the first element in the queue.
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is FAIL
 */
uint8_t cq_dropFirst();


/**
 * Copy the first message in the queue to buf.
 * Returns SUCCESS if the queue is NOT empty.
 * Otherwise the return value is FAIL
 */
uint8_t cq_getFirst(data_msg_t *buf, uint8_t *dataType, uint8_t *payloadLength);


/**
 * Returns the number of buffered Messages
 */
uint8_t cq_cntUsedBuffer();


/**
 * Returns the number of available buffer space
 */
uint8_t cq_cntFreeBuffer();

void cq_close();

void cq_open();

bool cq_isClosed();



#endif /* PROTOCOL_DOZER_CIRCULAR_QUEUE_H_ */
