/*
 * uart.h
 *
 *  Created on: Apr 24, 2018
 *      Author: marku
 */

#ifndef SYSTEM_UART_H_
#define SYSTEM_UART_H_


#ifndef UART_FIFO_BUFFER_SIZE
#define UART_FIFO_BUFFER_SIZE    1024       /* must be < 65536 */
#endif /* UART_FIFO_BUFFER_SIZE */

#ifndef UART_TX_TIMEOUT_MS
#define UART_TX_TIMEOUT_MS       100        /* uart_tx will block for at most this time */
#endif /* UART_TX_TIMEOUT_MS */


typedef struct
{
  uint8_t buffer[UART_FIFO_BUFFER_SIZE];
  uint16_t set_pointer;
  uint16_t get_pointer;
  uint16_t item_count;
  uint16_t dma_transfer_count;
} uart_fifo_t;


void uart_init();
void uart_deinit();

uart_fifo_t* uart_rx(void);

bool uart_tx_fifo_empty(void);

/*
 * Write asynchronously to TX buffer using DMA
 */
bool uart_tx(char* buffer, uint32_t size);

/*
 * Write synchronously to the UART device (blocking call)
 */
bool uart_tx_direct(char* buffer, uint32_t size);

void uart_transmit_fifo(void);

bool uart_read(char* chr);

/* block until TX is complete (or until the timeout is reached) */
void uart_wait_tx_complete(uint32_t timeout_ms);


#endif /* SYSTEM_UART_H_ */
