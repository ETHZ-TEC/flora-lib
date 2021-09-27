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

#ifndef SYSTEM_UART_H_
#define SYSTEM_UART_H_


#ifndef UART_FIFO_BUFFER_SIZE
#define UART_FIFO_BUFFER_SIZE    1024       /* buffer size for RX and TX queues, must be < 65536 */
#endif /* UART_FIFO_BUFFER_SIZE */

#ifndef UART_TX_TIMEOUT_MS
#define UART_TX_TIMEOUT_MS       100        /* uart_tx will block for at most this time */
#endif /* UART_TX_TIMEOUT_MS */

#ifndef UART_RX_ENABLE
#define UART_RX_ENABLE           CLI_ENABLE   /* by default, only enable if CLI is enabled */
#endif /* UART_RX_ENABLE */


typedef struct
{
  uint8_t buffer[UART_FIFO_BUFFER_SIZE];
  volatile uint16_t set_pointer;
  volatile uint16_t get_pointer;
  volatile uint16_t item_count;
  volatile uint16_t dma_transfer_count;
} uart_fifo_t;


void uart_init_rx();

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
