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


#ifndef DEVKIT
  #define DMA_UART_TX hdma_usart1_tx
  #define DMA_UART_RX hdma_usart1_rx
  extern DMA_HandleTypeDef hdma_usart1_tx;
  extern DMA_HandleTypeDef hdma_usart1_rx;
#else
  #define DMA_UART_TX hdma_usart2_tx
  #define DMA_UART_RX hdma_usart2_rx
  extern DMA_HandleTypeDef hdma_usart2_tx;
  extern DMA_HandleTypeDef hdma_usart2_rx;
#endif


static uart_fifo_t tx_fifo = {0};
#if UART_RX_ENABLE
static uart_fifo_t rx_fifo = {0};
#endif /* UART_RX_ENABLE */

volatile static bool uart_initialized = false;


void uart_tx_fifo_send();


void uart_init()
{
#if UART_RX_ENABLE
  HAL_UART_Receive_DMA(&UART, (uint8_t*) rx_fifo.buffer, (uint16_t) UART_FIFO_BUFFER_SIZE);
#endif /* UART_RX_ENABLE */

  uart_initialized = true;
}


#if UART_RX_ENABLE

uart_fifo_t* uart_rx()
{
  if (uart_initialized)
  {
    rx_fifo.set_pointer = UART_FIFO_BUFFER_SIZE - (uint16_t) __HAL_DMA_GET_COUNTER(&DMA_UART_RX);
    rx_fifo.item_count = ((unsigned) rx_fifo.set_pointer - (unsigned) rx_fifo.get_pointer) % (unsigned) UART_FIFO_BUFFER_SIZE;
  }
  return &rx_fifo;
}

#endif /* UART_RX_ENABLE */


bool uart_tx(char* buffer, uint32_t size)
{
  uint64_t aborttime = lptimer_now() + LPTIMER_MS_TO_TICKS(UART_TX_TIMEOUT_MS);
  if (uart_initialized && size) {
    while (size && (lptimer_now() < aborttime)) {
      uint16_t processable_size = (UART_FIFO_BUFFER_SIZE - tx_fifo.item_count);
      if (processable_size > size) {
        processable_size = size;
      }
      if (processable_size) {
        if ((tx_fifo.set_pointer + processable_size) > UART_FIFO_BUFFER_SIZE) {
          processable_size = UART_FIFO_BUFFER_SIZE - tx_fifo.set_pointer;
        }
        memcpy((void*) tx_fifo.buffer + tx_fifo.set_pointer, (void*) buffer, processable_size);
        tx_fifo.set_pointer = (tx_fifo.set_pointer + processable_size) % UART_FIFO_BUFFER_SIZE;
        size               -= processable_size;
        buffer             += processable_size;
        tx_fifo.item_count += processable_size;

        uart_tx_fifo_send();
      }
    }
    if (size == 0) {
      return true;
    }
  }
  return false;
}


/* send directly without using the TX FIFO or DMA support / interrupts (blocking call) */
bool uart_tx_direct(char* buffer, uint32_t size)
{
  return (HAL_OK == HAL_UART_Transmit(&UART, (uint8_t*)buffer, size, UART_TX_TIMEOUT_MS));
}


void uart_tx_fifo_send(void)
{
  if (tx_fifo.item_count && !tx_fifo.dma_transfer_count) {
    uint16_t processable_size;
    uint32_t next_item = tx_fifo.get_pointer;
    if ((next_item + tx_fifo.item_count) >= UART_FIFO_BUFFER_SIZE) {
      processable_size = UART_FIFO_BUFFER_SIZE - next_item;
    } else {
      processable_size = tx_fifo.item_count;
    }

    if (HAL_OK == HAL_UART_Transmit_DMA(&UART, (uint8_t*) (tx_fifo.buffer + next_item), processable_size)) {
      tx_fifo.dma_transfer_count = processable_size;
    }
  }
}


bool uart_tx_fifo_empty(void)
{
  return (tx_fifo.item_count == 0);
}


#if UART_RX_ENABLE

bool uart_read(char* chr)
{
  if(uart_initialized) {
    uart_rx();
    if (rx_fifo.item_count > 0) {
      *chr = rx_fifo.buffer[rx_fifo.get_pointer];
      rx_fifo.get_pointer = ((unsigned) rx_fifo.get_pointer + 1U) % (unsigned) UART_FIFO_BUFFER_SIZE;
      rx_fifo.item_count--;
      return true;
    } else {
      return false;
    }
  }
  return false;
}

#endif /* UART_RX_ENABLE */


void uart_wait_tx_complete(uint32_t timeout_ms)
{
  if (uart_initialized && tx_fifo.item_count) {
    uart_tx_fifo_send();    // make sure a DMA transfer is active
    uint64_t aborttime = lptimer_now() + LPTIMER_MS_TO_TICKS(timeout_ms);
    while (tx_fifo.item_count && (lptimer_now() < aborttime));
  }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_initialized) {
    tx_fifo.get_pointer = (tx_fifo.get_pointer + tx_fifo.dma_transfer_count) % UART_FIFO_BUFFER_SIZE;
    tx_fifo.item_count -= tx_fifo.dma_transfer_count;
    tx_fifo.dma_transfer_count = 0;
    uart_tx_fifo_send();
  }
}

