/*
 * uart.c
 *
 *  Created on: Apr 24, 2018
 *      Author: marku
 */

#include "flora_lib.h"


#ifndef DEVKIT
#define UART huart1
  extern UART_HandleTypeDef huart1;
#define DMA_UART_TX hdma_usart1_tx;
#define DMA_UART_RX hdma_usart1_rx
  extern DMA_HandleTypeDef hdma_usart1_tx;
  extern DMA_HandleTypeDef hdma_usart1_rx;
#else
#define UART huart2
  extern UART_HandleTypeDef huart2;
#define DMA_UART_TX hdma_usart2_tx;
#define DMA_UART_RX hdma_usart2_rx
  extern DMA_HandleTypeDef hdma_usart2_tx;
  extern DMA_HandleTypeDef hdma_usart2_rx;
#endif


uart_fifo_t tx_fifo = {0};
uart_fifo_t rx_fifo = {0};

volatile static bool uart_initialized = false;



void uart_tx_fifo_send();



void uart_init()
{
  HAL_UART_Receive_DMA(&UART, (uint8_t*) rx_fifo.buffer, (uint16_t) UART_FIFO_BUFFER_SIZE);

  uart_initialized = true;
}

uart_fifo_t* uart_rx()
{
  if(uart_initialized)
  {
    rx_fifo.set_pointer = UART_FIFO_BUFFER_SIZE - (uint16_t) __HAL_DMA_GET_COUNTER(&DMA_UART_RX);
    rx_fifo.item_count = ((unsigned) rx_fifo.set_pointer - (unsigned) rx_fifo.get_pointer) % (unsigned) UART_FIFO_BUFFER_SIZE;
  }
  return &rx_fifo;
}


bool uart_tx(char* buffer, uint32_t size)
{
  uint32_t aborttime = HAL_GetTick() + UART_TX_TIMEOUT_MS;
  if(uart_initialized && size) {
    while (size && (HAL_GetTick() < aborttime)) {
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
  return (HAL_OK == HAL_UART_Transmit(&UART, (uint8_t*)buffer, size, 5000));
}

void uart_tx_fifo_send(void)
{
  if (tx_fifo.item_count && !tx_fifo.dma_transfer_count)
  {
    uint16_t processable_size;
    uint32_t next_item = tx_fifo.get_pointer;
    if ((next_item + tx_fifo.item_count) >= UART_FIFO_BUFFER_SIZE) {
      processable_size = UART_FIFO_BUFFER_SIZE - next_item;
    }
    else {
      processable_size = tx_fifo.item_count;
    }

    if (HAL_OK == HAL_UART_Transmit_DMA(&UART, (uint8_t*) (tx_fifo.buffer + next_item), processable_size))
    {
      tx_fifo.dma_transfer_count = processable_size;
    }
  }
}


bool uart_tx_fifo_empty(void)
{
  return (tx_fifo.item_count == 0);
}


bool uart_read(char* chr)
{
  if(uart_initialized)
  {
    uart_rx();

    if (rx_fifo.item_count > 0) {
      *chr = rx_fifo.buffer[rx_fifo.get_pointer];
      rx_fifo.get_pointer = ((unsigned) rx_fifo.get_pointer + 1U) % (unsigned) UART_FIFO_BUFFER_SIZE;
      rx_fifo.item_count--;
      return true;
    }
    else
    {
      return false;
    }

    return false;
  }
  else
  {
    return false;
  }

}



void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (uart_initialized)
  {
    tx_fifo.get_pointer = (tx_fifo.get_pointer + tx_fifo.dma_transfer_count) % UART_FIFO_BUFFER_SIZE;
    tx_fifo.item_count -= tx_fifo.dma_transfer_count;
    tx_fifo.dma_transfer_count = 0;
    uart_tx_fifo_send();
  }

  return;
}

