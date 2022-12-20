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

#ifndef SYSTEM_PLATFORM_H_
#define SYSTEM_PLATFORM_H_


#define FLASH_MEM_START     0x08000000  // = FLASH_BASE
#define FLASH_MEM_PAGE_SIZE 2048        // = FLASH_PAGE_SIZE
#ifndef DEVKIT
  #define FLASH_MEM_SIZE    0x00040000
  #define FLASH_MEM_PAGES   (FLASH_MEM_SIZE / FLASH_MEM_PAGE_SIZE)    // 128
#else
  #define FLASH_MEM_SIZE    0x00100000  // 1MB
  #define FLASH_MEM_PAGES   (FLASH_MEM_SIZE / FLASH_MEM_PAGE_SIZE)    // 512
#endif

// unique 96-bit device ID consisting of the X/Y coordinates on the wafer (32 bits), wafer number (8 bits) and lot number (56 bits)
#define DEVICE_ID_REG       0x1FFF7590


#ifndef DEVKIT
  #define RADIO_SPI hspi2
  extern SPI_HandleTypeDef hspi2;
  #ifndef UART
    #define UART huart1
  #endif
  extern UART_HandleTypeDef UART;
#else
  #define RADIO_SPI hspi1
  extern SPI_HandleTypeDef hspi1;
  #define UART huart2
  extern UART_HandleTypeDef huart2;
#endif


#define HALTICK_TIMER       htim1
#define HALTICK_IRQ         TIM1_UP_TIM16_IRQn
extern TIM_HandleTypeDef    htim1;


#endif /* SYSTEM_PLATFORM_H_ */
