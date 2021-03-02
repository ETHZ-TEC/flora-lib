/*
 * platform.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef SYSTEM_PLATFORM_H_
#define SYSTEM_PLATFORM_H_


#ifndef DEVKIT
  #define CONFIG_MEMORY_BANK FLASH_BANK_1
  #define CONFIG_LAST_PAGE_INDEX 127U // 256 KiB Flash @ 2 KiB Pages
#else
  #define CONFIG_MEMORY_BANK FLASH_BANK_2
  #define CONFIG_LAST_PAGE_INDEX 511U // 1024 KiB Flash @ 2 KiB Pages
#endif


#define FLASH_MEM_SIZE      0x00040000
#define FLASH_MEM_START     0x08000000  // = FLASH_BASE
#define FLASH_MEM_PAGE_SIZE 2048        // = FLASH_PAGE_SIZE
#define FLASH_MEM_PAGES     (FLASH_MEM_SIZE / FLASH_MEM_PAGE_SIZE)    // 128

// unique 96-bit device ID consisting of the X/Y coordinates on the wafer (32 bits), wafer number (8 bits) and lot number (56 bits)
#define DEVICE_ID_REG       0x1FFF7590


#ifndef DEVKIT
#define RADIO_SPI hspi2
extern SPI_HandleTypeDef hspi2;
#else
#define RADIO_SPI hspi1
extern SPI_HandleTypeDef hspi1;
#endif


#define HALTICK_TIMER       htim1
#define HALTICK_IRQ         TIM1_UP_TIM16_IRQn
extern TIM_HandleTypeDef    htim1;


#endif /* SYSTEM_PLATFORM_H_ */
