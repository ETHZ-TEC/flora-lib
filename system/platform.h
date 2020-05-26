/*
 * platform.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef SYSTEM_PLATFORM_H_
#define SYSTEM_PLATFORM_H_

#ifndef BASEBOARD
#define BASEBOARD     0
#endif /* BASEBOARD */


#ifndef DEVKIT
  #define CONFIG_MEMORY_BANK FLASH_BANK_1
  #define CONFIG_LAST_PAGE_INDEX 127U // 256 KiB Flash @ 2 KiB Pages
#else
  #define CONFIG_MEMORY_BANK FLASH_BANK_2
  #define CONFIG_LAST_PAGE_INDEX 511U // 1024 KiB Flash @ 2 KiB Pages
#endif

#define NSS_LOW() HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET)

#define FLASH_MEM_SIZE      0x00040000
#define FLASH_MEM_START     0x08000000  // = FLASH_BASE
#define FLASH_MEM_PAGE_SIZE 2048        // = FLASH_PAGE_SIZE
#define FLASH_MEM_PAGES     (FLASH_MEM_SIZE / FLASH_MEM_PAGE_SIZE)    // 128


#ifndef DEVKIT
#define RADIO_SPI hspi2
SPI_HandleTypeDef hspi2;
#else
#define RADIO_SPI hspi1
SPI_HandleTypeDef hspi1;
#endif

#endif /* SYSTEM_PLATFORM_H_ */
