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

#define CONFIG_NUMBER_OF_PAGES 1U
#define CONFIG_FLASH_START 0x8000000U
#define CONFIG_FLASH_ADDRESS (CONFIG_FLASH_START + CONFIG_LAST_PAGE_INDEX * 2048)

#define NSS_LOW() HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_RESET)
#define NSS_HIGH() HAL_GPIO_WritePin(RADIO_NSS_GPIO_Port, RADIO_NSS_Pin, GPIO_PIN_SET)

#ifndef DEVKIT
#define RADIO_SPI hspi2
SPI_HandleTypeDef hspi2;
#else
#define RADIO_SPI hspi1
SPI_HandleTypeDef hspi1;
#endif

#endif /* SYSTEM_PLATFORM_H_ */
