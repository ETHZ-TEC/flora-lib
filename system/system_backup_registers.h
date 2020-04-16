/*
 * system_backup_registers.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef SYSTEM_SYSTEM_BACKUP_REGISTERS_H_
#define SYSTEM_SYSTEM_BACKUP_REGISTERS_H_


#define SYSTEM_RTC_WPR ((uint32_t*) (RTC_BASE + 0x24U))
#define SYSTEM_RTC_BKUPR0 ((uint32_t*) (RTC_BASE + 0x50U))


typedef enum {
  SYSTEM_BOOT_DEFAULT = 0x0,
  SYSTEM_BOOT_BOOTLOADER = 0x42
} bootmode_t;


typedef struct
{
  // up to 32 words
  uint32_t hal_rtc_set; // written in main.c by STM32Cube HAL
  uint32_t bootmode;
  uint32_t sleepmode;
} system_backup_registers_t;


system_backup_registers_t* system_backup_access();
system_backup_registers_t* system_backup_get();




#endif /* SYSTEM_SYSTEM_BACKUP_REGISTERS_H_ */
