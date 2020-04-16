/*
 * system_backup_registers.c
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#include "arch/arch.h"


system_backup_registers_t* backup_registers;

system_backup_registers_t* system_backup_access() {
  // Enable access to the backup registers

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  // Disable write-protection of the backup registers
  *SYSTEM_RTC_WPR = 0xCA;
  *SYSTEM_RTC_WPR = 0x53;

  // Map backup registers to backup register struct
  backup_registers = (system_backup_registers_t*) SYSTEM_RTC_BKUPR0;

  return backup_registers;
}

system_backup_registers_t* system_backup_get() {
  return backup_registers;
}
