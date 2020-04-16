/*
 * system_boot.c
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#include "flora_lib.h"


typedef  void (*pFunction)(void);
pFunction system_jump_to_bootloader;


void system_bootmode_check() {


  // Read-out which bootmode was selected
  uint32_t bootmode = system_backup_access()->bootmode;

  // Check if BOOTLOADER bootmode was set before
  if (bootmode == SYSTEM_BOOT_BOOTLOADER)
  {
    // Reset bootmode to boot normally after next device reset.
    system_backup_get()->bootmode = SYSTEM_BOOT_DEFAULT;

    HAL_RCC_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    __disable_irq();

    // ARM Cortex-M Programming Guide to Memory Barrier Instructions.
    __DSB();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    // Remap is not visible at once. Execute some unrelated command!
    __DSB();
    __ISB();

    system_jump_to_bootloader = (void (*)(void)) (*((uint32_t *)(SYSTEM_BOOTLOADER_ROM_ADDRESS + 4)));
    __set_MSP(*(__IO uint32_t*) SYSTEM_BOOTLOADER_ROM_ADDRESS);
    system_jump_to_bootloader();
  }

  return;
}
