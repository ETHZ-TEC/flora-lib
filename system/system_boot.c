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
