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
