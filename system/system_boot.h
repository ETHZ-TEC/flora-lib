/*
 * system_boot.h
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#ifndef SYSTEM_SYSTEM_BOOT_H_
#define SYSTEM_SYSTEM_BOOT_H_

#include <stdbool.h>

#define SYSTEM_BOOTLOADER_ROM_ADDRESS 0x1FFF0000U


void system_bootmode_check();


#endif /* SYSTEM_SYSTEM_BOOT_H_ */
