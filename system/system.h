/*
 * system.h
 *
 *  Created on: 25.04.2018
 *      Author: marku
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

/* include lib and global defines */
#include "flora_lib.h"

/* include all stm32hal related files */
#include "system_boot.h"
#include "system_backup_registers.h"


void system_boot();
void system_init();
void system_run();
void system_update();
void system_sleep(bool deep);

void system_reset();
void system_reset_into_bootloader();

#endif /* SYSTEM_SYSTEM_H_ */
