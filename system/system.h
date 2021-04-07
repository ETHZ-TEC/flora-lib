/*
 * system.h
 *
 *  Created on: 25.04.2018
 *      Author: marku
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_


#include "stm32l4xx_hal.h"
#include "system/uart.h"
#include "system/gpio.h"
#include "system/gpio_exti.h"
#include "system/platform.h"
#include "system/system_boot.h"
#include "system/system_backup_registers.h"
#include "system/lpm.h"


void system_boot(void);
void system_init(void);
void system_run(void);
void system_update(void);
void system_sleep(bool deep);

void system_reset(void);
void system_reset_into_bootloader(void);
const char* system_get_reset_cause(uint8_t* out_reset_flag);

#endif /* SYSTEM_SYSTEM_H_ */
