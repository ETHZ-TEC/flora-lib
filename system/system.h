/*
 * system.h
 *
 *  Created on: 25.04.2018
 *      Author: marku
 */

#ifndef SYSTEM_SYSTEM_H_
#define SYSTEM_SYSTEM_H_

#include <stdint.h>
#include <stdbool.h>

/* include all stm32hal related files */
#include "stm32l4xx_hal.h"
#include "arch/stm32hal/gpio.h"
#include "arch/stm32hal/gpio_exti.h"
#include "arch/stm32hal/misc.h"


void system_boot();
void system_init();
void system_run();
void system_update();
void system_sleep(bool deep);

void system_reset();
void system_reset_into_bootloader();

#endif /* SYSTEM_SYSTEM_H_ */
