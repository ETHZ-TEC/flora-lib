/*
 * radio_platform.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef RADIO_RADIO_PLATFORM_H_
#define RADIO_RADIO_PLATFORM_H_

#include <stdbool.h>

// Platform specific commands

void radio_set_nss_pin();
bool radio_read_busy_pin();
bool radio_read_dio1_pin();

#endif /* RADIO_RADIO_PLATFORM_H_ */
