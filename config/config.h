/*
 * config.h
 *
 *  Created on: May 9, 2018
 *      Author: marku
 */

#ifndef CONFIG_CONFIG_H_
#define CONFIG_CONFIG_H_

#include "flora_lib.h"
#include "config_def.h"

void config_init();
config_t* config_get();
void config_set(config_t config);
void config_store(config_t* config);
config_t* config_load();


void config_print(char* search);
void config_set_entry(char* entry, uint32_t value);

#endif /* CONFIG_CONFIG_H_ */
