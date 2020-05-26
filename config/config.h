/*
 * config.h
 *
 *  Created on: May 9, 2018
 *      Author: marku
 */

#ifndef CONFIG_CONFIG_H_
#define CONFIG_CONFIG_H_

#include "config_def.h"

#ifndef CONFIG_ENABLE
#define CONFIG_ENABLE           1
#endif /* CONFIG_ENABLE */


#define CONFIG_NUMBER_OF_PAGES  1U
#define CONFIG_FLASH_START      0x8000000U
#define CONFIG_FLASH_ADDRESS    (CONFIG_FLASH_START + CONFIG_LAST_PAGE_INDEX * 2048)


void config_init();
config_t* config_get();
void config_set(config_t config);
void config_store(config_t* config);
config_t* config_load();


void config_print(char* search);
void config_set_entry(char* entry, uint32_t value);

#endif /* CONFIG_CONFIG_H_ */
