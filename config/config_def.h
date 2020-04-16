/*
 * config_def.h
 *
 *  Created on: May 9, 2018
 *      Author: marku
 */

#ifndef CONFIG_CONFIG_DEF_H_
#define CONFIG_CONFIG_DEF_H_


typedef struct {
  uint32_t magic_word;
  uint32_t uid;
  uint32_t role; // [0: BASE, 1: RELAY]
  uint32_t cli_mode; // [0: machine mode, 1: user interactive]
} config_t;


#define CONFIG_MAGIC_WORD 0xdeadbeef

#endif /* CONFIG_CONFIG_DEF_H_ */
