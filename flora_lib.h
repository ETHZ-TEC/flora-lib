/*
 * flora_lib.h
 *
 *  Created on: Apr 16, 2020
 *      Author: rdaforno
 */

#ifndef FLORA_LIB_H_
#define FLORA_LIB_H_

/* --- include required standard libraries --- */

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <limits.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include <time.h>
#include <math.h>


/* --- include global config --- */

#include "app_config.h"


/* --- includes all flora library files --- */

/* DPP definitions and lib */
#include "dpp/definitions/messages/dpp_message.h"
#include "dpp/libraries/dpp_lib.h"


/* drivers */
#include "system/system.h"
#include "time/rtc.h"
#include "time/lptimer.h"
#include "time/hs_timer.h"
#include "radio/flora_radio.h"
#include "bolt/bolt.h"

/* protocols, networking */
#include "flocklab/flocklab.h"
#include "protocol/protocol.h"
#include "protocol/gloria/gloria.h"
#if DOZER_ENABLE
#include "protocol/dozer/dozer.h"
#endif /* DOZER_ENABLE */
#if SLWB_ENABLE
#include "protocol/simple_lwb/slwb.h"
#endif /* SLWB_ENABLE */
#if GMW_ENABLE
#include "protocol/gmw/gmw.h"
#endif /* GMW_ENABLE */
#if ELWB_ENABLE
#include "protocol/elwb/elwb.h"
#endif /* ELWB_ENABLE */

/* misc utilities */
#include "utils/misc.h"
#include "utils/log.h"
#include "led/led.h"
#include "config/config.h"
#include "cli/cli.h"


/* --- application specific pin configuration --- */

#include "main.h"


/* --- global error checks --- */

#ifndef __OPTIMIZE__
#error "Compiler optimizations have to be enabled! Recommended setting is -O2."
#endif

#if __GNUC__ != 7
#warning "Compiler version has changed."
#endif


#endif /* FLORA_LIB_H_ */
