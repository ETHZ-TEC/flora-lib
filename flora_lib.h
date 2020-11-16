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
#include "radio/radio.h"
#include "bolt/bolt.h"

/* protocols, networking */
#include "flocklab/flocklab.h"
#include "protocol/protocol.h"

/* misc utilities */
#include "utils/misc.h"
#include "utils/log.h"
#include "utils/nvcfg.h"
#include "utils/dcstat.h"
#include "utils/led.h"
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
