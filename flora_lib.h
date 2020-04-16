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
#include <ctype.h>
#include <stdarg.h>
#include <math.h>
//#include <sys/_stdint.h>


/* --- include global config --- */

#include "app_config.h"


/* --- includes all flora library files --- */

/* DPP definitions and lib */
#include "dpp/definitions/messages/dpp_message.h"
#include "dpp/libraries/dpp_lib.h"

/* drivers */
#include "arch/arch.h"
#include "system/system.h"
#include "radio/flora_radio.h"
#include "time/hs_timer.h"
#include "time/rtc.h"
#include "bolt/bolt.h"
#include "time/rtc.h"
#include "time/lptimer.h"
#include "time/hs_timer.h"

/* protocols */
#include "protocol/protocol.h"
#include "protocol/gloria/gloria.h"
#include "protocol/simple_lwb/slwb.h"
#include "protocol/elwb/elwb.h"

/* misc utilities */
#include "utils/log.h"
#include "led/led.h"
#include "flocklab/flocklab.h"
#include "config/config.h"
#include "cli/cli.h"


#endif /* FLORA_LIB_H_ */
