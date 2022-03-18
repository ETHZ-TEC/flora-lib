/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "utils/fw_ota.h"
#include "cli/cli.h"

/* deployments */
#include "deployment/ps.h"


/* --- application specific pin configuration --- */

#include "main.h"


/* --- global error checks --- */

#if !defined(__OPTIMIZE__) && (!defined(FLORA_COMPILER_OPT_CHECK) || (FLORA_COMPILER_OPT_CHECK != 0))
#error "Compiler optimizations have to be enabled! Recommended setting is -O2."
#endif

#if (__GNUC__ != 7) && (!defined(FLORA_COMPILER_VER_CHECK) || (FLORA_COMPILER_VER_CHECK != 0))
#warning "Compiler version has changed."
#endif


#endif /* FLORA_LIB_H_ */
