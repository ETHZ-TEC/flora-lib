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

/*---------------------------------------------------------------------------*/
/**
 * \addtogroup gmw-platform
 * @{
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *
 *            Platform dependent implementation for GMW
 */
/*---------------------------------------------------------------------------*/
#include "flora_lib.h"

#if GMW_ENABLE
/*---------------------------------------------------------------------------*/
void
gmw_platform_init(void)
{
#ifdef GLOSSY_RF_PIN
  PIN_UNSEL(GLOSSY_RF_PIN);
  PIN_CLR(GLOSSY_RF_PIN);
  PIN_CFG_OUT(GLOSSY_RF_PIN);
#endif /* GLOSSY_RF_PIN */

#ifdef GLOSSY_START_PIN
  PIN_UNSEL(GLOSSY_START_PIN);
  PIN_CLR(GLOSSY_START_PIN);
  PIN_CFG_OUT(GLOSSY_START_PIN);
#endif /* GLOSSY_START_PIN */

#ifdef GLOSSY_TX_PIN
  PIN_UNSEL(GLOSSY_TX_PIN);
  PIN_CLR(GLOSSY_TX_PIN);
  PIN_CFG_OUT(GLOSSY_TX_PIN);
#endif /* GLOSSY_TX_PIN */

#ifdef GLOSSY_RX_PIN
  PIN_UNSEL(GLOSSY_RX_PIN);
  PIN_CLR(GLOSSY_RX_PIN);
  PIN_CFG_OUT(GLOSSY_RX_PIN);
#endif /* GLOSSY_RX_PIN */

#ifdef GMW_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(GMW_CONF_TASK_ACT_PIN);
  PIN_CLR(GMW_CONF_TASK_ACT_PIN);
#endif /* GMW_CONF_TASK_ACT_PIN */

#ifdef GMW_CONF_SLOT_ACT_PIN
  PIN_CFG_OUT(GMW_CONF_SLOT_ACT_PIN);
  PIN_CLR(GMW_CONF_SLOT_ACT_PIN);
#endif /* GMW_CONF_SLOT_ACT_PIN */

#ifdef GMW_NOISE_DETECT_PIN
  PIN_UNSEL(GMW_NOISE_DETECT_PIN);
  PIN_CFG_OUT(GMW_NOISE_DETECT_PIN);
  PIN_CLR(GMW_NOISE_DETECT_PIN);
#endif /* GMW_NOISE_DETECT_PIN */

#ifdef GMW_GLOSSY_DETECT_PIN
  PIN_UNSEL(GMW_GLOSSY_DETECT_PIN);
  PIN_CFG_OUT(GMW_GLOSSY_DETECT_PIN);
  PIN_CLR(GMW_GLOSSY_DETECT_PIN);
#endif /* GMW_GLOSSY_DETECT_PIN */

#ifdef GMW_CONF_DEBUG_PIN
  PIN_UNSEL(GMW_CONF_DEBUG_PIN);
  PIN_CFG_OUT(GMW_CONF_DEBUG_PIN);
  PIN_CLR(GMW_CONF_DEBUG_PIN);
#endif /* GMW_CONF_DEBUG_PIN */

  //lptimer_ext_init();    -> is already called in platform.c
}
/*---------------------------------------------------------------------------*/
void
gmw_set_maximum_packet_length(uint8_t length)
{
  //TODO
}
/*---------------------------------------------------------------------------*/
void
gmw_set_rf_channel(gmw_rf_tx_channel_t channel)
{
  //TODO
}
/*---------------------------------------------------------------------------*/
void
gmw_set_tx_power(gmw_rf_tx_power_t power)
{
  //TODO
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_high_noise_detected(void)
{
#if GMW_CONF_USE_NOISE_DETECTION
  return gmw_high_noise_test();
#else
  return 0;
#endif
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_communication_active(void)
{
  //TODO
  return 0;
}
/*---------------------------------------------------------------------------*/
int8_t
gmw_get_rssi_last(void)
{
  //TODO
  return 0;
}
/*---------------------------------------------------------------------------*/
#if GMW_CONF_USE_MULTI_PRIMITIVES
//TODO
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
/*---------------------------------------------------------------------------*/

#endif /* GMW_ENABLE */

/** @} */
