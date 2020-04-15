/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 *
 * Authors: Reto Da Forno
 *          Tonio Gsell
 *          Akos Pasztor
 */

/*
 * packet structure and message type definitions
 * COM file (communication processor)
 */


#ifndef __DPP_COM_MESSAGE_H__
#define __DPP_COM_MESSAGE_H__


/* 
 * S T R U C T S
 */

#pragma pack(1)     /* force alignment to 1 byte */


#define DPP_COM_HEALTH_LEN    28    /* bytes */
typedef struct {
  /* all values are from the last health period, unless otherwise stated */
  uint32_t uptime;        /* Uptime [seconds] */
  uint16_t msg_cnt;       /* Number of received messages */
  uint16_t core_vcc;      /* Core voltage [10^-3 V] */
  int16_t  core_temp;     /* Core temperature [10^-2 °C] */
  uint16_t cpu_dc;        /* CPU duty cycle [10^-2 %] */
  uint8_t  stack;         /* Stack [watermark over the last period in %] */

  uint8_t  radio_snr;     /* Average signal-to-noise ratio [dBm] */
  uint8_t  radio_rssi;    /* Average RSSI value [-dBm] */
  int8_t   radio_tx_pwr;  /* Transmit power [dBm] */
  uint16_t radio_rx_dc;   /* Radio transmit duty cycle [10^-2 %] */
  uint16_t radio_tx_dc;   /* Radio listen duty cycle [10^-2 %] */
  uint16_t radio_per;     /* Radio packet error rate [10^-2 %] */
  
  uint16_t rx_cnt;        /* Number of successfully received packets */
  uint8_t  tx_queue;      /* Number of packets in the transmit buffer */
  uint8_t  rx_queue;      /* Number of packets in the receive buffer */
  uint8_t  tx_dropped;    /* Dropped packets due to TX queue full */
  uint8_t  rx_dropped;    /* Dropped packets due to RX queue full */
} dpp_com_health_t;


#define DPP_LWB_HEALTH_LEN    15   /* bytes */
typedef struct {
  uint8_t  bootstrap_cnt; /* Sync lost counter */
  uint8_t  sleep_cnt;     /* Deepsleep counter */
  uint16_t fsr;           /* Flood success rate [10^-2 %] */
  uint16_t t_to_rx;       /* Listen time to start of packet reception [us] */
  uint16_t t_flood;       /* Flood duration [us] */
  uint8_t  n_tx;          /* TX count of last Glossy flood */
  uint8_t  n_rx;          /* RX count of last Glossy flood */
  uint8_t  n_hops;        /* Average hop count of first received packet of a flood */
  uint8_t  n_hops_max;    /* Max. hop count of first received packet of a flood */
  uint8_t  unsynced_cnt;  /* Lost sync counter */
  uint8_t  drift;         /* estimated drift in ppm */
  uint8_t  bus_load;      /* bus utilization in % */
} dpp_lwb_health_t;


/* response to a 'get'/'read' command */
#define DPP_COM_RESPONSE_HDR_LEN  4
typedef struct {
  dpp_command_type_t type;                                                  /* what was the command */
  uint16_t           arg;                                                   /* what was the argument */
  uint8_t            res[DPP_MSG_PAYLOAD_LEN - DPP_COM_RESPONSE_HDR_LEN];   /* result / response (binary data) */
} dpp_com_response_t;


#pragma pack()


#endif /* __DPP_COM_MESSAGE_H__ */
