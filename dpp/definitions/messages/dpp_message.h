/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
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
 * MASTER file
 */

#ifndef __DPP_MESSAGE_H__
#define __DPP_MESSAGE_H__


/*
 * D E F I N E S
 */

/* max. message length (header + payload + CRC), limited by the COM module */
#define DPP_MSG_PKT_LEN           126     /* uint8_t (max. 255) */
#define DPP_MSG_HDR_LEN           16
/* max. message payload length */
#define DPP_MSG_PAYLOAD_LEN       (DPP_MSG_PKT_LEN - DPP_MSG_HDR_LEN - 2)
/* actual message length incl. CRC */
#define DPP_MSG_LEN(msg)          \
    ((msg)->header.payload_len + DPP_MSG_HDR_LEN + 2)

/* minimal message type with smaller header (set last bit of message type to
 * indicate an minimal message) */
#define DPP_MSG_TYPE_MIN          0x80    /* mask (msb) */
#define DPP_MSG_MIN_PKT_LEN       128     /* uint8_t (max. 255) */
#define DPP_MSG_MIN_HDR_LEN       4
#define DPP_MSG_MIN_PAYLOAD_LEN   \
    (DPP_MSG_MIN_PKT_LEN - DPP_MSG_MIN_HDR_LEN - 2)
#define DPP_MSG_MIN_LEN(msg)      \
    ((msg).header.payload_len + DPP_MSG_MIN_HDR_LEN + 2)


/* special (reserved) device IDs */
#define DPP_DEVICE_ID_SINK        0
#define DPP_DEVICE_ID_BROADCAST   0xffff


/* the message type (7 bits available, MSB is reserved) */
typedef enum {
  /* general message types */
  DPP_MSG_TYPE_INVALID      = 0,    /* unknown / invalid message type */
  DPP_MSG_TYPE_TIMESYNC     = 1,    /* timestamp */
  DPP_MSG_TYPE_EVENT        = 2,    /* event notification */
  DPP_MSG_TYPE_NODE_INFO    = 3,    /* node info (sent after a reset) */
  DPP_MSG_TYPE_CMD          = 4,    /* command message */
  DPP_MSG_TYPE_FW           = 5,    /* firmware update message */

  /* message types concerning the communication processor */
  DPP_MSG_TYPE_COM_RESPONSE = 10,   /* command response */
  DPP_MSG_TYPE_COM_HEALTH   = 11,   /* com processor periodic health message */
  DPP_MSG_TYPE_LWB_HEALTH   = 12,   /* periodic LWB health message */

  /* message types concerning the application processor */
  DPP_MSG_TYPE_APP_HEALTH   = 21,   /* app processor periodic health message */

  /* application-specific message types (must start from 30U) */
  DPP_MSG_TYPE_AE_EVENT     = 32,   /* acoustic emission event */
  DPP_MSG_TYPE_AE_DATA      = 33,   /* acoustic emission data */
  DPP_MSG_TYPE_GNSS_SV      = 34,   /* GNSS SV message */
  DPP_MSG_TYPE_WGPS_STATUS  = 35,   /* Wireless GPS  */
  DPP_MSG_TYPE_GEOPHONE_ACQ = 36,   /* Geophone acquire data */
  DPP_MSG_TYPE_IMU          = 37,   /* imu data */
  DPP_MSG_TYPE_ADCDATA      = 38,   /* event acquistion (ADC) data */
  DPP_MSG_TYPE_INCLINO      = 39,   /* inclinometer data */

  /* event-driven specific message types */
  DPP_MSG_TYPE_STAG_WAKEUP  = 41,   /* Staggered Wakeup message */
  DPP_MSG_TYPE_ACK_COMMAND  = 42,   /* Acknowledgment from the Base Station combined with a command to the sensor nodes */


  /* no types below this */
  DPP_MSG_TYPE_LASTID       = 127
} dpp_message_type_t;


/* timestamp in microseconds */
typedef uint64_t dpp_timestamp_t;


/*
 * I N C L U D E S
 */

#include "dpp_component_id.h"
#include "dpp_type.h"
#include "dpp_com_message.h"
#include "dpp_app_message.h"

/*
 * S T R U C T S
 */

/* for all following structs, force alignment to 1 byte */
#pragma pack(1)


#define DPP_NODE_INFO_LEN   42          /* bytes */
typedef struct {
  uint8_t             component_id;     /* application specific component id */
  uint8_t             rst_flag;         /* reset cause / source */
  uint16_t            rst_cnt;          /* reset counter */
  uint8_t             mcu_desc[12];     /* MCU description, i.g. 'CC430F5147' */
  uint8_t             compiler_desc[4]; /* compiler abbreviation (e.g. 'GCC') */
  uint32_t            compiler_ver;     /* compiler version: human-readable (e.g. AAABBBCCC where A: major, B: minor, C: patch */
  uint32_t            compile_date;     /* compilation time (seconds since 1970) */
  uint8_t             fw_name[8];       /* name of the firmware/application */
  uint16_t            fw_ver;           /* firmware version: human-readable (e.g. ABBCC where A: major, B: minor, C: patch */
  uint32_t            sw_rev_id;        /* repository revision number (GIT or SVN) */
} dpp_node_info_t;


#define DPP_EVENT_LEN       6
typedef struct {
  dpp_event_type_t    type;       /* event type / id */
  uint32_t            value;      /* event value / subtype */
} dpp_event_t;


#define DPP_COMMAND_HDR_LEN 2
typedef struct {
  dpp_command_type_t  type;         /* command ID */
  union {
    uint8_t           arg[DPP_MSG_PAYLOAD_LEN - 2];   /* arguments (length depends on command) */
    uint16_t          arg16[(DPP_MSG_PAYLOAD_LEN - 2) / 2];
    uint32_t          arg32[(DPP_MSG_PAYLOAD_LEN - 2) / 4];
  };
} dpp_command_t;


#define DPP_FW_HDR_LEN      4
#define DPP_FW_BLOCK_SIZE   64
typedef struct {
  dpp_fw_type_t type : 8;
  uint8_t       component_id;
  uint16_t      version;
  union {
    struct {
      uint16_t  len;    /* total length in bytes */
      uint32_t  crc;    /* 32bit CRC over complete FW data ('len' bytes) */
    } info;
    struct {
      uint16_t  ofs;    /* offset = block ID (starting from 0) */
      uint8_t   data[DPP_FW_BLOCK_SIZE];
    } data;
    struct {
      uint16_t  num;    /* 16-bit to avoid alignment issues */
      uint16_t  block_ids[(DPP_MSG_PAYLOAD_LEN - 2 - DPP_FW_HDR_LEN) / 2];
    } req;
  };
} dpp_fw_t;


/* application layer packet format (a packet is called 'message') */
/* crc is calculated over the whole structure and appended to the message */
typedef struct {
  /* header: DPP_MSG_HDR_LEN bytes */
  struct {
    uint16_t            device_id;        /* sender node ID */
    dpp_message_type_t  type : 8;         /* force 1 byte */
    uint8_t             payload_len;      /* payload length [bytes] */
    uint16_t            target_id;        /* recipient device ID */
    uint16_t            seqnr;            /* sequence number */
    uint64_t            generation_time;  /* packet generation time (us) */
  } header;
  /* none of the union members may be larger than DPP_MSG_PAYLOAD_LEN bytes
   * except for 'payload' */
  union {
    dpp_com_health_t    com_health;
    dpp_app_health_t    app_health;
    dpp_command_t       cmd;
    dpp_node_info_t     node_info;
    dpp_event_t         evt;              /* do not rename to 'event', compiler usage in TinyOS */
    dpp_imu_t           imu;
    dpp_inclino_t       inclino;
    dpp_gnss_sv_t       gnss_sv;
    dpp_wgps_status_t   wgps_status;
    dpp_timestamp_t     timestamp;
    dpp_fw_t            firmware;
    dpp_lwb_health_t    lwb_health;
    dpp_geophone_acq_t  geo_acq;
    dpp_geophone_adc_t  geo_adc;
    uint8_t             payload[DPP_MSG_PAYLOAD_LEN + 2];   /* raw bytes (add +2 to increase overall structure size to include the crc!) */
    uint16_t            payload16[DPP_MSG_PAYLOAD_LEN / 2]; /* rounded down! */
  };
} dpp_message_t;


/* application layer packet format (minimal type) */
typedef struct {
  struct {
    uint16_t            device_id;       /* sender node ID */
    dpp_message_type_t  type : 8;        /* message type (MSB must be set!) */
    uint8_t             payload_len;     /* payload length [bytes] */
  } header;
  union {
    uint8_t             payload[DPP_MSG_MIN_PAYLOAD_LEN + 2];   /* raw bytes */
  };
} dpp_message_min_t;


#pragma pack()    /* restore default structure packing rule */


/*
 * M A C R O S
 */

#define DPP_MSG_SET_CRC16(msg, crc) \
  (msg)->payload[(msg)->header.payload_len] = crc & 0xff; \
  (msg)->payload[(msg)->header.payload_len + 1] = (crc >> 8) & 0xff;
#define DPP_MSG_GET_CRC16(msg)      \
  ((uint16_t)(msg)->payload[(msg)->header.payload_len] | \
   (uint16_t)(msg)->payload[(msg)->header.payload_len + 1] << 8)


/*
 * E R R O R   C H E C K S
 */

/* error checks (max. message length) */
#if (DPP_MSG_PAYLOAD_LEN < DPP_MSG_COM_HEALTH_LEN) || \
    (DPP_MSG_PAYLOAD_LEN < DPP_MSG_NODE_INFO_LEN)
  #error "payload of message_t is too big"
#endif

#ifdef BOLT_CONF_MAX_MSG_LEN
  #if BOLT_CONF_MAX_MSG_LEN < MSG_PKT_LEN
    #error "BOLT max msg length is too small"
  #endif
#endif /* BOLT_CONF_MAX_MSG_LEN */

#if DPP_FW_BLOCK_SIZE > (DPP_MSG_PAYLOAD_LEN - 2 - DPP_FW_HDR_LEN)
  #error "DPP_FW_BLOCK_SIZE is too big"
#endif /* DPP_FW_BLOCK_SIZE */


#endif /* __DPP_MESSAGE_H__ */
