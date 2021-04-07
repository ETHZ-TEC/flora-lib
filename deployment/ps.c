/*
 * Copyright (c) 2020, Swiss Federal Institute of Technology (ETH Zurich).
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

/*
 * DPP message processing (permasense deployment)
 */

#include "flora_lib.h"


/* check if DPP message is valid */
bool ps_validate_msg(dpp_message_t* msg)
{
  uint16_t msg_len = DPP_MSG_LEN(msg);

  if (!msg ||
      msg_len > DPP_MSG_PKT_LEN ||
      DPP_MSG_GET_CRC16(msg) != crc16((uint8_t*)msg, msg_len - DPP_MSG_CRC_LEN, 0)) {
    return false;
  }
  return true;
}


/* compose a DPP message */
uint8_t ps_compose_msg(uint16_t recipient,
                       dpp_message_type_t type,
                       const uint8_t* data,
                       uint8_t len,
                       dpp_message_t* out_msg)
{
  /* separate sequence number for each interface */
  static uint16_t seq_no = 0;

  if (!out_msg) {
    return 0;
  }

  /* compose the message header */
  out_msg->header.device_id   = config.node_id;
  out_msg->header.type        = type;
  type &= ~DPP_MSG_TYPE_MIN;
  out_msg->header.payload_len = len;
  if (!len) {
    switch(type) {
    case DPP_MSG_TYPE_COM_HEALTH:
      out_msg->header.payload_len = sizeof(dpp_com_health_t); break;
    case DPP_MSG_TYPE_CMD:
      out_msg->header.payload_len = 6; break;  /* default is 6 bytes */
    case DPP_MSG_TYPE_EVENT:
      out_msg->header.payload_len = sizeof(dpp_event_t); break;
    case DPP_MSG_TYPE_NODE_INFO:
      out_msg->header.payload_len = sizeof(dpp_node_info_t); break;
    case DPP_MSG_TYPE_TIMESYNC:
      out_msg->header.payload_len = sizeof(dpp_timestamp_t); break;
    case DPP_MSG_TYPE_HEALTH_MIN:
      out_msg->header.payload_len = sizeof(dpp_health_min_t); break;
    case DPP_MSG_TYPE_ACK_COMMAND:
      if (data) {
        out_msg->header.payload_len = DPP_ACK_CMD_LEN((dpp_ack_cmd_t*)data);
      } else {
        out_msg->header.payload_len = DPP_ACK_CMD_LEN((dpp_ack_cmd_t*)&(out_msg->ack_cmd));
      }
      break;
    case DPP_MSG_TYPE_GEO_ACQ_AGGR:
    case DPP_MSG_TYPE_HEALTH_AGGR:
      if (data) {
        out_msg->header.payload_len = DPP_DATA_AGGR_LEN((dpp_data_aggr_t*)data);
      } else {
        out_msg->header.payload_len = DPP_DATA_AGGR_LEN((dpp_data_aggr_t*)&(out_msg->data_aggr));
      }
      break;
    case DPP_MSG_TYPE_GEOPHONE_ACQ:
      out_msg->header.payload_len = DPP_GEOPHONE_ACQ_LEN; break;
    default:
      break;
    }
  }

  /* check message length */
  if (out_msg->header.payload_len > DPP_MSG_PAYLOAD_LEN) {
    LOG_WARNING("invalid message length %u", len);
    return 0;
  }

  uint8_t msg_len;
  if ((out_msg->header.type & DPP_MSG_TYPE_MIN) == 0) {
    /* fill in the additional header fields */
    out_msg->header.target_id       = recipient;
    out_msg->header.seqnr           = seq_no++;
    out_msg->header.generation_time = get_time(0);
    /* copy the payload if valid */
    if (out_msg->header.payload_len && data) {
      memcpy(out_msg->payload, data, out_msg->header.payload_len);
    }
    /* calculate and append the CRC */
    msg_len = DPP_MSG_LEN(out_msg);
    uint16_t crc = crc16((uint8_t*)out_msg, msg_len - DPP_MSG_CRC_LEN, 0);
    DPP_MSG_SET_CRC16(out_msg, crc);

  } else {
    dpp_message_min_t* msg_min = (dpp_message_min_t*)out_msg;
    /* copy the payload if valid */
    if (msg_min->header.payload_len) {
      if (data) {
        memcpy(msg_min->payload, data, msg_min->header.payload_len);
      } else {
        /* note: src and dest are the same buffer, but offset by 10 bytes */
        memcpy(msg_min->payload, out_msg->payload, msg_min->header.payload_len);
      }
    }
    /* calculate and append the CRC */
    msg_len = DPP_MSG_MIN_LEN(msg_min);
    uint16_t crc = crc16((uint8_t*)msg_min, msg_len - DPP_MSG_CRC_LEN, 0);
    DPP_MSG_SET_CRC16(msg_min, crc);
  }

  return msg_len;
}


void ps_update_msg_crc(dpp_message_t* msg)
{
  if (msg) {
    uint32_t len = msg->header.payload_len;
    if (msg->header.type & DPP_MSG_TYPE_MIN) {
      len += DPP_MSG_MIN_HDR_LEN;
    } else {
      len += DPP_MSG_HDR_LEN;
    }
    uint16_t crc = crc16((uint8_t*)msg, len, 0);
    *((uint8_t*)msg + len) = crc & 0xff;
    *((uint8_t*)msg + len + 1) = (crc >> 8) & 0xff;
  }
}


void ps_compose_nodeinfo(dpp_message_t* out_msg, uint32_t cfg_field)
{
  if (!out_msg) {
    return;
  }

  memset((uint8_t*)&out_msg->node_info, 0, sizeof(dpp_node_info_t));
  out_msg->node_info.component_id = DPP_COMPONENT_ID_SX1262;
  out_msg->node_info.compiler_ver = (__GNUC__ * 1000000 + __GNUC_MINOR__ * 1000 + __GNUC_PATCHLEVEL__);
  out_msg->node_info.compile_date = BUILD_TIME;   // UNIX timestamp
#ifdef FW_VERSION
  out_msg->node_info.fw_ver       = (uint16_t)FW_VERSION;
#endif /* FW_VERSION */
  out_msg->node_info.rst_cnt      = config.rst_cnt;
  system_get_reset_cause(&out_msg->node_info.rst_flag);
#ifdef GIT_REV_INT
  out_msg->node_info.sw_rev_id    = GIT_REV_INT;
#endif /* GIT_REV_INT */
  out_msg->node_info.config       = cfg_field;
  memcpy(out_msg->node_info.compiler_desc, "GCC", MIN(4, strlen("GCC")));
#ifdef FW_NAME
  memcpy(out_msg->node_info.fw_name, FW_NAME, MIN(8, strlen(FW_NAME)));
#endif /* FW_NAME */
#ifdef STM32L433xx
  memcpy(out_msg->node_info.mcu_desc, "STM32L433CC", MIN(12, strlen("STM32L433CC")));
#endif /* STM32L433xx */
}
