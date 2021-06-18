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

/*
 * fw_ota.c
 *
 * over-the-air firmware updater
 */

#include "flora_lib.h"


#if FW_OTA_ENABLE


/* typedefs */
typedef enum
{
  FW_STATE_INVALID,     /* uninitialized */
  FW_STATE_INIT,        /* initialized (flash memory cleared) */
  FW_STATE_RECEIVING,   /* in the process of receiving the new fw data */
  FW_STATE_READY,       /* FW is ready for the update */
} fw_state_t;

typedef struct
{
  uint16_t    version;    /* FW version */
  uint16_t    block_cnt;  /* received blocks */
  uint32_t    data_len;   /* FW length in bytes */
  uint32_t    data_crc;   /* 32bit CRC over the FW data */
  fw_state_t  state;
  uint8_t     blocks[(FW_MAX_NUM_BLOCKS + 7) / 8];
} fw_info_t;

/* global handles */
#ifdef HAL_IWDG_MODULE_ENABLED
extern IWDG_HandleTypeDef hiwdg;
#endif /* HAL_IWDG_MODULE_ENABLED */

/* private variables */
static fw_info_t          fw_info = { 0 };
#if !FW_OTA_MASTER
static dpp_message_min_t  fw_msg_buffer;
static bool               pkt_rcvd = false;

/* prototypes of private functions */
static void fw_compose_msg(dpp_message_type_t type, uint8_t len);
static void fw_transmit_msg(void);
static void fw_receive_msg(void);
static bool fw_process_msg(dpp_message_t* msg);
static bool fw_verify_data(const dpp_fw_t* fw_pkt);
static bool fw_update(void);
static void fw_request_data(void);
static void fw_copy_data(void);
static void fw_tx_done(void);
static void fw_rx_done(uint8_t* payload, uint16_t size,  int16_t rssi, int8_t snr, bool crc_error);
static void fw_rx_timeout(bool crc_error);

#endif /* FW_OTA_MASTER */

static bool fw_store_data(const dpp_fw_t* fw_pkt);
static bool fw_erase_data(void);
static bool fw_program_data(const uint32_t dest_addr, uint32_t src_addr, uint32_t num_bytes);


/* functions */


#if !FW_OTA_MASTER

/*
 * enter FW update mode
 * notes:
 * - all unnecessary interrupts must be disabled before calling this function
 * - must not run in an interrupt context
 * - the function will return automatically if no new packets are received within FW_OTA_TIMEOUT
 */
void fw_ota(uint8_t radio_mod, uint8_t radio_band, int8_t radio_txpwr)
{
  uint64_t last_pkt_timestamp = lptimer_now();
  pkt_rcvd = false;

  LOG_INFO("starting OTA FW updater...");

  /* check if current FW info is valid */
  if (fw_info.data_len == 0 || fw_info.state == FW_STATE_INVALID) {
    memset(&fw_info, 0, sizeof(fw_info_t));
    fw_erase_data();
    fw_info.state = FW_STATE_INIT;
  }

  /* set radio config */
  radio_set_config_rxtx(true,
                        radio_band,
                        radio_modulations[radio_mod].datarate,
                        radio_txpwr,
                        radio_modulations[radio_mod].bandwidth,
                        radio_modulations[radio_mod].preambleLen,
                        radio_modulations[radio_mod].coderate,
                        0,
                        false,
                        0,
                        true);

  /* send an ACK to the base station */
  fw_msg_buffer.evt.type  = EVENT_SX1262_FW_OTA_STATUS;
  fw_msg_buffer.evt.value = 0;
  if (fw_info.data_len) {
    uint32_t total_block_cnt = (fw_info.data_len + DPP_FW_BLOCK_SIZE - 1) / DPP_FW_BLOCK_SIZE;
    fw_msg_buffer.evt.value  = fw_info.block_cnt * 100 / total_block_cnt;   // status in percent
    LOG_INFO("found FW version %05u (length: %luB    %u/%u blocks)", fw_info.version, fw_info.data_len, fw_info.block_cnt, total_block_cnt);
  }
  fw_compose_msg(DPP_MSG_TYPE_EVENT, DPP_EVENT_LEN);
  fw_transmit_msg();
  /* note: receive mode will be entered in TX done callback */

  /* main loop */
  while ((lptimer_now() - last_pkt_timestamp) < (FW_OTA_TIMEOUT * LPTIMER_SECOND)) {
    /* message in receive queue? -> process it */
    if (pkt_rcvd) {
      if (fw_process_msg((dpp_message_t*)&fw_msg_buffer)) {
        last_pkt_timestamp = lptimer_now();
      }
      pkt_rcvd = false;
    }
    HAL_Delay(10);
    /* flush print queue */
#if !LOG_PRINT_IMMEDIATELY
    log_flush();
#endif /* LOG_PRINT_IMMEDIATELY */
  }

  LOG_INFO("timeout");
}


/* composes a valid DPP message from fw_msg_buffer (type MIN with reduced header) */
static void fw_compose_msg(dpp_message_type_t type, uint8_t len)
{
  /* compose the message header */
  fw_msg_buffer.header.device_id   = FW_OTA_NODE_ID;
  fw_msg_buffer.header.type        = type;
  type &= ~DPP_MSG_TYPE_MIN;
  fw_msg_buffer.header.payload_len = len;

  /* calculate and append the CRC */
  uint32_t msg_buffer_len = DPP_MSG_MIN_LEN(&fw_msg_buffer);
  uint16_t crc = crc16((uint8_t*)&fw_msg_buffer, msg_buffer_len - DPP_MSG_CRC_LEN, 0);
  DPP_MSG_SET_CRC16(&fw_msg_buffer, crc);
}


static void fw_transmit_msg(void)
{
  radio_standby();
  radio_set_tx_callback(&fw_tx_done);
  radio_set_irq_mode(IRQ_MODE_TX);
  radio_transmit((uint8_t*)&fw_msg_buffer, DPP_MSG_LEN(&fw_msg_buffer));
  LOG_VERBOSE("sending packet...");
}


static void fw_receive_msg(void)
{
  LOG_VERBOSE("receive mode");
  radio_standby();          // make sure radio is in standby mode
  radio_set_rx_callback(&fw_rx_done);
  radio_set_timeout_callback(&fw_rx_timeout);
  radio_set_irq_mode(IRQ_MODE_RX_CRC);
  radio_receive(0);
}


/* process a received FW message */
static bool fw_process_msg(dpp_message_t* msg)
{
  /* make sure the message is valid */
  if (!msg ||
      (msg->header.type & ~DPP_MSG_TYPE_MIN) != DPP_MSG_TYPE_FW ||
      msg->header.payload_len < DPP_FW_HDR_LEN ||
      msg->header.payload_len > DPP_MSG_PAYLOAD_LEN) {
    LOG_WARNING("invalid message");
    return false;
  }

  /* check the CRC */
  uint16_t calc_crc,
           msg_crc;
  if (msg->header.type & DPP_MSG_TYPE_MIN) {
    calc_crc = crc16((uint8_t*)&fw_msg_buffer, DPP_MSG_MIN_LEN(&fw_msg_buffer) - DPP_MSG_CRC_LEN, 0);
    msg_crc  = DPP_MSG_GET_CRC16((dpp_message_min_t*)&fw_msg_buffer);
  } else {
    calc_crc = crc16((uint8_t*)&fw_msg_buffer, DPP_MSG_LEN(&fw_msg_buffer) - DPP_MSG_CRC_LEN, 0);
    msg_crc  = DPP_MSG_GET_CRC16(&fw_msg_buffer);
  }
  if (calc_crc != msg_crc) {
    LOG_WARNING("invalid CRC");
    return false;
  }

  dpp_fw_t* fw_pkt = &(msg->firmware);
  if (msg->header.type & DPP_MSG_TYPE_MIN) {
    fw_pkt = &(((dpp_message_min_t*)msg)->firmware);
  }

  if (fw_pkt->component_id != DPP_COMPONENT_ID_SX1262) {
    LOG_WARNING("invalid component ID");
    return false;
  }

#if !FW_OTA_ALLOW_DOWNGRADE
  /* only continue if FW version is newer than current version */
  if (fw_pkt->version <= FW_VERSION) {
    return true;
  }
#endif /* FW_OTA_ALLOW_DOWNGRADE */

  /* inspect the message type */
  switch (fw_pkt->type)
  {
  case DPP_FW_TYPE_DATA:
    /* check the block size - Payload: | fw_header | ofs (2 bytes) | data | */
    if (msg->header.payload_len != DPP_FW_DATA_LEN) {
      LOG_WARNING("invalid FW data block");
      return false;
    }
    return fw_store_data(fw_pkt);

  case DPP_FW_TYPE_CHECK:
    /* check the payload size - Payload: | fw_header | len (4 bytes) | CRC (4 bytes) | */
    if (msg->header.payload_len != DPP_FW_INFO_LEN) {
      LOG_WARNING("invalid FW packet of type %u", DPP_FW_TYPE_CHECK);
      return false;
    }
    LOG_INFO("checking FW file...");
    fw_verify_data(fw_pkt);
    return true;   /* always return success for this msg type */

  case DPP_FW_TYPE_UPDATE:
    radio_standby();
    LOG_INFO("updating firmware...");
    return fw_update();

  default:
    LOG_WARNING("unknown FW msg type");
    return false;
  }
  return true;
}


/* request the missing FW data blocks (pass the total # blocks) */
static void fw_request_data(void)
{
  uint16_t blkid,
           cnt = 0;
  uint32_t num_blocks = (fw_info.data_len + DPP_FW_BLOCK_SIZE - 1) / DPP_FW_BLOCK_SIZE;
  for (blkid = 0; blkid < num_blocks; blkid++) {
    /* is the bit for this block cleared? */
    if ((fw_info.blocks[blkid >> 3] & (1 << (blkid & 0x7))) == 0) {
      /* store this ID in the output buffer */
      fw_msg_buffer.firmware.req.block_ids[cnt++] = blkid;
      /* Require 2 bytes per block ID - Payload: | fw_header | num (2 bytes) | block_ids (2 bytes each) | */
      if (cnt >= DPP_FW_REQ_MAX_BLOCKS) {
        break;    /* no more space in the buffer for additional IDs */
      }
    }
  }
  if (cnt == 0) {
    return;       /* no need to send a block request */
  }
  /* send request */
  fw_msg_buffer.firmware.type         = DPP_FW_TYPE_DATAREQ;
  fw_msg_buffer.firmware.component_id = DPP_COMPONENT_ID_SX1262;
  fw_msg_buffer.firmware.version      = fw_info.version;
  fw_msg_buffer.firmware.req.num      = cnt;
  fw_compose_msg(DPP_MSG_TYPE_FW, DPP_FW_REQ_LEN(cnt));
  fw_transmit_msg();

  LOG_INFO("%u FW data blocks requested", cnt);
}


/* verify the FW data */
static bool fw_verify_data(const dpp_fw_t* fw_pkt)
{
  if (fw_info.state != FW_STATE_READY) {
    if (!fw_pkt->info.len || fw_pkt->info.len > FW_MAX_FILE_SIZE) {
      return false;
    }
    /* fill in the missing info */
    fw_info.data_crc = fw_pkt->info.crc;
    fw_info.data_len = fw_pkt->info.len;

    LOG_INFO("FW size: %uB  (CRC: 0x%x)", fw_info.data_len, fw_info.data_crc);

    /* first, check whether all blocks are in the memory */
    uint32_t num_blocks = (fw_info.data_len + DPP_FW_BLOCK_SIZE - 1) / DPP_FW_BLOCK_SIZE;
    if (fw_info.block_cnt < num_blocks) {
      fw_request_data();
      return false;
    }
    /* data is in place, verify checksum */
    uint32_t crc  = crc32((uint8_t*)FW_START_ADDR, fw_info.data_len, 0);
    if (fw_info.data_crc != crc) {
      LOG_ERROR("invalid FW data CRC (%lx vs %lx)", crc, fw_info.data_crc);
      fw_info.state = FW_STATE_INVALID;     // drop all the collected FW data
      return false;
    }
    /* checksum is ok */
    fw_info.state = FW_STATE_READY;
    LOG_INFO("FW data verified");
  }
  /* notify */
  fw_msg_buffer.firmware.type         = DPP_FW_TYPE_READY;
  fw_msg_buffer.firmware.component_id = DPP_COMPONENT_ID_SX1262;
  fw_msg_buffer.firmware.version      = fw_info.version;
  fw_compose_msg(DPP_MSG_TYPE_FW, DPP_FW_HDR_LEN);
  fw_transmit_msg();

  return true;
}


/* initiate the firmware update (for host nodes: node's own firmware) */
static bool fw_update(void)
{
  if (FW_STATE_READY != fw_info.state) {
    return false;
  }
  /* check CRC one more time */
  uint32_t crc  = crc32((uint8_t*)FW_START_ADDR, fw_info.data_len, 0);
  if (fw_info.data_crc != crc) {
    LOG_ERROR("invalid FW data CRC (%lx vs %lx)", crc, fw_info.data_crc);
    fw_info.state = FW_STATE_INVALID;     // drop all the collected FW data
    return false;
  }
  /* point of no return: this will potentially overwrite parts of the heap and/or stack, the only way to recover in case of an error is an MCU reset */
  PIN_SET(LED_RED);
  /* reset the watchdog and disable all interrupts */
#ifdef HAL_IWDG_MODULE_ENABLED
  HAL_IWDG_Refresh(&hiwdg);
#endif /* HAL_IWDG_MODULE_ENABLED */
  __disable_irq();
  __DSB();
  __ISB();

  /* copy function to SRAM (as an alternative, let the compiler place the function in SRAM with the attribute __RAM_FUNC) */
  //const uint32_t max_func_size = 512;
  //uint8_t func_buffer[max_func_size];
  //memcpy(func_buffer, (void*)fw_copy_data, max_func_size);
  //void (*execute_from_sram)(void) = (void*)(&func_buffer[1]);   // address needs to be odd to tell the processor to keep using thumb instruction mode
  //execute_from_sram();

  fw_copy_data();

  return true;
}

//__attribute__ ((long_call, section (".code_in_ram")))
static __NOINLINE __RAM_FUNC void fw_copy_data(void)
{
  uint32_t num_attempts = 3;

  /* make sure this function does not contain any calls or access any variables on the heap
   * -> inline all functions and use local constants/defines instead of global variables */

  /* unlock the flash memory */
  WRITE_REG(FLASH->KEYR, FLASH_KEY1);
  WRITE_REG(FLASH->KEYR, FLASH_KEY2);
  if (READ_BIT(FLASH->CR, FLASH_CR_LOCK) == 0U) {

    /* disable instruction and data cache  */
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_DISABLE();

    while (num_attempts) {
      /* kick the watchdog */
  #ifdef HAL_IWDG_MODULE_ENABLED
      __HAL_IWDG_RELOAD_COUNTER(&hiwdg);
  #endif /* HAL_IWDG_MODULE_ENABLED */

      /* clear error flags */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

      /* wait until flash ready */
      while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

      /* erase pages */
      uint32_t page_idx;
      for (page_idx = 0; page_idx < FLASH_MEM_PAGES / 2; page_idx++) {
        MODIFY_REG(FLASH->CR, FLASH_CR_PNB, ((page_idx & 0xFFU) << FLASH_CR_PNB_Pos));
        SET_BIT(FLASH->CR, FLASH_CR_PER);
        SET_BIT(FLASH->CR, FLASH_CR_STRT);

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

        CLEAR_BIT(FLASH->CR, (FLASH_CR_PER | FLASH_CR_PNB));
      }

      uint32_t addr;
      /* program the firmware */
      for (addr = FLASH_MEM_START; addr < FLASH_MEM_START + fw_info.data_len; addr += 8) {

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

        SET_BIT(FLASH->CR, FLASH_CR_PG);
        *(__IO uint32_t*)addr = ADDR_VAL32(addr + (FLASH_MEM_SIZE / 2));
        __ISB();
        *(__IO uint32_t*)(addr + 4U) = ADDR_VAL32(addr + (FLASH_MEM_SIZE / 2) + 4U);

        while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
        if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_EOP)) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);

        CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
      }

      /* verify the firmware */
      for (addr = FLASH_MEM_START; addr < FLASH_MEM_START + fw_info.data_len; addr += 4) {
        if (ADDR_VAL32(addr) != ADDR_VAL32(addr + (FLASH_MEM_SIZE / 2))) {
          break;
        }
      }
      if (addr >= (FLASH_MEM_START + fw_info.data_len)) {
        break;    // success
      }
      num_attempts--;
    }

    /* lock flash */
    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    if (num_attempts == 0) {
      while (1) {
        PIN_SET(LED_RED);
        volatile uint32_t loop = 200000;
        while (loop) loop--;
        PIN_CLR(LED_RED);
        loop = 200000;
        while (loop) loop--;
      }
    }
  }

  /* reset the MCU */
  NVIC_SystemReset();
}


static void fw_tx_done(void)
{
  fw_receive_msg();   /* enter receive mode */
}


static void fw_rx_done(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr, bool crc_error)
{
  if (crc_error) {
    LOG_WARNING("CRC error");
  } else {
    LOG_VERBOSE("RX done (len: %d  RSSI: %d  SNR: %d)", size, rssi, snr);
    if (!pkt_rcvd) {
      if (size <= DPP_MSG_PKT_LEN) {
        memcpy((uint8_t*)&fw_msg_buffer, payload, size);
        pkt_rcvd = true;
      } else {
        LOG_WARNING("message of invalid size (%u) received", size);
      }
    } else {
      LOG_WARNING("a message is already pending");
    }
  }
  fw_receive_msg();     /* restart receive mode */
}


static void fw_rx_timeout(bool crc_error)
{
  if (crc_error) {
    LOG_VERBOSE("CRC error");
  } else {
    LOG_VERBOSE("RX timeout");
  }
  fw_receive_msg();   /* enter receive mode */
}

#else /* FW_OTA_MASTER */


// for the FW master only: store a FW data block in flash memory for dissemination at a later point in time (note: size of one data chunk must be DPP_FW_BLOCK_SIZE)
bool fw_ota_add_data(const dpp_fw_t* fw_data)
{
  if ((fw_data->type == DPP_FW_TYPE_DATA) && (fw_data->component_id == DPP_COMPONENT_ID_SX1262)) {
    return fw_store_data(fw_data);
  }
  return false;
}


bool fw_ota_get_info(dpp_fw_t* out_fw_info)
{
  if (out_fw_info && (fw_info.state == FW_STATE_READY)) {
    out_fw_info->type         = DPP_FW_TYPE_CHECK;
    out_fw_info->component_id = DPP_COMPONENT_ID_SX1262;
    out_fw_info->version      = fw_info.version;
    out_fw_info->info.len     = fw_info.data_len;
    out_fw_info->info.crc     = fw_info.data_crc;
    return true;
  }
  return false;
}


bool fw_ota_verify(uint16_t version, uint32_t len, uint32_t crc)
{
  fw_info.version  = version;
  fw_info.data_len = len;
  fw_info.data_crc = crc32((uint8_t*)FW_START_ADDR, fw_info.data_len, 0);
  /* verify checksum */
  if (fw_info.data_crc != crc) {
    return false;
  }
  fw_info.state = FW_STATE_READY;
  return true;
}


bool fw_ota_get_data(uint32_t block_id, dpp_fw_t* out_fw_data)
{
  if (out_fw_data && (fw_info.state == FW_STATE_READY) && (block_id < FW_MAX_NUM_BLOCKS)) {
    out_fw_data->type         = DPP_FW_TYPE_DATA;
    out_fw_data->component_id = DPP_COMPONENT_ID_SX1262;
    out_fw_data->version      = fw_info.version;
    out_fw_data->data.ofs     = block_id;
    memcpy(out_fw_data->data.data, (uint8_t*)((uint32_t)FW_START_ADDR + (block_id * DPP_FW_BLOCK_SIZE)), DPP_FW_BLOCK_SIZE);
    return true;
  }
  return false;
}


// write an Intel hex image into the flash memory; length is the firmware data length in bytes and must be a multiple of 8
bool fw_ota_set_image(const char* image, uint32_t len, uint16_t version)
{
  if (!image && !len) {
    return false;       // either length or image have to be provided
  }
  if (image) {
    len = strlen(image);
    if (!fw_erase_data()) {
      fw_info.state = FW_STATE_INVALID;
      LOG_WARNING("failed to erase flash memory");
      return false;
    }
    uint32_t i;
    uint8_t  tmp[8];
    // process and image file 16 characters (8 bytes) at a time
    for (i = 0; i < len; i += 16) {
      uint32_t cnt  = 0;
      uint8_t  byte = 0;
      while (cnt < 16) {
        char ch = image[i + cnt];
        if (ch >= 'A' && ch <= 'F') {
          byte += ch - 'A' + 10;
        } else if (ch >= 'a' && ch <= 'f') {
          byte += ch - 'a' + 10;
        } else if (ch >= '0' && ch <= '9') {
          byte += ch - '0';
        } else {
          LOG_WARNING("invalid character '%c' detected", ch);
          return false;
        }
        if (cnt & 1) {
          tmp[cnt / 2] = byte;
          byte         = 0;
        } else {
          byte <<= 4;
        }
        cnt++;
      }
      // program 8 bytes
      if (!fw_program_data(FW_START_ADDR + i / 2, (uint32_t)tmp, 8)) {
        LOG_WARNING("failed to program data to flash");
        return false;
      }
    }
    len = len / 2;
  } // else: assume the image is valid and skip the flash programming process

  fw_info.version  = version;
  fw_info.data_len = len;
  fw_info.data_crc = crc32((uint8_t*)FW_START_ADDR, fw_info.data_len, 0);
  fw_info.state    = FW_STATE_READY;
  LOG_INFO("FW image stored in memory (version: %05u   len: %luB   CRC: 0x%x)", fw_info.version, fw_info.data_len, fw_info.data_crc);

  return true;
}

#endif /* FW_OTA_MASTER */


/* pass the function a FW data block of length DPP_FW_BLOCK_SIZE */
static bool fw_store_data(const dpp_fw_t* fw_pkt)
{
  /* check block offset */
  if (fw_pkt->data.ofs >= FW_MAX_NUM_BLOCKS) {
    return false;
  }
  /* first packet of this FW version? */
  if (fw_info.state <= FW_STATE_INIT || fw_pkt->version != fw_info.version) {
    /* erase memory if new FW version or invalid state */
    if (fw_info.state == FW_STATE_INVALID || fw_pkt->version != fw_info.version) {
      if (!fw_erase_data()) {
        fw_info.state = FW_STATE_INVALID;
        LOG_WARNING("failed to erase firmware in flash memory");
        return false;
      }
    }
    /* initialize the meta data (FW info) */
    memset(&fw_info, 0, sizeof(fw_info_t));
    fw_info.state   = FW_STATE_RECEIVING;
    fw_info.version = fw_pkt->version;
    LOG_INFO("new FW version %05u detected", fw_info.version);
  }
  if (fw_info.state == FW_STATE_READY) {
    return true; /* no need to continue, we already have all data */
  }
  /* check whether we already have this block */
  uint16_t blkid   = fw_pkt->data.ofs;                    /* offset = block ID */
  uint16_t bitmask = (1 << (blkid & 0x7));
  if (blkid >= FW_MAX_NUM_BLOCKS) {
    LOG_WARNING("invalid block ID %u", blkid);
    return false;
  }
  if (!(fw_info.blocks[blkid >> 3] & bitmask)) {
    /* store the received data */
    if (!fw_program_data(FW_START_ADDR + (uint32_t)blkid * DPP_FW_BLOCK_SIZE, (uint32_t)(fw_pkt->data.data), DPP_FW_BLOCK_SIZE)) {
      LOG_WARNING("failed to write FW data to flash memory");
      return false;
    }
    /* mark the block as 'existing' */
    fw_info.blocks[blkid >> 3] |= bitmask;
    fw_info.block_cnt++;

    LOG_INFO("FW block %u processed", fw_pkt->data.ofs);
  }
  return true;
}


/* write one block of data into the flash memory */
static bool fw_program_data(const uint32_t dest_addr, uint32_t src_addr, uint32_t num_bytes)
{
  /* must be a multiple of 8 */
  if (!num_bytes ||
      (num_bytes & 0x7) ||
      dest_addr < FW_START_ADDR ||
      (dest_addr + num_bytes) > (FW_START_ADDR + FW_MAX_FILE_SIZE)) {
    return false;
  }
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  /* make sure error flags are cleared */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

  uint32_t i;
  for (i = 0; i < num_bytes; i += 8) {
    /* double word is 64 bits */
    uint64_t tmp;
    memcpy(&tmp, (uint8_t*)(src_addr + i), 8);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr + i, tmp) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  /* verify flash */
  for (i = 0; i < num_bytes; i += 4) {
    if (ADDR_VAL32(dest_addr + i) != ADDR_VAL32(src_addr + i)) {
      return false;
    }
  }

  return true;
}


/* erase the flash memory */
static bool fw_erase_data(void)
{
  FLASH_EraseInitTypeDef erase_config = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = FLASH_BANK_1,
      .Page = FLASH_MEM_PAGES / 2,          /* start page index */
      .NbPages = FLASH_MEM_PAGES / 2 - 1,   /* number of pages to erase (all but the last) */
  };
  uint32_t flash_erase_error = 0;

  LOG_VERBOSE("erasing flash memory...");
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  /* make sure error flags are cleared */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
  /* erase the page */
  bool retval = (HAL_FLASHEx_Erase(&erase_config, &flash_erase_error) == HAL_OK);
  HAL_FLASH_Lock();

  LOG_VERBOSE("data erased");

  return retval;
}

#endif /* FW_OTA_ENABLE */

