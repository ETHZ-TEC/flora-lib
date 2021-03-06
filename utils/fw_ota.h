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
 * fw_ota.h
 *
 * over-the-air firmware updater
 */

#ifndef UTILS_FW_OTA_H_
#define UTILS_FW_OTA_H_


#ifndef FW_OTA_ENABLE
#define FW_OTA_ENABLE           0
#endif /* FW_OTA_ENABLE */

#ifndef FW_OTA_MASTER
#define FW_OTA_MASTER           0
#endif /* FW_OTA_MASTER */

#ifndef FW_OTA_NODE_ID
#define FW_OTA_NODE_ID          NODE_ID
#endif /* FW_OTA_NODE_ID */

#ifndef FW_OTA_ALLOW_DOWNGRADE
#define FW_OTA_ALLOW_DOWNGRADE  1         /* if not enabled, the current FW version must be specified with FW_VERSION */
#endif /* FW_OTA_ALLOW_DOWNGRADE */

#ifndef FW_OTA_TIMEOUT
#define FW_OTA_TIMEOUT          300       /* timeout in seconds */
#endif /* FW_OTA_TIMEOUT */


#define FW_START_ADDR           (FLASH_MEM_START + FLASH_MEM_SIZE / 2)        /* store the FW in the 2nd half of the flash memory */
#define FW_MAX_FILE_SIZE        (FLASH_MEM_SIZE / 2 - FLASH_MEM_PAGE_SIZE)    /* use half of the flash memory minus 1 page (for nvcfg) */
#define FW_MAX_NUM_BLOCKS       ((FW_MAX_FILE_SIZE + DPP_FW_BLOCK_SIZE - 1) / DPP_FW_BLOCK_SIZE)


#if FW_OTA_MASTER

bool      fw_ota_add_data(const dpp_fw_t* fw_data);                       /* add a FW data block to flash memory, size of one data chunk must be DPP_FW_BLOCK_SIZE (master only) */
bool      fw_ota_verify(uint16_t version, uint32_t len, uint32_t crc);    /* verify the FW in flash (master only) */
bool      fw_ota_get_info(dpp_fw_t* out_fw_info);                         /* generate FW check message containing the FW info (master only) */
bool      fw_ota_get_data(uint32_t block_id, dpp_fw_t* out_fw_data);      /* loads the requested FW data block from the flash memory into the output buffer (master only) */
bool      fw_ota_set_image(const char* image, uint32_t len, uint16_t version);  /* write firmware image (provided as hex string) into the flash memory */

#else /* FW_OTA_MASTER */

void      fw_ota(uint8_t radio_mod, uint8_t radio_band, int8_t radio_txpwr);    /* enter FW update mode (source node only) */

#endif /* FW_OTA_MASTER */

#endif /* UTILS_FW_OTA_H_ */
