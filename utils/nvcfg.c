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
 * non-volatile configuration storage (in flash memory)
 */

#include "flora_lib.h"


#if NVCFG_ENABLE


static bool nvcfg_program(const uint32_t dest_addr, const uint32_t src_addr)
{
  //LOG_VERBOSE("writing data to address 0x%x...", dest_addr);
  uint32_t i;
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  /* make sure error flags are cleared */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );

  for (i = 0; i < NVCFG_BLOCK_SIZE_WITH_CRC; i += 8) {    /* note: NVCFG_BLOCK_SIZE_WITH_CRC must be a multiple of 8! */
    /* double word is 64 bits */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest_addr + i, ADDR_VAL64(src_addr + i)) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  /* verify flash */
  for (i = 0; i < NVCFG_BLOCK_SIZE_WITH_CRC; i += 4) {   /* note: NVCFG_BLOCK_SIZE_WITH_CRC must be a multiple of 8! */
    if (ADDR_VAL32(dest_addr + i) != ADDR_VAL32(src_addr + i)) {
      LOG_WARNING("flash verify failed");
      return false;
    }
  }
  return true;
}


bool nvcfg_erase(void)
{
  FLASH_EraseInitTypeDef erase_config = {
      .TypeErase = FLASH_TYPEERASE_PAGES,
      .Banks = FLASH_BANK_1,
      .Page = NVCFG_MEM_PAGE_ID,
      .NbPages = 1,
  };
  uint32_t flash_erase_error = 0;

  LOG_VERBOSE("erasing flash page...");
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }
  /* make sure error flags are cleared */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_OPTVERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
  /* erase the page */
  bool retval = (HAL_FLASHEx_Erase(&erase_config, &flash_erase_error) == HAL_OK);
  HAL_FLASH_Lock();
  return retval;
}


bool nvcfg_load(void* out_data)
{
  uint32_t addr = NVCFG_MEM_ADDR;

  /* find the first empty block */
  while (((addr + NVCFG_BLOCK_SIZE_WITH_CRC) <= (NVCFG_MEM_ADDR + NVCFG_MEM_SIZE)) && ADDR_VAL16(addr) != (uint16_t)NVCFG_FLASH_EMPTY_VALUE) {
    addr += NVCFG_BLOCK_SIZE_WITH_CRC;
  }
  //LOG_VERBOSE("first empty block is 0x%x", addr);
  if (NVCFG_MEM_ADDR == addr) {   /* 1st block is empty? */
    return false; /* no data to load */
  }
  addr -= NVCFG_BLOCK_SIZE_WITH_CRC;  /* go back to previous block */

  /* check the CRC (first NVCFG_CRC_SIZE bytes of the block are the CRC!) */
  uint16_t crc_calc = crc16((uint8_t*)addr + NVCFG_CRC_SIZE, NVCFG_BLOCK_SIZE, 0);
  if (crc_calc == (uint16_t)NVCFG_FLASH_EMPTY_VALUE) {
    crc_calc = 1;             /* make sure CRC != NVCFG_FLASH_EMPTY_VALUE */
  }
  if (crc_calc == ADDR_VAL16(addr)) {
    memcpy(out_data, (uint8_t*)addr + NVCFG_CRC_SIZE, NVCFG_BLOCK_SIZE);
    return true;
  }
  LOG_WARNING("invalid CRC detected");
  return false;   /* invalid CRC */
}


bool nvcfg_save(const void* data)
{
  static uint8_t nvcfg_buffer[NVCFG_BLOCK_SIZE_WITH_CRC] = { 0 };
  uint32_t addr  = NVCFG_MEM_ADDR;
  uint32_t tries = NVCFG_ERASE_RETRY;

#if NVCFG_ONLY_SAVE_IF_CHANGED
  /* before continuing, check whether the data has changed */
  if (nvcfg_load(nvcfg_buffer)) {
    uint32_t i;
    for (i = 0; i < NVCFG_BLOCK_SIZE; i++) {
      if (nvcfg_buffer[i] != ((uint8_t*)data)[i]) {
        break;
      }
    }
    if (i == NVCFG_BLOCK_SIZE) {
      /* no difference detected -> skip save operation to reduce flash memory wear */
      return true;
    }
  }
#endif /* NVCFG_ONLY_SAVE_IF_CHANGED */

  /* find the first empty block in the reserved flash memory page */
  while ((addr + NVCFG_BLOCK_SIZE_WITH_CRC) <= (NVCFG_MEM_ADDR + NVCFG_MEM_SIZE) && ADDR_VAL16(addr) != (uint16_t)NVCFG_FLASH_EMPTY_VALUE) {
    addr += NVCFG_BLOCK_SIZE_WITH_CRC;
  }
  /* no empty slot found? */
  if ((addr + NVCFG_BLOCK_SIZE_WITH_CRC) > (NVCFG_MEM_ADDR + NVCFG_MEM_SIZE)) {
    /* erase the segment */
    while (tries && !nvcfg_erase()) {
      LOG_WARNING("erase failed");
      tries--;
    }
    /* basic wear leveling protection: wait 100ms, just in case nvcfg_save() is accidentally called in a loop */
    delay_us(100000);
    if (tries == 0) {
      return false; /* failed */
    }
    addr = NVCFG_MEM_ADDR;
  }

  /* copy the data and calculate the CRC */
  memcpy(nvcfg_buffer + NVCFG_CRC_SIZE, data, NVCFG_BLOCK_SIZE);
  uint16_t crc_calc = crc16((uint8_t*)data, NVCFG_BLOCK_SIZE, 0);
  if (crc_calc == (uint16_t)NVCFG_FLASH_EMPTY_VALUE) {
    crc_calc = 1;             /* make sure CRC != NVCFG_FLASH_EMPTY_VALUE */
  }
  ADDR_VAL16(nvcfg_buffer) = crc_calc;
  /* store the data */
  return nvcfg_program(addr, (uint32_t)nvcfg_buffer);
}

#endif /* NVCFG_ENABLE */
