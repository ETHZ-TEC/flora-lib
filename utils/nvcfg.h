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

#ifndef UTILS_NVCFG_H_
#define UTILS_NVCFG_H_


#ifndef NVCFG_ENABLE
#define NVCFG_ENABLE       0
#endif /* NVCFG_ENABLE */

/* be careful when changing the following parameters */
#define NVCFG_MEM_ADDR     (FLASH_MEM_START + (NVCFG_MEM_PAGE_ID * FLASH_MEM_PAGE_SIZE))  /* place at the end of the flash memory */
#define NVCFG_MEM_SIZE     FLASH_MEM_PAGE_SIZE                                            /* use one flash page (2kB) */
#define NVCFG_MEM_PAGE_ID  (FLASH_MEM_PAGES - 1)                                          /* use the last flash page */

#ifndef NVCFG_BLOCK_SIZE
#define NVCFG_BLOCK_SIZE   8         /* length without CRC */
#endif /* NVCFG_BLOCK_SIZE */

#define NVCFG_CRC_SIZE     2

#define NVCFG_BLOCK_SIZE_WITH_CRC   ((((NVCFG_BLOCK_SIZE + NVCFG_CRC_SIZE) + 7) / 8) * 8)     /* must be a multiple of 8 */

#define NVCFG_FLASH_EMPTY_VALUE     0xffffffff   /* on the STM32L4, flash values are all zeros after an erasure */

/* how many times the flash memory should be erased at max. (if unsuccessful) */
#ifndef NVCFG_ERASE_RETRY
#define NVCFG_ERASE_RETRY  3
#endif /* NVCFG_ERASE_RETRY */

#ifndef NVCFG_ONLY_SAVE_IF_CHANGED
#define NVCFG_ONLY_SAVE_IF_CHANGED  1       /* if set to 1, nvcfg_save() will only write the new config into the memory if it is different from the previous config */
#endif /* NVCFG_ONLY_SAVE_IF_CHANGED */

#if NVCFG_ERASE_RETRY == 0
#error "NVCFG_ERASE_RETRY can't be zero"
#endif

#if (NVCFG_BLOCK_SIZE + 2) > NVCFG_MEM_SIZE || NVCFG_BLOCK_SIZE & 0x1
#error "invalid NVCFG_BLOCK_SIZE (must be a multiple of 2 and smaller than NVCFG_MEM_SIZE)"
#endif

#if NVCFG_BLOCK_SIZE_WITH_CRC & 0x7
#error "invalid value for NVCFG_BLOCK_SIZE_WITH_CRC"
#endif


/**
 * @brief erase the whole memory segment
 * @note it is not recommended to call this function manually
 */
bool nvcfg_erase(void);

/**
 * @brief load the most recent data block from the predefined flash memory
 * location
 * @param out_data the buffer to hold the loaded data, must be at least
 * NVCFG_BLOCK_SIZE bytes long
 * @return true if successful (i.e. CRC ok), false otherwise
 * @note the crc checksum will not be copied into out_data!
 */
bool nvcfg_load(void* out_data);

/**
 * @brief store a block of data in the predefined flash memory segment
 * @param data data to save, must be exactly NVCFG_BLOCK_SIZE bytes long
 * @return true if successful, false otherwise
 */
bool nvcfg_save(const void* data);


#endif /* UTILS_NVCFG_H_ */

