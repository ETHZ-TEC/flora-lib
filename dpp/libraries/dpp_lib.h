/*
 * dpp_lib.h
 *
 *  Created on: Mar 27, 2020
 *      Author: rdaforno
 */

#ifndef __DPP_LIB_H__
#define __DPP_LIB_H__


/* function prototypes */

uint16_t crc16(const uint8_t* data, uint8_t num_bytes, uint16_t init_value);

uint32_t hexstr_to_uint32(const char* str);
uint32_t uint16_to_str(uint16_t val, char* out_buffer);


#endif /* __DPP_LIB_H__ */
