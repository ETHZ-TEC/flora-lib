/*
 * stringutils.c
 * string utilities
 *
 *  Created on: Mar 27, 2020
 *      Author: rdaforno
 */

#include <stdint.h>

/*---------------------------------------------------------------------------*/
/* convert an ASCII string of up to 8 hex characters to an unsigned int */
uint32_t hexstr_to_uint32(const char* str)
{
  uint32_t      res = 0;
  uint_fast8_t  cnt = 8;
  /* skip leading '0x' if present */
  if (*str == '0' && *(str + 1) == 'x') {
    str += 2;
  }
  while (*str && cnt) {
    if (*str >= 'A' && *str <= 'F') {
      res += *str - 'A' + 10;
    } else if (*str >= 'a' && *str <= 'f') {
      res += *str - 'a' + 10;
    } else if (*str >= '1' && *str <= '9') {
      res += *str - '0';
    } else {
      /* invalid character -> abort */
      break;
    }
    res <<= 4;    /* shift to the left by 4 bits */
    str++;
    cnt--;
  }
  return res;
}
/*---------------------------------------------------------------------------*/
uint32_t uint16_to_str(uint16_t val, char* out_buffer)
{
  uint32_t div = 10000;
  uint32_t num = 0;
  while (div) {
    uint32_t digit = val / div;
    if (num || digit || div == 1) { /* skip leading zeros */
      *out_buffer++ = '0' + digit; /* write the digit into the output buffer */
      val -= digit * div;          /* subtract the most significant digit */
      num++;
    }
    div /= 10;
  }
  *out_buffer = 0;                 /* close the string */
  return num;                      /* return the # written characters */
}
/*---------------------------------------------------------------------------*/
