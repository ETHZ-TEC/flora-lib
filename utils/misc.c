/*
 * misc.c
 *
 *  Created on: Aug 22, 2019
 *      Author: rdaforno
 */

#include "flora_lib.h"


void delay(volatile uint32_t loop_passes)
{
  while (loop_passes) loop_passes--;
}

void delay_us(volatile uint32_t us)
{
  /* note: needs to be adjusted if the CPU clock is changed! */
  while (us) {
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    asm("NOP");
    us--;
  }
}

bool swo_println(const char* str)
{
  while (*str)
  {
    ITM_SendChar(*str);
    str++;
  }
  ITM_SendChar('\r');
  ITM_SendChar('\n');

  return true;
}

bool swo_print(const char* str)
{
  while (*str)
  {
    ITM_SendChar(*str);
    str++;
  }
  return true;
}
