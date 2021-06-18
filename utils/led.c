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

#include "flora_lib.h"

#ifndef DEVKIT
#define LED_SYSTEM_GPIO_Port LED_GREEN_GPIO_Port
#define LED_SYSTEM_Pin LED_GREEN_Pin
#define LED_EVENT_GPIO_Port LED_RED_GPIO_Port
#define LED_EVENT_Pin LED_RED_Pin

extern TIM_HandleTypeDef htim16;

static bool pulse_enabled;
static uint8_t pulse_counter;
static uint16_t pulse_period; // in ms
static bool pulse_direction; // increasing brightness

#else
#define LED_SYSTEM_GPIO_Port DEVKIT_LED_RX_GPIO_Port
#define LED_SYSTEM_Pin DEVKIT_LED_RX_Pin
#define LED_EVENT_GPIO_Port DEVKIT_LED_TX_GPIO_Port
#define LED_EVENT_Pin DEVKIT_LED_TX_Pin

static bool pulse_enabled;
static uint8_t pulse_counter;
static uint16_t pulse_period; // in ms
#endif

bool leds_initialized = false;

static uint16_t blink_counter;
static uint16_t blink_period; // in ms


static uint8_t gamma_correction_table[64] =
{
  0,1,2,3,4,5,6,6,7,8,
  9,10,11,12,13,15,16,17,19,20,
  22,24,26,28,30,32,34,37,39,42,
  45,48,51,54,57,61,65,68,72,77,
  81,86,91,96,101,107,112,118,125,131,
  138,145,152,160,168,176,185,194,203,213,
  223,234,244,255
};

inline static uint16_t gamma_correction(uint8_t intensity);


static void led_system_pulse_init();
static void led_system_pulse_update(uint32_t systick);
static void led_event_blink_init();
static void led_event_blink_update(uint32_t systick);


void leds_init()
{
  led_off(LED_SYSTEM);
  led_off(LED_EVENT);

#ifndef DEVKIT
  __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // init green LED PWM
#endif

  led_system_pulse_init();
  led_event_blink_init();

  leds_initialized = true;
}


void leds_deinit()
{
  led_off(LED_SYSTEM);
  led_off(LED_EVENT);

#ifndef DEVKIT
  __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, 0);
  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
#endif

  leds_initialized = false;
}


void leds_update()
{
  if (leds_initialized) {
    if (pulse_enabled) {
      led_system_pulse_update(hs_timer_get_current_timestamp() >> 13);
    }

    led_event_blink_update(hs_timer_get_current_timestamp() >> 13);
  }
}


void leds_sleep()
{
  if (leds_initialized)
  {
    blink_counter = 0;
    led_off(LED_EVENT);
    pulse_enabled = false;
    led_off(LED_SYSTEM);
  }
}


void leds_wakeup()
{
  if (leds_initialized)
  {
    pulse_counter = 0;
    pulse_enabled = true;
  }
}




void led_set(led_s led, uint8_t intensity)
{
  switch (led)
  {
    case LED_SYSTEM:
#ifndef DEVKIT
      __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, gamma_correction(intensity));
#else
      led_on(LED_SYSTEM);
#endif
      break;
    case LED_EVENT:
      if (intensity)
        led_on(LED_EVENT);
      else
        led_off(LED_EVENT);
      break;
    default:
      break;
  }
}

void led_on(led_s led)
{
  switch (led)
  {
    case LED_SYSTEM:
#ifndef DEVKIT
      __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, 0x03FF);
#else
      HAL_GPIO_WritePin(LED_SYSTEM_GPIO_Port, LED_SYSTEM_Pin, GPIO_PIN_SET);
#endif
      break;
    case LED_EVENT:
      HAL_GPIO_WritePin(LED_EVENT_GPIO_Port, LED_EVENT_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

void led_off(led_s led)
{
  switch (led)
  {
    case LED_SYSTEM:
#ifndef DEVKIT
      __HAL_TIM_SetCompare(&htim16, TIM_CHANNEL_1, 0);
#else
      HAL_GPIO_WritePin(LED_SYSTEM_GPIO_Port, LED_SYSTEM_Pin, GPIO_PIN_RESET);
#endif
      break;

    case LED_EVENT:
      HAL_GPIO_WritePin(LED_EVENT_GPIO_Port, LED_EVENT_Pin, GPIO_PIN_RESET);
      break;

    default:
      break;
  }
}

void led_toggle(led_s led)
{
  switch (led)
  {
#ifndef DEVKIT
    case LED_SYSTEM:
        HAL_GPIO_TogglePin(LED_SYSTEM_GPIO_Port, LED_SYSTEM_Pin);
        break;
#endif
    case LED_EVENT:
      HAL_GPIO_TogglePin(LED_EVENT_GPIO_Port, LED_EVENT_Pin);
      break;

    default:
      break;
  }
}


void led_set_event_blink(uint32_t length, uint32_t period)
{
  if (length) {
    blink_counter = length;
  }
  else {
    blink_counter = 5;
  }

  if (period) {
    blink_period = period;
  }
  else {
    blink_period = 100;
  }
}


void led_system_pulse_init()
{
  pulse_enabled = true;
  pulse_counter = 0;
  pulse_period = LED_PULSE_PERIOD_DEFAULT;
#ifndef DEVKIT
  pulse_direction = true;
#endif
}

void led_system_pulse_update(uint32_t systick)
{
#ifndef DEVKIT
  if (!(systick % (pulse_period / 256 / 2))) {
    led_set(LED_SYSTEM, pulse_counter % 256);
    if (pulse_direction) {
      pulse_counter++;
    }
    else {
      pulse_counter--;
    }

    if (pulse_counter == 0x00 || pulse_counter == 0xFF) {
      pulse_direction ^= true; // invert direction
    }
  }
#else
  if (((systick / (pulse_period / 2)) % 2) == 0)
  {
    led_on(LED_SYSTEM);
  }
  else
  {
    led_off(LED_SYSTEM);
  }

#endif
}


void led_event_blink_init()
{
  blink_counter = 0;
  blink_period = 100; // ms
}

void led_event_blink_update(uint32_t systick)
{
  if (blink_counter && ((systick / (blink_period / 2)) % 2) == 0)
  {
    led_on(LED_EVENT);
  }
  else
  {
    led_off(LED_EVENT);
  }

  if (systick % blink_period == 0)
  {
    if (blink_counter)
    {
      blink_counter--;
    }
  }
}


inline static uint16_t gamma_correction(uint8_t intensity)
{
  if (intensity == 0){
    return 0x0000;
  }
  else if (intensity == 255) {
    return 0x03FF;
  }
  else {
    uint8_t residual = intensity & 0x03;

    uint8_t lower = gamma_correction_table[intensity >> 2];
    uint8_t upper = gamma_correction_table[(intensity + 1) >> 2];

    uint16_t corrected = ((0x04 - residual) * (uint16_t) lower + residual * (uint16_t) upper);

    return corrected; // return 1024-bit gamma corrected intensity
  }
}
