/*
 * led.h
 *
 *  Created on: Apr 24, 2018
 *      Author: marku
 */

#ifndef UTILS_LED_H_
#define UTILS_LED_H_


#define LED_PULSE_PERIOD_DEFAULT 2000

typedef enum
{
  LED_EVENT,
  LED_SYSTEM,
} led_s;


void leds_init();
void leds_deinit();
void leds_update();
void leds_sleep();
void leds_wakeup();

void led_set(led_s led, uint8_t intensity);
void led_on(led_s led);
void led_off(led_s led);
void led_toggle(led_s led);

void led_set_event_blink(uint32_t length, uint32_t period);

#endif /* UTILS_LED_H_ */
