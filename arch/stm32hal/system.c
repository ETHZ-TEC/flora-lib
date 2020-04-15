/*
 * system.c
 *
 *  Created on: 25.04.2018
 *      Author: marku
 */

#include <stdlib.h>
#include <stdbool.h>

#include "main.h"

#include "radio/radio_helpers.h"
#include "system/system.h"

#include "system/system_backup_registers.h"
#include "time/rtc.h"

#include "system/system_boot.h"
#include "time/hs_timer.h"
#include "config/config.h"
#include "led/led.h"
#include "cli/uart.h"
#include "cli/cli.h"
#include "protocol/protocol.h"
#include "protocol/gloria/gloria.h"
#include "time/rtc.h"

#include "radio/flora_radio.h"

#include "stm32l4xx_hal.h"

extern void SystemClock_Config(void);
extern bool lwb_cad_sleep_flag;
extern uint64_t hs_timer_scheduled_timestamp;

static volatile bool system_going_to_sleep = false;
static volatile bool system_initialized = false;

void system_boot() {
  system_bootmode_check();
}

void system_init()
{
#ifndef DEVKIT
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, GPIO_PIN_12); // RADIO_NSS
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, GPIO_PIN_1); // BOLT_REQ
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, GPIO_PIN_2); // BOLT_MODE
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, GPIO_PIN_5); // BOLT_SCK
  HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_A, GPIO_PIN_7); // BOLT_MOSI
#else
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_8); // RADIO_NSS
#endif
  HAL_PWREx_EnablePullUpPullDownConfig();

#ifdef HAL_RTC_MODULE_ENABLED
  rtc_init();
#endif /* HAL_RTC_MODULE_ENABLED */
  hs_timer_init();
  config_init();
  radio_init();
  uart_init();
#if CLI_ENABLE
  cli_init();
#endif /* CLI_ENABLE */
  protocol_init();

#if !USE_SWO
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  #if BASEBOARD || !FLOCKLAB
    HAL_GPIO_WritePin(COM_GPIO2_GPIO_Port, COM_GPIO2_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = COM_GPIO2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // default set to pull-up because on BaseBoard COM_GPIO2 is connected to Colibri_EN
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  #else /* BASEBOARD || !FLOCKLAB */
    // configure as input
    HAL_GPIO_WritePin(COM_GPIO2_GPIO_Port, COM_GPIO2_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = COM_GPIO2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // default set to pull-up because on BaseBoard COM_GPIO2 is connected to Colibri_EN
    // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  #endif /* BASEBOARD || !FLOCKLAB */
  HAL_GPIO_Init(COM_GPIO2_GPIO_Port, &GPIO_InitStruct);
#endif /* USE_SWO */

#if FLOCKLAB
  flocklab_init();
#else /* FLOCKLAB */
  leds_init();
#endif /* FLOCKLAB */

  srand((unsigned int) hs_timer_get_current_timestamp());

#ifndef DEVKIT
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);
#endif

  system_initialized = true;
}


void system_run() {
  if (system_initialized)
  {
    while (true) {
      HAL_Delay(1);
      //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
    }
  }
}


void system_update(){
  if (system_initialized)
  {
    radio_update();
    leds_update();
    cli_update();
    gloria_update();
  }
}


void system_sleep(bool deep)
{
  if (deep) {
    radio_sleep(false);
    leds_sleep();

    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
    HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);

    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_DISABLE();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_OFF);

    HAL_PWREx_EnterSHUTDOWNMode();
  }
  else {
    radio_sleep(false);
    leds_sleep();

    // Todo Implement standby mode with recovery (requires to define a backup-register
    // for process counter and stack similar to bootloader, jump to stack, and customization
    // of the startup code, to not overwrite static variables)

    //HAL_PWREx_EnableSRAM2ContentRetention();
    //HAL_PWR_EnterSTANDBYMode();

    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFE);
    hs_timer_init();
    leds_wakeup();
    SystemClock_Config();
  }
}

void system_wakeup()
{
  leds_wakeup();
}


void system_reset()
{
  radio_reset();
  HAL_NVIC_SystemReset();
}

void system_reset_into_bootloader()
{
  system_backup_get()->bootmode = SYSTEM_BOOT_BOOTLOADER;
  HAL_NVIC_SystemReset();
}
