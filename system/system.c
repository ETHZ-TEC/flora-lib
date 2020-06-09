/*
 * system.c
 *
 *  Created on: 25.04.2018
 *      Author: marku
 */

#include "flora_lib.h"


extern void SystemClock_Config(void);
extern bool lwb_cad_sleep_flag;
extern uint64_t hs_timer_scheduled_timestamp;

static volatile bool system_going_to_sleep = false;
static volatile bool system_initialized = false;


void system_boot(void)
{
  system_bootmode_check();
}


void system_init(void)
{
  /* init timers */
#ifdef HAL_RTC_MODULE_ENABLED
  rtc_init();
#endif /* HAL_RTC_MODULE_ENABLED */
  hs_timer_init();

  /* init pins */
  leds_init();
  gpio_init();
#if FLOCKLAB
  flocklab_init();
#endif /* FLOCKLAB */
#ifdef DEVKIT
  HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, GPIO_PIN_8); // RADIO_NSS
#endif
  HAL_PWREx_EnablePullUpPullDownConfig();

  /* init peripherals / drivers */
  uart_init();
#if BOLT_ENABLE
  bolt_init();
#endif /* BOLT_ENABLE */

  /* init radio and protocols */
  radio_init();
#if CLI_ENABLE
  cli_init();
#endif /* CLI_ENABLE */

  /* set seed for random generator */
  srand((unsigned int) hs_timer_get_current_timestamp());

  /* configure interrupts */
#ifndef DEVKIT
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);
#endif

  /* make sure PC13 (RF_DIO1) EXTI is disabled (only needed for wakeup from LPM) */
  HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

#if !CLI_ENABLE
  HAL_NVIC_DisableIRQ(USART1_IRQn);
#endif /* CLI_ENABLE */

  system_initialized = true;
}


void system_run(void)
{
  if (system_initialized)
  {
    while (true) {
      HAL_Delay(1);
      //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
    }
  }
}


void system_update(void)
{
  if (system_initialized)
  {
    radio_update_cli();
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

void system_wakeup(void)
{
  leds_wakeup();
}


void system_reset(void)
{
  radio_reset();
  HAL_NVIC_SystemReset();
}

void system_reset_into_bootloader(void)
{
  system_backup_get()->bootmode = SYSTEM_BOOT_BOOTLOADER;
  HAL_NVIC_SystemReset();
}

const char* system_get_reset_cause(uint32_t* out_reset_flag)
{
  static uint_fast8_t reset_flag = 0;

  if (!reset_flag) {
    reset_flag = RCC->CSR >> 24;
    RCC->CSR |= (1 << 23);          // clear flags
  }
  if (out_reset_flag) {
    *out_reset_flag = reset_flag;
  }
  if (reset_flag & 0x80) {
    return "LPM";   // illegal low-power mode entry
  }
  if (reset_flag & 0x40) {
    return "WWDG";  // window watchdog
  }
  if (reset_flag & 0x20) {
    return "IWDG";  // independent watchdog
  }
  if (reset_flag & 0x10) {
    return "SW";    // software triggered
  }
  if (reset_flag & 0x08) {
    return "BOR";   // brownout
  }
  if (reset_flag & 0x04) {
    return "nRST";  // reset pin
  }
  if (reset_flag & 0x02) {
    return "OBL";   // option byte loader
  }
  if (reset_flag & 0x01) {
    return "FWR";   // firewall reset
  }
  return "?";
}

