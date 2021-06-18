# Library Overview

The Flora library provides drivers and helper functions for the DPP2 LoRa comboard with the STM32L433CC MCU and the Semtech SX1262 radio.

## Folders

| Name | Description |
|----|----|
|`bolt/`         | Contains Bolt files and its functions |
|`cli/`          | Command line interface |
|`deployment/`   | Deployment-specific code |
|`dpp/`          | Submodule, contains message definitions and some library functions (CRC, data structures, ...). |
|`flocklab/`     | Defines all FlockLab pins, provides functions to set and clear the FlockLab LEDs |
|`protocol/`     | Wireless protocols |
|`radio/`        | Radio driver and helper functions for the Semtech SX1262 radio. |
|`system/`       | General system initialization and helper functions (UART, GPIO, ...). |
|`time/`         | Timers (LPTIM, TIM, RTC) |
|`utils/`        | Misc helper functions / utilities (LEDs, logging, etc.) |
