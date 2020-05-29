# Library Overview

This file gives a short overview of all existing libraries in this folder and a short description of each of them.

## Folders

| Name | Description |
|----|----|
|`arch/`         ||
|`bolt/`         | Contains Bolt files and its functions |
|`cli/`          ||
|`config/`       ||
|`dpp/`          | Submodule, contains message/events definitions and some library functions (CRC, data structures, ...). [Common Git](https://gitlab.ethz.ch/tec/research/dpp/software/common) |
|`flocklab/`     | Defines all FlockLab pins, provides functions to set and clear the FlockLab LEDs |
|`protocol/`     ||
|`radio/`        | Radio helper functions for the Semtech SX1262 Radio. Most important file: `radio_helpers.h`. There are all helper functions defined. |
|`system/`       ||
|`time/`         | Provides functions for HS Timer, LPTIM and RTC |
|`utils/`        | Misc helper functions / utilities (LEDs, logging, etc.) |
