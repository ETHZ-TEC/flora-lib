/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 *
 * Authors: Reto Da Forno
 *          Tonio Gsell
 *          Akos Pasztor      
 */

/* 
 * event types specific to APP (application processor)
 *
 * 
 * typedef enum {
 *   ...
 *   DPP_EVENT_[device_name]  = [start_id],
 *   [your_defines]
 *   ...  
 * } dpp_event_type_t;
 * 
 * add a comma after each define!
 */

/* Label ------------------- | #No. | Description ----------------------------*/
     
EVENT_APP_START,            /* 0x01 | Application and OS started */

EVENT_RTC_INVALID,          /* 0x02 | RTC DateTime is invalid (e.g. after cold-start) */
EVENT_RTC_NOSYNC,           /* 0x03 | RTC hasn't been sync'd for CONF_RTC_SYNC_INTERVAL */
EVENT_RTC_SYNC,             /* 0x04 | RTC sync'd to GPS time */

EVENT_SCHEDULE_LPM_ENTER,   /* 0x05 | Enter Low-Power Mode */
EVENT_SCHEDULE_LPM_EXIT,    /* 0x06 | Exit Low-Power Mode. Value [0:normal_exit, 1:power_source_is_usb] */
EVENT_SCHEDULE_GPS_ON,      /* 0x07 | GPS is switched on. Value [0:always_on, 1:normal_on, 2:inverted_on] */
EVENT_SCHEDULE_GPS_OFF,     /* 0x08 | GPS is switched off. Value [0:always_off, 1:normal_off, 2:inverted_off] */

EVENT_GPS_CRC_ERR,          /* 0x09 | GPS UBX RAWX CRC Error */

EVENT_USB_NONE,             /* 0x0A | USB not initialized */
EVENT_USB_CDC,              /* 0x0B | USB CDC initialized */
EVENT_USB_CDC_GPS_TUNNEL,   /* 0x0C | USB CDC initialized, with GPS UART tunneling */
EVENT_USB_MSC,              /* 0x0D | USB MSC initialized */

EVENT_SD_CFG_DEF,           /* 0x0E | Default config created on SD card */
EVENT_SD_CFG_ERR,           /* 0x0F | Config file syntax error */

EVENT_SD_NOCARD,            /* 0x10 | No SD card */
EVENT_SD_MOUNT_ERR,         /* 0x11 | Failed to mount SD card */
EVENT_SD_CREATE_ERR,        /* 0x12 | Failed to create file on SD card */
EVENT_SD_OPEN_ERR,          /* 0x13 | Failed to open file on SD card */
EVENT_SD_WRITE_ERR,         /* 0x14 | Failed to write file on SD card */

EVENT_ADC_CURR_OOR,         /* 0x15 | Current Meas. Out Of Range. Value [measurement] */
EVENT_ADC_VBAT_OOR,         /* 0x16 | Battery VDC Meas. Out Of Range. Value [measurement] */
EVENT_ADC_VSYS_OOR,         /* 0x17 | System VDC Meas. Out Of Range. Value [measurement] */
EVENT_ADC_TCORE_OOR,        /* 0x18 | Core Temperature Meas. Out Of Range. Value [measurement] */
EVENT_TEMP_OOR,             /* 0x19 | SHT Temperature Meas. Out Of Range. Value [measurement] */
EVENT_HUMID_OOR,            /* 0x1A | SHT Humidity Meas. Out Of Range. Value [measurement] */
EVENT_STACK_90,             /* 0x1B | One of the tasks reached 90% stack watermark. Value [%] */
EVENT_HEAP_90,              /* 0x1C | RTOS Heap reached 90% watermark. Value [%] */
EVENT_SD_90,                /* 0x1D | SD card is 90% full. Value [free space in KBytes] */
EVENT_SD_FULL,              /* 0x1E | SD card is full (free space is less than 1KByte) */

EVENT_0x1F,                 /* 0x1F | - */

EVENT_RB_GNSS_OV,           /* 0x20 | GNSS RingBuffer overflow */
EVENT_RB_STATUS_OV,         /* 0x21 | Status RingBuffer overflow */
EVENT_RB_HEALTH_OV,         /* 0x22 | App Health RingBuffer overflow */
EVENT_RB_EVENT_OV,          /* 0x23 | Event RingBuffer overflow */

EVENT_APP_CMD_OK,           /* 0x24 | APP CMD received, executed */
EVENT_APP_CMD_ERR,          /* 0x25 | APP CMD error */
EVENT_APP_CMD_BUF_OV,       /* 0x26 | APP CMD buffer overflow */

EVENT_BOLT_RD_ERR,          /* 0x27 | BOLT Read Error */
EVENT_BOLT_WR_TIMEOUT,      /* 0x28 | BOLT Write Timeout */
EVENT_0x29,                 /* 0x29 | - */
EVENT_0x2A,                 /* 0x2A | - */
EVENT_0x2B,                 /* 0x2B | - */
EVENT_0x2C,                 /* 0x2C | - */
EVENT_0x2D,                 /* 0x2D | - */
EVENT_0x2E,                 /* 0x2E | - */
EVENT_0x2F,                 /* 0x2F | - */

EVENT_SW_RESET,             /* 0x30 | SW Reset */
EVENT_SELFTEST_START,       /* 0x31 | Self-Test started */
EVENT_SELFTEST_DONE,        /* 0x32 | Self-Test finished */
EVENT_IWDG_RESET,           /* 0x33 | IWDG Reset */
EVENT_STACK_TMRSVC_90,      /* 0x34 | Timer Daemon Task reached 90% stack watermark. Value [%] */
EVENT_STACK_IDLE_90,        /* 0x35 | Idle Task reached 90% stack watermark. Value [%] */
EVENT_0x36,                 /* 0x36 |  */
EVENT_0x37,                 /* 0x37 |  */
EVENT_0x38,                 /* 0x38 |  */
EVENT_0x39,                 /* 0x39 |  */
EVENT_0x3A,                 /* 0x3A |  */
EVENT_0x3B,                 /* 0x3B |  */
EVENT_0x3C,                 /* 0x3C |  */
EVENT_0x3D,                 /* 0x3D |  */
EVENT_0x3E,                 /* 0x3E |  */
EVENT_0x3F,                 /* 0x3F |  */

EVENT_0x40,                 /* 0x40 |  */











