/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
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

/* Label --------------------------- | #No. | Description ---------------------*/

EVENT_GEOPHONE_START,               /* 0x01 | Application and OS started */
EVENT_GEOPHONE_SELFTEST,            /* 0x02 | Self-Test. Value: result of self-test */
EVENT_GEOPHONE_TSYNC_TIMEOUT,       /* 0x03 | TimeSync request timeout */
EVENT_GEOPHONE_BOLT_MSG_TIMESTAMP,  /* 0x04 | Timestamp received */
EVENT_GEOPHONE_BOLT_READ_ERROR,     /* 0x05 | BOLT read error. Value: error source */
EVENT_GEOPHONE_BOLT_WRITE_ERROR,    /* 0x06 | BOLT write error. Value: error source */
EVENT_GEOPHONE_BOLT_INVALID,        /* 0x07 | BOLT invalid request from application */
EVENT_GEOPHONE_LOG_ERROR,           /* 0x08 | LOG error. Value: error source */
EVENT_GEOPHONE_ACQ_ERR_TIMEOUT,     /* 0x09 | ACQ sampling error: semaphore timeout */
EVENT_GEOPHONE_CONFIG_WR_ERR,       /* 0x0A | Config write to SD error */
EVENT_GEOPHONE_CONFIG_RD_ERR,       /* 0x0B | Config read from SD error */
EVENT_GEOPHONE_CONFIG_SYNTAX_ERR,   /* 0x0C | Config syntax error */
EVENT_GEOPHONE_NA_0x0D,             /* 0x0D | */
EVENT_GEOPHONE_NA_0x0E,             /* 0x0E | */
EVENT_GEOPHONE_NA_0x0F,             /* 0x0F | */

EVENT_GEOPHONE_CMD_RESET,           /* 0x10 | APP CMD: reset */
EVENT_GEOPHONE_CMD_SELFTEST,        /* 0x11 | APP CMD: self-test */
EVENT_GEOPHONE_CMD_EXTTRG,          /* 0x12 | APP CMD: external trigger */
EVENT_GEOPHONE_CMD_GAIN1,           /* 0x13 | APP CMD: triggering gain stage 1 */
EVENT_GEOPHONE_CMD_GAIN2,           /* 0x14 | APP CMD: triggering gain stage 2 */
EVENT_GEOPHONE_CMD_TRG_TH_POS,      /* 0x15 | APP CMD: positive trigger threshold value*/
EVENT_GEOPHONE_CMD_TRG_TH_NEG,      /* 0x16 | APP CMD: negative trigger threshold value*/
EVENT_GEOPHONE_CMD_POSTTRG,         /* 0x17 | APP CMD: set post-triggering. Value: interval [s] */
EVENT_GEOPHONE_CMD_TIMEOUT,         /* 0x18 | APP CMD: set sampling timeout. Value: interval [s] */
EVENT_GEOPHONE_CMD_PGA,             /* 0x19 | APP CMD: set ADC PGA. Value: gain */
EVENT_GEOPHONE_CMD_FORMAT,          /* 0x1A | APP CMD: set ADC data format. Value: data-format */
EVENT_GEOPHONE_CMD_IMU_FREQ_HP,     /* 0x1B | APP CMD: set IMU frequency High-power. Value: frequency [Hz] */
EVENT_GEOPHONE_MSC_MOUNT,           /* 0x1C | USB MSC mounted */
EVENT_GEOPHONE_MSC_EJECT,           /* 0x1D | USB MSC ejected */
EVENT_GEOPHONE_CMD_SPS,             /* 0x1E | APP CMD: set ADC rate (SPS). Value: frequency [Hz] */
EVENT_GEOPHONE_CMD_SYS_OPMODE,      /* 0x1F | APP CMD: set operating mode. Value: @see enum OpModes */

EVENT_GEOPHONE_STACK_90,            /* 0x20 | RTOS stack watermark 90% reached. Value: task ID */
EVENT_GEOPHONE_STACK_TMRSVC,        /* 0x21 | Timer Daemon Stack watermark 90% reached. Value: stack usage percentage */
EVENT_GEOPHONE_STACK_IDLE,          /* 0x22 | Idle Task Stack watermark 90% reached. Value: stack usage percentage */
EVENT_GEOPHONE_HEAP_90,             /* 0x23 | Heap usage 90% reached. Value: heap usage percentage */
EVENT_GEOPHONE_VCC_OOR,             /* 0x24 | Battery VCC value is out-of-range */
EVENT_GEOPHONE_TEMP_OOR,            /* 0x25 | Temperature value is out-of-range */
EVENT_GEOPHONE_HUMID_OOR,           /* 0x26 | Humidity value is out-of-range */

EVENT_GEOPHONE_RB_FULL,             /* 0x27 | Ring buffer full (argument: buffer ID) */
EVENT_GEOPHONE_SD_IO_ERROR,         /* 0x28 | SD card read or write error */
EVENT_GEOPHONE_SD_INV_FILE,         /* 0x29 | File does not exist or is invalid */
EVENT_GEOPHONE_CMD_REQ_ADCDATA,     /* 0x2A | APP CMD: get ADC data (waveform). Value: error code */
EVENT_GEOPHONE_CMD_DEL_DATA,        /* 0x2B | APP CMD: data from SD card deleted. Value: # deleted files */
EVENT_GEOPHONE_CMD_RESET_CFG,       /* 0x2C | APP CMD: config has been reset to the default values */
EVENT_GEOPHONE_CMD_IMU_FREQ_LP,     /* 0x2D | APP CMD: set IMU frequency Low-power. Value: frequency [Hz] */
EVENT_GEOPHONE_CMD_IMU_FREQ_AA,     /* 0x2E | APP CMD: set IMU frequency Anti-Aliasing. Value: frequency [Hz] */
EVENT_GEOPHONE_CMD_IMU_OPMODE,      /* 0x2F | APP CMD: set IMU Operation Mode. */
EVENT_GEOPHONE_CMD_IMU_TRG_LVL,     /* 0x30 | APP CMD: set IMU Trigger Level. */
EVENT_GEOPHONE_CMD_IMU_DATA_DEC,    /* 0x31 | APP CMD: set IMU Data Decimation. Value: samples */

EVENT_GEOPHONE_SCHED_CL_ERR,        /* 0x32 | Scheduler clear error */
EVENT_GEOPHONE_SCHED_WR_ERR,        /* 0x33 | Add an entry to the scheduler error */
EVENT_GEOPHONE_CMD_SYS_WAKEUP_PERIOD, /* 0x34 | APP CMD: set new Wakeup time. Remember to make an application reset afterwards */
EVENT_GEOPHONE_CMD_TSYNC_PERIOD,    /* 0x35 | APP CMD: Set the new Bolt tysnc period */
EVENT_GEOPHONE_SD_FULL,             /* 0x36 | SD card is almost full */

/* Label --------------------------- | #No. | Description ----------------------------*/
