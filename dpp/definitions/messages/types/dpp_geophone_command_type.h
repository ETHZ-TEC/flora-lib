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
 * command types specific to APP (application processor)
 *
 *
 * typedef enum {
 *   ...
 *   DPP_COMMAND_[device_name]  = [start_id],
 *   [your_defines]
 *   ...
 * } dpp_command_type_t;
 *
 * add a comma after each define!
 */

/* Label ----------------------- | #No. | Description ----------------------------*/

CMD_GEOPHONE_RESET,             /* 0x01 | Perform Reset */                  /* @param none */
CMD_GEOPHONE_SELFTEST,          /* 0x02 | Perform Self-Test */              /* @param none */
CMD_GEOPHONE_SYS_OPMODE,        /* 0x03 | System Operation Mode */          /* @param 0: normal, @see enum OpModes */
CMD_GEOPHONE_EXTTRG,            /* 0x04 | External Trigger */               /* @param none */
CMD_GEOPHONE_TRG_GAIN,          /* 0x05 | Trigger Amplification Gain */     /* @param 0: stage1, 1: stage2 */
CMD_GEOPHONE_TRG_TH_POS,        /* 0x06 | Trigger Positive Threshold */     /* @param voltage [1500-3000] mV */
CMD_GEOPHONE_TRG_TH_NEG,        /* 0x07 | Trigger Negative Threshold */     /* @param voltage [0-1500] mV */
CMD_GEOPHONE_POSTTRG,           /* 0x08 | ADC Post-trigger interval */      /* @param > 0 seconds */
CMD_GEOPHONE_TIMEOUT,           /* 0x09 | ADC Sampling Timeout */           /* @param > 0 seconds */
CMD_GEOPHONE_ADC_PGA,           /* 0x0A | ADC PGA Value */                  /* @param [0-128] */
CMD_GEOPHONE_ADC_FORMAT,        /* 0x0B | ADC Output Format */              /* @param 0: two's complement, 1: offset binary */
CMD_GEOPHONE_IMU_FREQ_HP,       /* 0x0C | IMU Frequency High Power */       /* @param [0-800] Hz */
CMD_GEOPHONE_REQ_ADCDATA,       /* 0x0D | Request ADC data (waveform) */    /* @param acquisition ID (32-bit), mode (8-bit) */
CMD_GEOPHONE_ADC_SPS,           /* 0x0E | ADC Rate (samples per second) */  /* @param rate: 0 = 1kHz, 1 = 500Hz, 2 = 250Hz, 3 = 125Hz */
CMD_GEOPHONE_DEL_DATA,          /* 0x0F | Delete data on SD card */         /* @param start and end UTC timestamp in s (2x 32-bit) */
CMD_GEOPHONE_RESET_CFG,         /* 0x10 | Reset config to default values */ /* @param none */
CMD_GEOPHONE_IMU_FREQ_LP,       /* 0x11 | IMU Frequency Low Power */        /* @param [0-800] Hz */
CMD_GEOPHONE_IMU_FREQ_AA,       /* 0x12 | IMU Frequency Anti-Aliasing */    /* @param [0-400] Hz */
CMD_GEOPHONE_IMU_OPMODE,        /* 0x13 | IMU Operation Mode */             /* @param 0: normal */
CMD_GEOPHONE_IMU_TRG_LVL,       /* 0x14 | IMU Trigger Level */              /* @param 1: lowest setting */
CMD_GEOPHONE_IMU_DATA_DEC,      /* 0x15 | IMU Data Decimation Factor */     /* @param 1: no decimation */
CMD_GEOPHONE_SCHED_CLEAR,       /* 0x16 | Clear Scheduler */                /* @param none */
CMD_GEOPHONE_SCHED_ADD,         /* 0x17 | Add new entry to Scheduler */     /* @param start unix time stamp (32bit), period (20bit), duration (4bit), task mode (8bit) */
CMD_GEOPHONE_SYS_WAKEUP_PERIOD, /* 0x18 | Set Wakeup period */              /* @param [0-30] s; requires application reset to take effect */
CMD_GEOPHONE_TSYNC_PERIOD,      /* 0x19 | Set Tsync period time */          /* @param [0 - 65535] s */

/* Label ----------------------- | #No. | Description ----------------------------*/
