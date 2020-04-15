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

/* Label ------------------- | #No. | Description ----------------------------*/

CMD_RESET,                  /* 0x01 | Perform Reset */
CMD_SELFTEST,               /* 0x02 | Perform SelfTest */
CMD_GPS_SAMPLING,           /* 0x03 | Set GPS sampling rate. Value: 2 bytes, [sec] */
CMD_STATUS_SAMPLING,        /* 0x04 | Set Status sampling rate. Value: 2 bytes, [sec] */
CMD_HEALTH_SAMPLING,        /* 0x05 | Set Health sampling rate. Value: 2 bytes, [sec] */
CMD_LO_PWR_ENTRY_VDC,       /* 0x06 | Set Low Power Entry Voltage. Value: 2 bytes, [10^-3 V] */
CMD_LO_PWR_EXIT_VDC,        /* 0x07 | Set Low Power Exit Voltage. Value: 2 bytes, [10^-3 V] */
CMD_SCHEDULE,               /* 0x08 | Set GPS Schedule. Value: 5 bytes [day_selector_bitfield, hi_start_hour, hi_stop_hour, lo_start_hour, lo_stop_hour] */


