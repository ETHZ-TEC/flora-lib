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
 * command types specific to COM (communication processor)
 *
 * 
 * typedef enum {
 *   ...
 *   DPP_COMMAND_[device name]  = [component id],
 *   [your_defines]
 *   ...  
 * } dpp_command_type_t;
 * 
 * add a comma after each line, don't assign values/numbers to the enums
 */

/* Label ------------------------- | #No. | Description ---------------------*/

CMD_CC430_RESET,                  /* 0x01 | perform soft reset */
CMD_CC430_SET_ROUND_PERIOD,       /* 0x02 | communication round period T for schedule in s */
CMD_CC430_SET_HEALTH_PERIOD,      /* 0x03 | period for health messages in seconds (0 = no health messages) */
CMD_CC430_SET_EVENT_LEVEL,        /* 0x04 | event notification level */
CMD_CC430_SET_TX_POWER,           /* 0x05 | set radio transmit power, 0 = -30dBm, 1 = -12dBm, 2 = -6dBm, 3 = 0dBm, 4 = +10dBm, 5 = MAX (~+12) */
CMD_CC430_SET_DBG_FLAGS,          /* 0x06 | set debug flags (1 = turn LEDs off) */
CMD_CC430_DBG_READ_MEM,           /* 0x07 | for debugging: read memory location, address (16-bit) + length (8-bit), max. 32 bytes */
CMD_CC430_ADD_NODE,               /* 0x08 | register a node ID in the network (only host nodes handle this command) */
CMD_CC430_SET_NODE_ID,            /* 0x09 | change the device ID of a source node (only as a workaround in case a node loses its configuration) */

/* last symbol in this file must be a comma */

