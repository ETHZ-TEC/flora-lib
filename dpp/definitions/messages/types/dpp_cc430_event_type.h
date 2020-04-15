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
 * event types specific to the CC430 (communication processor)
 *
 * 
 * typedef enum {
 *   ...
 *   DPP_EVENT_[device name]  = [component id],
 *   [your_defines]
 *   ...  
 * } dpp_event_type_t;
 * 
 * add a comma after each line, don't assign values/numbers to the enums
 */

/* Label ----------------------- | #No. | Description ---------------------- | Value ---*/

/* general notifications */
EVENT_CC430_NODE_RST,           /* 0x01 | a reset has occurred               | reset cause  */
EVENT_CC430_CFG_CHANGED,        /* 0x02 | config has changed (ACK)           | cmd type and new value */

/* errors and warnings */
EVENT_CC430_RADIO_ERROR,        /* 0x03 | radio protocol error (glossy, LWB) | error code */
EVENT_CC430_BOLT_ERROR,         /* 0x04 | bolt communication error           | error code */
EVENT_CC430_MEM_OVERFLOW,       /* 0x05 | memory full or overflown           | error code */
EVENT_CC430_QUEUE_FULL,         /* 0x06 | queue full, message dropped        | queue ID */
EVENT_CC430_INV_MSG,            /* 0x07 | invalid message received           | error code */
EVENT_CC430_INV_CMD,            /* 0x08 | invalid command received           | cmd type and value */
EVENT_CC430_MSG_IGNORED,        /* 0x09 | unknown message type, ignored      | msg type */
EVENT_CC430_TIME_UPDATED,       /* 0x0a | local time has been adjusted       | offset */

/* status messages */
EVENT_CC430_FW_PROGRESS,        /* 0x0b | FW update progress                 | percentage of FW image already stored */
EVENT_CC430_NODE_ADDED,         /* 0x0c | host: a node has joined            | node ID */
EVENT_CC430_NODE_REMOVED,       /* 0x0d | host: node removed or ignored      | node ID */

/* more error notifications */
EVENT_CC430_CORRUPTED_CONFIG,   /* 0x0e | config in flash memory corrupted   | 0 */
EVENT_CC430_CORRUPTED_SCHEDULE, /* 0x0f | corrupted schedule received (CRC)  | 0 */

/* last symbol in this file must be a comma */

