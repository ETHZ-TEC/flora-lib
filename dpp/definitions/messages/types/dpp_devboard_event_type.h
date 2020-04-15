/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * event types specific to the BOLT bridge (msp432 app processor)
 */

/* Label ----------------------- | #No. | Description ---------------------- | Value ---*/

EVENT_DEVBOARD_BOLT_ERROR,      /* 0x01 | BOLT communication error           | type of error  */
EVENT_DEVBOARD_I2C_ERROR,       /* 0x02 | I2C communication error            | type of error  */
EVENT_DEVBOARD_UART_ERROR,      /* 0x03 | UART communication error           | type of error  */
EVENT_DEVBOARD_INV_CRC,         /* 0x04 | Message dropped due to invalid CRC | src and pkt type */
EVENT_DEVBOARD_INV_LEN,         /* 0x05 | Message dropped due to inv. length | src and pkt type */
EVENT_DEVBOARD_QUEUE_FULL,      /* 0x06 | Message dropped due to queue full  | queue ID */
EVENT_DEVBOARD_CFG_CHANGED,     /* 0x07 | Command processed, config changed  | cmd type and new value */

/* last symbol in this file must be a comma */
