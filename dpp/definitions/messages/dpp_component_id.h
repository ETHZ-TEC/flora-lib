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
 * DPP hardware component IDs
 *
 * - each device (= DPP component) gets a unique ID
 * - each hardware revision should get a separate ID (e.g. WGPS and WGPS2)
 * - the component ID does not necessarily have to reflect the use case (as
 *   e.g. in WGPS), but must be an unambiguous identifier for a physical
 *   device and its revision
 */

#ifndef __DPP_COMPONENT_ID_H__
#define __DPP_COMPONENT_ID_H__

enum {
  DPP_COMPONENT_ID_INVALID    = 0,
  DPP_COMPONENT_ID_CC430      = 1,  /* ComBoard with the CC430 */
  DPP_COMPONENT_ID_WGPS2      = 2,  /* Wireless GPS 2 */
  DPP_COMPONENT_ID_GEOPHONE   = 3,  /* Geophone 1x */
  DPP_COMPONENT_ID_DEVBOARD   = 4,  /* DevBoard MSP432 application processor */
  DPP_COMPONENT_ID_SX1262     = 5,  /* ComBoard with Semtech SX1262 */
  DPP_COMPONENT_ID_GEO3X      = 6,  /* Geophone 3x */
  DPP_COMPONENT_ID_GEOMINI    = 7,  /* Geophone mini */

  DPP_COMPONENT_ID_LASTID     = 0xff
};


#endif /* __DPP_COMPONENT_ID_H__ */
