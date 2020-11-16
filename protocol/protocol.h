/*
 * protocol.h
 *
 *  Created on: Nov 16, 2020
 *      Author: rdaforno
 */

#ifndef PROTOCOL_PROTOCOL_H_
#define PROTOCOL_PROTOCOL_H_

/*
 * Each of the implemented protocols and protocol variants has a unique 4-bit ID which must be included as the first byte (least significant bits) in all sent packets.
 * The remaining (upper) 4 bits can be used arbitrarily by the protocol implementation, e.g. to distinguish messages of different types.
 */

#define PROTOCOL_ID_INVALID               0x0
#define PROTOCOL_ID_GLORIA                0x1
#define PROTOCOL_ID_DOZER                 0x2
#define PROTOCOL_ID_CHAOS                 0x3

#define PROTOCOL_ID_MASK                  0x0f
#define PROTOCOL_CHECK_ID(id, protocol)   (((id) & PROTOCOL_ID_MASK) == (PROTOCOL_ID_##protocol))


#include "protocol/gloria/gloria.h"
#if DOZER_ENABLE
#include "protocol/dozer/dozer.h"
#endif /* DOZER_ENABLE */
#if SLWB_ENABLE
#include "protocol/simple_lwb/slwb.h"
#endif /* SLWB_ENABLE */
#if GMW_ENABLE
#include "protocol/gmw/gmw.h"
#endif /* GMW_ENABLE */
#if ELWB_ENABLE
#include "protocol/elwb/elwb.h"
#endif /* ELWB_ENABLE */


#endif /* PROTOCOL_PROTOCOL_H_ */
