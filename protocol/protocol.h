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
#define PROTOCOL_ID_DISCOSYNC             0x4

#define PROTOCOL_ID_CUSTOM                0xc     /* reserved ID for user-defined protocols (not part of the Flora lib) -> each protocol must have a unique 4-bit sub-ID (see table below) */
#define PROTOCOL_ID_CUSTOM2               0xd
#define PROTOCOL_ID_CUSTOM3               0xe
#define PROTOCOL_ID_CUSTOM4               0xf

#define PROTOCOL_ID_MASK                  0x0f
#define PROTOCOL_CHECK_ID(id, protocol)   (((id) & PROTOCOL_ID_MASK) == (PROTOCOL_ID_##protocol))


/*
 * list of custom protocols (defined in a user project) -> the upper 4 bits are used to differentiate them
 * NOTE: register all user-defined protocols here that need to rely on a unique identifier on the lowest layer
 */
#define PROTOCOL_ID_CUSTOM_FT_DISCOVERY   (PROTOCOL_ID_CUSTOM + 0x10)         /* used in Fault Tolerance thesis project */


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
#include "protocol/discosync/discosync.h"


#endif /* PROTOCOL_PROTOCOL_H_ */
