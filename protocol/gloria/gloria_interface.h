/*
* gloria_interface.h
*
*  Created on: 04.12.2019
*      Author: Roman Trub
 */

#ifndef PROTOCOL_GLORIA_GLORIA_INTERFACE_H_
#define PROTOCOL_GLORIA_GLORIA_INTERFACE_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "time/hs_timer.h"
#include "protocol/protocol.h"
#include "radio/radio_helpers.h"
#include "system/system.h"

#include "flocklab/flocklab.h"

#include "protocol/gloria/gloria_constants.h"
#include "protocol/gloria/gloria_structures.h"
#include "protocol/gloria/gloria_helpers.h"
#include "protocol/gloria/gloria_radio.h"
#include "protocol/gloria/gloria_time.h"
#include "cli/commands/gloria_cmd.h"

/*******************************************************************************
* BEGIN: GMW INTERFACE
******************************************************************************/

/* CONFIG DEFAULTS ************************************************************/

/**
 * Maximum length of payload (in Bytes) expected to be passed to the gloria
 * interface (e.g. from GMW). This value must not exceed the max payload length
 * of gloria (GLORIA_MAX_PAYLOAD_LENGTH).
 */
#ifndef GLORIA_INTERFACE_MAX_PAYLOAD_LEN
#define GLORIA_INTERFACE_MAX_PAYLOAD_LEN    GLORIA_MAX_PAYLOAD_LENGTH
#endif /* GLORIA_INTERFACE_MAX_PAYLOAD_LEN */

/**
 * Select frequency and bandwidth from predefined configurations in
 * radio_constants.c
 */
#ifndef GLORIA_INTERFACE_RF_BAND
#define GLORIA_INTERFACE_RF_BAND            RADIO_DEFAULT_BAND
#endif /* GLORIA_INTERFACE_RF_BAND */

/**
 * Select modem, bandwidth, datarate, coderate, preambleLen from predefined
 * configurations in radio_constants.c
 */
#ifndef GLORIA_INTERFACE_MODULATION
#define GLORIA_INTERFACE_MODULATION         3
#endif /* GLORIA_INTERFACE_MODULATION */

/**
 * Radio Tx power in dBm
 */
#ifndef GLORIA_INTERFACE_POWER
#define GLORIA_INTERFACE_POWER              0  // in dBm
#endif /* GLORIA_INTERFACE_POWER */

/**
 * Upper bound of the slot number during one flood (limits flood duration)
 * NOTE: also limits max number of hops
 * NOTE: intentially set to a large number since GMW does not specify this
 *       number but limits the gloria round by calling gloria_stop().
 */
#ifndef GLORIA_INTERFACE_MAX_SLOTS
// #define GLORIA_INTERFACE_MAX_SLOTS          4
#define GLORIA_INTERFACE_MAX_SLOTS          127
#endif /* GLORIA_INTERFACE_MAX_SLOTS */


/* CONFIG CHECKS **************************************************************/

#if GLORIA_INTERFACE_MAX_PAYLOAD_LEN > GLORIA_MAX_PAYLOAD_LENGTH
#error "GLORIA_INTERFACE_MAX_PAYLOAD_LEN exceeds allowed range!"
#endif


/* FUNCTIONS ******************************************************************/

// FIXME: update description
/**
 * \brief               Start Glossy and stall all other application tasks.
 *
 * \param initiator_id  Node ID of the flood initiator.
 * \param payload       A pointer to the data.
 *                      At the initiator, Glossy reads from the given memory
 *                      location data provided by the application.
 *                      At a receiver, Glossy writes to the given memory
 *                      location data for the application.
 *                      NOTE: At the receiver, the payload buffer must have a
 *                      size of at least GLORIA_INTERFACE_MAX_PAYLOAD_LEN Bytes.
 * \param payload_len   Length of the flooding data, in bytes.
 *                      NOTE: Only used if node which calls gloria_start is
 *                      initiator, otherwise this value is ignored!
 * \param n_tx_max      Maximum number of transmissions (N).
 * \param sync_slot     Not zero if Gloria must provide time synchronization,
 *                      zero otherwise.
 */
void gloria_start(uint16_t initiator_id,
                  uint8_t *payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  uint8_t sync_slot);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 *                   NOTE: GMW is not using this return value but uses the
 *                   gloria_get_rx_cnt() function.
 */
uint8_t gloria_stop(void);

/**
 * \brief            Get the number of received packets during the last gloria
 *                   run.
 * \Returns          Number of messages received during the last gloria run.
 *                   NOTE: There are only 2 return values (0 or 1) since gloria
 *                   receives the message at most once.
 *                   NOTE: If function is called on the node which initiated the
 *                   flood corresponding to the last gloria run, the return
 *                   value is always 0.
 */
uint8_t gloria_get_rx_cnt(void);

/**
 * \brief            Get the received payload length of the last flood
 * \returns          Length of the payload received during the last gloria run.
 *                   NOTE: Returns 0 if gloria_get_rx_cnt returns 0!
 *                   NOTE: If function is called on the node which initiated the
 *                   flood corresponding to the last gloria run, the return
 *                   value is always 0.
 */
uint8_t gloria_get_payload_len(void);

/**
 * \brief            Get the index of the slot in which the message was
 *                   received.
 * \returns          Index of the slot in which the message was received (relay
 *                   counter) received during the last Gloria run.
 *                   (first packet = last packet, since in gloria a message is
 *                   received at most once)
 *                   NOTE: Returns 0 if gloria_get_rx_cnt returns 0!
 */
uint8_t gloria_get_rx_index(void);

/**
 * \brief            Get the number of preamble detect events during the last
 *                   gloria run.
 * \Returns          Number of preamble detect events during the last
 *                   gloria run.
 *                   NOTE: There can be arbitrary many preamble detect
 *                   events (not related to number of hops or number of
 *                   retransmissions since any valid packet triggers a preamble
 *                   detect event)
 */
uint8_t gloria_get_rx_preamble_cnt(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Gloria run, zero otherwise.
 *                   NOTE: With the current implementation, t_ref is not updated
 *                   if gloria is stopped before the flood ends, even if packets
 *                   have been recieved.
 */
uint8_t gloria_is_t_ref_updated(void);

/**
 * \brief            Get the sync reference time.
 * \returns          Reference timestamp (flood marker) of last sucessfully
 *                   received flood during a Gloria run with sync_slot set to
 *                   true, in lptimer clock ticks
 *                   NOTE: The returned t_ref value is NOT updated nor reset
 *                   during a gloria run with unsuccessful reception or a gloria
 *                   run without sync_slot set to 0.
 */
uint64_t gloria_get_t_ref(void);

/* Extended Interface *********************************************************/

/**
 * \brief            Set transmission power of the radio (in dBm)
 */
void gloria_set_tx_power(int8_t power);

/**
 * \brief            Set modulation config of the radio (valid options are
 *                   defined in radio_constants.c)
 */
void gloria_set_modulation(uint8_t modulation);

/**
 * \brief            Set frequency and bandwidth config of the radio (valid
 *                   options are defined in radio_constants.c)
 */
void gloria_set_band(uint8_t band);

/**
 * \brief            Calculates the time-on-air (in us) for a single packet with
 *                   the current radio settings. Argument `len` relates to the
 *                   upper layer payload length (i.e. without overhead added by
 *                   gloria).
 * \param            len: Number of payload Bytes from the upper layer
 */
uint32_t gloria_get_time_on_air(uint8_t payload_len);

/**
 * \brief            Enable the printing of finished (i.e. completely
 *                   transmitted/received floods)
 * \param            enable: Set to True to enable flood printing
 */
void gloria_enable_flood_printing(bool enable);


/*******************************************************************************
 * END: GMW INTERFACE
 ******************************************************************************/

#endif /* PROTOCOL_GLORIA_GLORIA_INTERFACE_H_ */
