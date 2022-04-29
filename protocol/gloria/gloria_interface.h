/*
 * Copyright (c) 2018 - 2022, ETH Zurich, Computer Engineering Group (TEC)
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
 */

#ifndef PROTOCOL_GLORIA_GLORIA_INTERFACE_H_
#define PROTOCOL_GLORIA_GLORIA_INTERFACE_H_


/* CONFIG DEFAULTS ************************************************************/

/**
 * Maximum length of payload (in Bytes) expected to be passed to the gloria
 * interface. This value must not exceed the max payload length
 * of Gloria (GLORIA_MAX_PAYLOAD_LENGTH).
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
#define GLORIA_INTERFACE_MODULATION         10
#endif /* GLORIA_INTERFACE_MODULATION */

/**
 * Radio Tx power in dBm
 */
#ifndef GLORIA_INTERFACE_POWER
#define GLORIA_INTERFACE_POWER              0  // in dBm
#endif /* GLORIA_INTERFACE_POWER */

/**
 * Whether to disable potentially interfering interrupts before starting a
 * Gloria flood. The disabled interrupts will be re-enabled in gloria_stop().
 */
#ifndef GLORIA_INTERFACE_DISABLE_INTERRUPTS
#define GLORIA_INTERFACE_DISABLE_INTERRUPTS 1
#endif /* GLORIA_INTERFACE_DISABLE_INTERRUPTS */

/**
 * Upper bound of the slot number during one flood (limits flood duration)
 * NOTE: also limits max number of hops
 * NOTE: intentionally set to a large number since GMW does not specify this
 *       number but limits the Gloria round by calling gloria_stop().
 */
#ifndef GLORIA_INTERFACE_MAX_SLOTS
#define GLORIA_INTERFACE_MAX_SLOTS          127
#endif /* GLORIA_INTERFACE_MAX_SLOTS */

/**
 * If set to 1, the initiator will include a timestamp in the Gloria header
 * (GLORIA_TIMESTAMP_LENGTH bytes, hstimer timestamp divided by
 * GLORIA_SCHEDULE_GRANULARITY).
 * NOTE: this is just the default setting and can be overwritten at runtime
 *       with gloria_enable_append_timestamp()
 */
#ifndef GLORIA_INTERFACE_APPEND_TIMESTAMP
#define GLORIA_INTERFACE_APPEND_TIMESTAMP   0
#endif /* GLORIA_INTERFACE_APPEND_TIMESTAMP */

/**
 * If enabled, gloria_stop() will wait for the current TX slot to finish before
 * forcing the radio into standby mode.
 * NOTE: If enabled, gloria_stop() must not be called from an interrupt that
 *       has the same or higher priority than the hs_timer / radio interrupt.
 */
#ifndef GLORIA_INTERFACE_WAIT_TX_FINISHED
#define GLORIA_INTERFACE_WAIT_TX_FINISHED   0
#endif /* GLORIA_INTERFACE_WAIT_TX_FINISHED */


/* CONFIG CHECKS **************************************************************/

#if GLORIA_INTERFACE_MAX_PAYLOAD_LEN > GLORIA_MAX_PAYLOAD_LENGTH
#error "GLORIA_INTERFACE_MAX_PAYLOAD_LEN exceeds allowed range!"
#endif

/* FUNCTIONS ******************************************************************/

/**
 * \brief               Start Gloria (for sending, participating in, receiving
 *                      a flood).
 *                      NOTE: Gloria interface requires that the radio is in
 *                      standby mode when gloria_start() is called. If radio is
 *                      in sleep mode, the timing is not feasible and errors
 *                      will occur!
 * \param is_initiator  Set to true to initiate a flood.
 * \param payload       A pointer to the data.
 *                      At the initiator, Glossy reads from the given memory
 *                      location data provided by the application.
 *                      At a receiver, Glossy writes to the given memory
 *                      location data for the application.
 * \param payload_len   Size of the `payload` buffer in bytes. On the receiver
 *                      side, this value limits the max. packet size that will
 *                      be accepted. Note that a payload length of zero is
 *                      allowed.
 * \param n_tx_max      Maximum number of transmissions (N).
 * \param sync_slot     Not zero if flood should be used to update the
 *                      reference, zero otherwise.
 */
void gloria_start(bool is_initiator,
                  uint8_t *payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  uint8_t sync_slot);

/**
 * \brief            Stop Gloria.
 *                   NOTE: Gloria interface leaves the radio in
 *                   standby mode after gloria_stop() is called. The user of the
 *                   interface should make sure to put it into sleep mode if
 *                   desired!
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t gloria_stop(void);

/**
 * \brief            Get the number of received packets during the last Gloria
 *                   run.
 * \Returns          Number of messages received during the last Gloria run.
 *                   NOTE: There are only 2 return values (0 or 1) since Gloria
 *                   receives the message at most once.
 *                   NOTE: If function is called on the node which initiated the
 *                   flood corresponding to the last Gloria run, the return
 *                   value is always 0.
 */
uint8_t gloria_get_rx_cnt(void);

/**
 * \brief            Get the received payload length of the last flood
 * \returns          Length of the payload received during the last Gloria run.
 *                   NOTE: Returns 0 if gloria_get_rx_cnt returns 0!
 *                   NOTE: If function is called on the node which initiated the
 *                   flood corresponding to the last Gloria run, the return
 *                   value is always 0.
 */
uint8_t gloria_get_payload_len(void);

/**
 * \brief            Get the index of the slot in which the message was
 *                   received.
 * \returns          Index of the slot in which the message was received (relay
 *                   counter) received during the last Gloria run.
 *                   (first packet = last packet, since in Gloria a message is
 *                   received at most once)
 *                   NOTE: Returns 0 if gloria_get_rx_cnt returns 0!
 */
uint8_t gloria_get_rx_index(void);

/**
 * \brief            Get the number of RX started events during the last
 *                   gloria run.
 * \Returns          Number of RX started events during the last gloria
 *                   run. Depending on the modulation, this can be the number
 *                   of detected preambles, sync words or valid headers.
 */
uint8_t gloria_get_rx_started_cnt(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Gloria run, zero otherwise.
 *                   NOTE: With the current implementation, t_ref is not updated
 *                   if Gloria is stopped before the flood ends, even if packets
 *                   have been received.
 */
uint8_t gloria_is_t_ref_updated(void);

/**
 * \brief            Get the sync reference time in lptimer ticks.
 * \returns          Reference timestamp (flood marker) of last successfully
 *                   received flood during a Gloria run with sync_slot set to
 *                   true, in lptimer clock ticks
 *                   NOTE: The returned t_ref value is NOT updated nor reset
 *                   during a Gloria run with unsuccessful reception or a gloria
 *                   run without sync_slot set to 0.
 */
uint64_t gloria_get_t_ref(void);

/**
 * \brief            Get the sync reference time in hs timer ticks.
 * \returns          Reference timestamp (flood marker) of last successfully
 *                   received flood during a Gloria run with sync_slot set to
 *                   true, in hs timer clock ticks
 */
uint64_t gloria_get_t_ref_hs(void);

/* Extended Interface *********************************************************/

/**
 * \brief            Set transmission power of the radio (in dBm)
 */
void gloria_set_tx_power(int8_t power);

/**
 * \brief            Set modulation config of the radio (valid options are
 *                   defined in radio_constants.c)
 * \param            modulation: flora modulation (see radio_constants.c)
 */
void gloria_set_modulation(uint8_t modulation);

/**
 * \brief            Set frequency and bandwidth config of the radio (valid
 *                   options are defined in radio_constants.c)
 * \param            band: flora band (see radio_constants.c)
 */
void gloria_set_band(uint8_t band);

/**
 * \brief            Calculates the time-on-air (in us) for a single packet with
 *                   the current radio settings.
 *                   NOTE: Before calling this function, configure gloria.
 *
 * \param            payload_len: Number of payload Bytes from the upper layer
 *                   (i.e. without overhead added by gloria).
 * \returns          Time-on-air in us
 */
uint32_t gloria_get_toa(uint8_t payload_len);

/**
 * \brief            Calculates the time-on-air (in us) for a single packet with
 *                   a specific radio setting.
 *                   (sl = stateless)
 *
 * \param            payload_len: Number of payload Bytes from the upper layer
 *                   (i.e. without overhead added by gloria).
 * \param            modulation: flora modulation (see radio_constants.c)
 * \returns          Time-on-air in us
 */
uint32_t gloria_get_toa_sl(uint8_t payload_len, uint8_t modulation);

/**
 * \brief            Returns the flood time (in us)
 *                   with the current modulation and band settings.
 *                   NOTE: Before calling this function, make sure to configure
 *                   gloria.
 *
 * \param            payload_len: Number of payload Bytes from the upper layer
 *                   (i.e. without overhead added by gloria).
 * \param            num_slots: number of slots the Gloria flood consists of
 * \return           Duration of the flood (in us)
 */
uint32_t gloria_get_flood_time(uint8_t payload_len, uint8_t num_slots);


/**
 * \brief            Returns the flood time (in us)
 *                   for a specific modulation and band setting.
 *                   (sl = stateless)
 *
 * \param            payload_len: Number of payload Bytes from the upper layer
 *                   (i.e. without overhead added by gloria).
 * \param            modulation: flora modulation (see radio_constants.c)
 * \param            num_slots: number of slots the Gloria flood consists of
 * \return           Duration of the flood (in us)
 */
 uint32_t gloria_get_flood_time_sl(uint8_t payload_len, uint8_t modulation, uint8_t num_slots);

/**
 * \brief            Enable the printing of finished (i.e. completely
 *                   transmitted/received floods)
 * \param            enable: Set to True to enable flood printing
 */
void gloria_enable_flood_printing(bool enable);

/**
 * \brief            Register callback function which is called when
 *                   participation in the flood terminated.
 *                   NOTE: Callback function is called only if participation in
 *                   flood stops before gloria_stop() is called!
 * \param            cb: callback function
 */
void gloria_register_flood_callback(gloria_flood_cb_t flood_cb);

/**
 * \brief           Set a custom RX packet filter
 * \param           A callback function that takes a pointer to the received
 *                  header, the header length, a pointer to the payload and the
 *                  payload length. It must returns true if the received packet
 *                  should be kept / accepted and false otherwise.
 *                  IMPORTANT: The function must be fast and deterministic. Make
 *                  sure the function completes execution within ~100us. Longer
 *                  execution times may disrupt the Gloria timing.
 * \note            The callback function will be cleared in gloria_stop().
 */
void gloria_set_pkt_filter(gloria_filter_cb_t filter_cb);

/**
 * \brief           Register a function that is called upon reception of a packet.
 * \param           Pointer to a callback function
 * \note            The callback function must be short. If the execution time is
 *                  larger than a few microseconds, the Gloria slotOverhead must
 *                  be adjusted.
 */
void gloria_register_rx_callback(gloria_rx_cb_t rx_cb);

/**
 * \brief           Set the transmission start time (TX marker) for the initiator.
 * \param           Start time in hs timer ticks.
 */
void gloria_set_tx_marker(uint64_t timestamp_hs);

/**
 * \brief           Set the offset of the first transmission slot relative to the
 *                  first successful reception.
 * \param           offset in number of slots
 */
void gloria_set_tx_delay(uint8_t delay_slots);

/**
 * \brief           Get the RSSI value of the last received packet.
 * \return          RSSI value in dBm
 */
int32_t gloria_get_rssi();

/**
 * \brief           Get the SNR value of the last received packet.
 * \return          SNR value in dBm
 */
int32_t gloria_get_snr();

/**
 * \brief           Enable or disable the inclusion of a timestamp in the
 *                  Gloria header.
 */
void gloria_enable_append_timestamp(bool enable);

/**
 * \brief           Returns the last received Gloria timestamp
 * \param           buffer to hold the timestamp
 * \return          true if a timestamp has been received during the last
 *                  flood (and copied into `out_timestamp`), false otherwise
 */
bool gloria_get_received_timestamp(uint8_t* out_timestamp);


/**
 * macros for required flood duration
 */

// flood duration in ms
#define GLORIA_INTERFACE_FLOOD_DURATION_MS(n_tx, n_hops, len)    (gloria_get_flood_time(len, n_tx + n_hops - 1) / 1000UL)
// flood duration in lptimer ticks
#define GLORIA_INTERFACE_FLOOD_DURATION(n_tx, n_hops, len)       LPTIMER_US_TO_TICKS(gloria_get_flood_time(len, n_tx + n_hops - 1))


#endif /* PROTOCOL_GLORIA_GLORIA_INTERFACE_H_ */
