/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
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

#ifndef RADIO_RADIO_HELPERS_H_
#define RADIO_RADIO_HELPERS_H_


void      radio_update_cli(void);

void      radio_set_lora_syncword(radio_lora_syncword_t syncword);
uint16_t  radio_get_syncword(void);
void      radio_set_config_rx(uint8_t modulation_index,
                              uint8_t band_index,
                              int32_t bandwidth,
                              int32_t datarate,
                              int32_t preamble_len,
                              uint16_t timeout,
                              bool implicit,
                              uint8_t implicit_len,
                              bool crc,
                              bool stop_rx_on_preamble);
void      radio_set_config_tx(uint8_t modulation_index,
                              uint8_t band_index,
                              int8_t power,
                              int32_t bandwidth,
                              int32_t datarate,
                              int32_t preamble_len,
                              bool implicit,
                              bool crc);
void      radio_set_config_rxtx(bool lora_mode,
                                uint8_t band_index,
                                int32_t datarate,
                                int8_t power,
                                int32_t bandwidth,
                                int32_t preamble_len,
                                uint8_t coderate,
                                uint16_t timeout,
                                bool implicit,
                                uint8_t implicit_len,
                                bool crc);
void      radio_set_config(uint8_t modulation_index,
                           uint8_t band_index,
                           int8_t power,
                           uint8_t max_payload_len);
void      radio_set_continuous_preamble(void);

void      radio_get_payload(uint8_t* buffer, uint8_t* offset, uint8_t* size);
uint8_t   radio_get_payload_size(void);
void      radio_set_payload_size(uint8_t size);
void      radio_set_payload(uint8_t* buffer, uint8_t size);
void      radio_set_payload_chunk(uint8_t* buffer, uint8_t offset, uint8_t size, bool last_chunk);    // the last chunk of the payload must be the one with the largest offset
void      radio_set_payload_while_transmit(uint8_t* buffer, uint8_t size);


/*!
 * \brief Calculates the time-on-air a number of symbols
 * \param [IN] length            Number of symbols (LoRa) / Bytes (FSK)
 * \param [IN] modulation        Modulation index (see radio_constants.c)
 * \retval airTime               Time-on-air in hs_timer ticks
 */
uint32_t  radio_get_symbol_toa_hs(uint16_t length, uint8_t modulation);
/*!
 * \brief Calculates the time-on-air of the preamble
 * \param [IN] length            Number of symbols (LoRa) / Bytes (FSK)
 * \param [IN] modulation        Modulation index (see radio_constants.c)
 * \retval airTime               Time-on-air in hs_timer ticks
 */
uint32_t  radio_get_preamble_toa_hs(uint16_t length, uint8_t modulation);


void      radio_print_message(radio_message_t* message);

void      radio_set_cad(uint8_t modulation, bool rx, bool use_timeout);
void      radio_set_continuous_preamble(void);
bool      radio_is_channel_free(uint8_t modulation, uint32_t timeout_ms);

uint32_t  radio_get_last_pkt_snr(void);
int32_t   radio_get_rssi(void);

RadioState_t          radio_get_status(void);       // returns the current radio driver status
RadioOperatingModes_t radio_get_chipmode(void);     // reads the current chip mode from the radio (sends a radio command)
void                  radio_print_status(void);

uint32_t  radio_get_error_count(void);    // returns the error counter of the radio driver (bus access and command errors)
uint16_t  radio_get_error_flags(void);    // returns the hardware error flags from the radio chip (see datasheet p.98)

/*!
 * \brief Get time-on-air for arbitrary radio settings.
 * (arb = arbitrary radio settings)
 *
 * \param [IN] modem        Radio modem to be used [0: FSK, 1: LoRa]
 * \param [IN] bandwidth    Sets the bandwidth (DSB)
 *                          FSK : >= 2600 and <= 250000 Hz
 *                          LoRa: [0: 125 kHz, 1: 250 kHz,
 *                                 2: 500 kHz, 3: Reserved]
 * \param [IN] datarate     Sets the Datarate
 *                          FSK : 600..300000 bits/s
 *                          LoRa: [6: 64, 7: 128, 8: 256, 9: 512,
 *                                10: 1024, 11: 2048, 12: 4096  chips]
 * \param [IN] coderate     Sets the coding rate (LoRa only)
 *                          FSK : N/A ( set to 0 )
 *                          LoRa: [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
 * \param [IN] preambleLen  Sets the Preamble length
 *                          FSK : Number of bytes
 *                          LoRa: Length in symbols (the hardware adds 4 more symbols)
 * \param [IN] fixLen       Fixed length packets [0: variable, 1: fixed]
 * \param [IN] payloadLen   Sets payload length (in Bytes) when fixed length is used
 * \param [IN] crcOn        Enables/Disables the CRC [0: OFF, 1: ON]
 *
 * \retval airTime          Computed airTime (us) for the given packet payload length
 */
uint32_t  radio_get_toa_arb(RadioModems_t modem, uint32_t bandwidth,
                          uint32_t datarate, uint8_t coderate,
                          uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                          bool crcOn );
/**
* \brief            Get time-on-air for specified flora modulation.
*
* \param            payload_len: Number of payload Bytes from the upper layer
* \param            modulation: flora modulation (see radio_constants.c)
* \retval           Time-on-air in us
*/
uint32_t  radio_get_toa(uint8_t payload_len, uint8_t modulation);
/**
* \brief            Get time-on-air for specified flora modulation.
*
* \param            payload_len: Number of payload Bytes from the upper layer
* \param            modulation: flora modulation (see radio_constants.c)
* \retval           Time-on-air in hs_timer ticks
*/
uint32_t  radio_get_toa_hs(uint8_t payload_len, uint8_t modulation);

/**
 * \brief           Calculate the required RX bandwidth for a given frequency and TX bandwidth
 */
uint32_t  radio_get_rx_bandwidth(uint32_t freq, uint32_t tx_bandwidth);


#endif /* RADIO_RADIO_HELPERS_H_ */
