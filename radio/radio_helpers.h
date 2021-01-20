/*
 * radio_helpers.h
 *
 * Helper functions (calculate, get, set, print)
 *
 *  Created on: Apr 10, 2018
 *      Author: marku
 */

#ifndef RADIO_RADIO_HELPERS_H_
#define RADIO_RADIO_HELPERS_H_


void      radio_update_cli(void);

void      radio_set_lora_syncword(radio_lora_syncword_t syncword);
uint16_t  radio_get_syncword(void);
void      radio_set_config_rx(uint8_t modulation_index, uint8_t band_index, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, uint16_t timeout, bool implicit, uint8_t implicit_length, bool crc, bool stop_rx_on_preamble);
void      radio_set_config_tx(uint8_t modulation_index, uint8_t band_index, int8_t power, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, bool implicit, bool crc);
void      radio_set_config_rxtx(bool lora_mode, uint8_t band_index, int32_t datarate, int8_t power, int32_t bandwidth, int32_t preamble_length, uint8_t coderate, uint16_t timeout, bool implicit,  uint8_t implicit_length, bool crc);
void      radio_set_continuous_preamble(void);

uint32_t  radio_calculate_timeout(bool preamble);     // calculates the RX timeout in hs ticks

void      radio_get_payload(uint8_t* buffer, uint8_t* offset, uint8_t* size);
uint8_t   radio_get_payload_size(void);
void      radio_set_payload(uint8_t* buffer, uint8_t size);
void      radio_set_payload_chunk(uint8_t* buffer, uint8_t offset, uint8_t size, bool last_chunk);    // the last chunk of the payload must be the one with the largest offset
void      radio_set_payload_while_transmit(uint8_t* buffer, uint8_t size);


/*!
 * \brief Calculates the time-on-air a number of symbols
 * \param [IN] length            Number of symbols (LoRa) / Bytes (FSK)
 * \param [IN] modulation        Modulation index (see radio_constants.c)
 * \retval airTime               Time-on-air in hs_timer ticks
 */
uint32_t  radio_get_symbol_toa(uint16_t length, uint8_t modulation);
/*!
 * \brief Calculates the time-on-air of the preamble
 * \param [IN] length            Number of symbols (LoRa) / Bytes (FSK)
 * \param [IN] modulation        Modulation index (see radio_constants.c)
 * \retval airTime               Time-on-air in hs_timer ticks
 */
uint32_t  radio_get_preamble_toa(uint16_t length, uint8_t modulation);


void      radio_set_packet_params_and_size(uint8_t size);

void      radio_print_message(radio_message_t* message);

void      radio_set_cad(uint8_t modulation, bool rx, bool use_timeout);
void      radio_set_continuous_preamble(void);
bool      radio_is_channel_free(uint8_t modulation, uint32_t timeout_ms);

uint32_t  radio_get_last_pkt_snr(void);
int32_t   radio_get_rssi(void);

RadioState_t          radio_get_status(void);
RadioOperatingModes_t radio_get_opmode(void);
void                  radio_print_status(void);

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



#endif /* RADIO_RADIO_HELPERS_H_ */
