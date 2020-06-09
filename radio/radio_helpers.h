/*
 * lora_helpers.h
 *
 *  Created on: Apr 10, 2018
 *      Author: marku
 */

#ifndef RADIO_HELPERS_H_
#define RADIO_HELPERS_H_


void      radio_set_lora_syncword(radio_lora_syncword_t syncword);
uint16_t  radio_get_syncword();
void      radio_set_config_rx(uint8_t modulation_index, uint8_t band_index, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, uint16_t timeout, bool implicit, uint8_t implicit_length, bool crc, bool stop_rx_on_preamble);
void      radio_set_config_tx(uint8_t modulation_index, uint8_t band_index, int8_t power, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, bool implicit, bool crc);
void      radio_set_config_rxtx(uint8_t modulation, uint32_t freq, int32_t datarate, int8_t power, int32_t bandwidth, int32_t preamble_length, uint8_t coderate, uint16_t timeout, bool implicit,  uint8_t implicit_length, bool crc);
void      radio_set_continuous_preamble();
void      radio_execute_manually(int64_t timer);
void      radio_set_tx(uint64_t timestamp);
void      radio_set_rx(uint64_t timestamp, uint32_t timeout);

void      radio_transmit(uint8_t* buffer, uint8_t size, bool schedule);
void      radio_transmit_at_precise_moment(uint8_t* buffer, uint8_t size, uint32_t time);
void      radio_retransmit_at_precise_moment(uint8_t* overwrite_buffer, uint8_t overwrite_size, uint8_t size, uint64_t time);
uint32_t  radio_calculate_timeout(bool preamble);
void      radio_receive_and_execute(bool boost, uint32_t schedule_timer);
void      radio_receive(bool schedule, bool boost, uint32_t timeout, uint32_t rx_timeout);
void      radio_get_payload(uint8_t* buffer, uint8_t* offset, uint8_t* size);
uint8_t   radio_get_payload_size();
void      radio_set_payload(uint8_t* buffer, uint8_t offset, uint8_t size);
void      radio_set_payload_while_transmit(uint8_t* buffer, uint8_t offset, uint8_t size);

void      radio_sync_receive();
void      radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule);

uint16_t  radio_get_preamble_length_from_duration(uint16_t duration, uint8_t modulation);
uint32_t  radio_get_symbol_toa(uint16_t length, uint8_t modulation);
uint32_t  radio_get_header_toa();
uint32_t  radio_get_preamble_toa(uint16_t length);
uint32_t  radio_get_preamble_toa_for_modulation(uint16_t length, uint8_t modulation);
uint32_t  radio_lookup_toa(uint8_t modulation, uint8_t size);
uint32_t  radio_calculate_message_toa(uint8_t modulation, uint8_t size, int32_t preamble);

void      radio_set_packet_params_and_size(uint8_t size);

void      radio_print_message(radio_message_t* message);

void      radio_set_cad_params(bool rx, bool use_timeout);
void      radio_set_cad();

uint32_t  radio_get_snr();
int32_t   radio_get_rssi();


#endif /* RADIO_HELPERS_H_ */
