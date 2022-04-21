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

#include "flora_lib.h"


extern volatile bool radio_irq_direct;
extern bool          cli_interactive_mode;
extern bool          cli_initialized;
extern SX126x_t      SX126x;
extern const struct  Radio_s Radio;


// See datasheet, https://www.semtech.com/uploads/documents/an1200.22.pdf, or http://www.sghoslya.com/p/lora_6.html for formula

//                                                    SF12    SF11    SF10    SF9   SF8    SF7    SF6    SF5
static const double radio_lora_symb_times[3][8] = { { 32.768, 16.384, 8.192, 4.096, 2.048, 1.024, 0.512, 0.256 },   // 125 kHz
                                                    { 16.384, 8.192,  4.096, 2.048, 1.024, 0.512, 0.256, 0.128 },   // 250 kHz
                                                    { 8.192,  4.096,  2.048, 1.024, 0.512, 0.256, 0.128, 0.064 } }; // 500 kHz

volatile static uint8_t lora_last_payload_size = 0;
volatile static uint8_t current_modulation = 0;
volatile static uint8_t channel_free = 0;       // set in the CAD callback

radio_message_t* last_message_list = NULL;


void radio_update_cli()
{
  if (!radio_irq_direct && cli_initialized) {
    Radio.IrqProcess();
  }

  radio_message_t* tmp = last_message_list;

  while (tmp != NULL) {
    radio_message_t* tmp_next;
    radio_print_message(tmp);
    tmp_next = tmp->next;
    free(tmp->payload);
    free(tmp);
    tmp = tmp_next;
  }

  last_message_list = NULL;
}


void radio_set_lora_syncword(radio_lora_syncword_t syncword)
{
  // Set whitening factor to custom value
  SX126xWriteRegister( REG_LR_SYNCWORD, (syncword >> 8) & 0xFF);
  SX126xWriteRegister( REG_LR_SYNCWORD + 1, syncword & 0xFF);
}


uint16_t radio_get_syncword()
{
  uint16_t syncword;
  syncword = SX126xReadRegister( REG_LR_SYNCWORD) << 8;
  syncword |= SX126xReadRegister( REG_LR_SYNCWORD + 1);
  return syncword;
}


void radio_get_payload(uint8_t* buffer, uint8_t* offset, uint8_t* size)
{
  SX126xGetRxBufferStatus( size, offset);
  SX126xReadBuffer( *offset, buffer, *size);
}


uint8_t radio_get_payload_size()
{
  uint8_t offset = 0;
  uint8_t size = 0;
  SX126xGetRxBufferStatus( &size, &offset);

  return size;
}


void radio_set_payload_size(uint8_t size)
{
  lora_last_payload_size = size;
  Radio.SetMaxPayloadLength(size);
}


void radio_set_payload(uint8_t* buffer, uint8_t size)
{
  radio_set_payload_size(size);
  if (buffer && size) {
    SX126xWriteBuffer(0, buffer, size);
  }
}


// the last chunk of the payload must be the one with the largest offset
void radio_set_payload_chunk(uint8_t* buffer, uint8_t offset, uint8_t size, bool last_chunk)
{
  if (last_chunk) {
    radio_set_payload_size(offset + size);
  }
  if (buffer && size) {
    SX126xWriteBuffer(offset, buffer, size);
  }
}


void radio_set_payload_while_transmit(uint8_t* buffer, uint8_t size)
{
  if (size > 0) {
    uint8_t  margin       = 16;
    uint32_t preamble_toa = radio_get_preamble_toa_hs(SX126x.PacketParams.Params.LoRa.PreambleLength, current_modulation);
    uint32_t header_toa   = radio_get_toa_hs(0, current_modulation);
    uint32_t overhead     = preamble_toa + header_toa + RADIO_TIME_STBY_RC_TO_TX * HS_TIMER_FREQUENCY_US;
    uint32_t start_time   = (uint32_t) hs_timer_get_schedule_timestamp() + overhead;
    uint32_t toa          = radio_get_toa_hs(lora_last_payload_size, current_modulation) - overhead;
    uint32_t toa_per_byte = toa / lora_last_payload_size;

    radio_set_payload_size(size);

    // busy wait for the start time
    int32_t difference;
    do {
      difference = (int32_t)(((uint32_t) hs_timer_get_current_timestamp()) - start_time);
    } while (difference < 0);

    uint16_t i = 0;
    while (i < size)
    {
      volatile uint32_t current_time = (uint32_t) hs_timer_get_current_timestamp();
      volatile uint32_t sent_bytes   = (current_time - start_time) / toa_per_byte;

      if ((current_time - start_time) > toa) {
        SX126xWriteBuffer(i, buffer + i, size - i);
        break;
      }
      else if (sent_bytes >= (i + margin)) {
        uint8_t size_to_transfer = ((size - i) >= margin) ? margin : (size - i);
        SX126xWriteBuffer(i, buffer + i, size_to_transfer);
        i += size_to_transfer;
      }
    }
  }
  else {
    radio_set_payload_size(0);
  }
}


void radio_set_config_tx(uint8_t modulation_index,
                         uint8_t band_index,
                         int8_t power,
                         int32_t bandwidth,     // DSB
                         int32_t datarate,
                         int32_t preamble_length,
                         bool implicit,
                         bool crc)
{
  if (modulation_index >= RADIO_NUM_MODULATIONS || band_index >= RADIO_NUM_BANDS) return;

  current_modulation = modulation_index;

  if (preamble_length == -1) {
    preamble_length = radio_modulations[current_modulation].preambleLen;
  }

  // Do not allow for illegal power level
  if (radio_bands[band_index].maxPower < power) {
    power = radio_bands[band_index].maxPower;
  }

  if (bandwidth == -1) {
    bandwidth = radio_modulations[current_modulation].bandwidth;
  }

  uint32_t fdev = radio_modulations[current_modulation].fdev;

  if (datarate == -1) {
    datarate = radio_modulations[current_modulation].datarate;
  }
  else {
    fdev = (bandwidth - datarate) / 2;
  }

  Radio.Standby();
  Radio.SetChannel(radio_bands[band_index].centerFrequency);
  Radio.SetTxConfig(
      radio_modulations[current_modulation].modem,
      power,
      (radio_modulations[current_modulation].modem == MODEM_FSK) ? fdev : 0, // FSK frequency deviation
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? bandwidth : 0,
      datarate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      preamble_length,
      implicit,
      crc,
      false,    // FHSS
      0,        // FHSS period
      false,    // iqInverted
      0         // timeout
  );
}


void radio_set_config_rx(uint8_t modulation_index,
                         uint8_t band_index,
                         int32_t bandwidth,           // DSB
                         int32_t datarate,
                         int32_t preamble_length,
                         uint16_t timeout,
                         bool implicit,
                         uint8_t implicit_len,
                         bool crc,
                         bool stop_rx_on_preamble)
{
  if (modulation_index >= RADIO_NUM_MODULATIONS || band_index >= RADIO_NUM_BANDS) return;

  current_modulation = modulation_index;

  if (preamble_length == -1) {
    preamble_length = radio_modulations[current_modulation].preambleLen;
  }
  if (bandwidth == -1) {
    bandwidth = radio_modulations[current_modulation].bandwidth;
  }

  int32_t bandwidth_rx = radio_get_rx_bandwidth(radio_bands[band_index].centerFrequency, bandwidth);

  if (datarate == -1) {
    datarate = radio_modulations[current_modulation].datarate;
  }

  Radio.Standby();
  Radio.SetChannel(radio_bands[band_index].centerFrequency);
  Radio.SetRxConfig(
      radio_modulations[current_modulation].modem,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? bandwidth : bandwidth_rx,
      datarate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      0,            // AFC Bandwidth (FSK only)
      preamble_length,
      timeout,
      implicit,
      (implicit ? implicit_len : RADIO_MAX_PAYLOAD_SIZE),
      crc,
      false,        // FHSS
      0,            // FHSS period
      false         // iqInverted
  );

  SX126xSetStopRxTimerOnPreambleDetect(stop_rx_on_preamble);
}


void radio_set_config(uint8_t modulation_index,
                      uint8_t band_index,
                      int8_t power,
                      uint8_t max_payload_len)
{
  if (modulation_index >= RADIO_NUM_MODULATIONS || band_index >= RADIO_NUM_BANDS) return;

  current_modulation = modulation_index;

  // Do not allow for illegal power level
  if (power > radio_bands[band_index].maxPower) {
    power = radio_bands[band_index].maxPower;
  }

  Radio.Standby();
  Radio.SetChannel(radio_bands[band_index].centerFrequency);
  Radio.SetTxConfig(
      radio_modulations[current_modulation].modem,
      power,
      (radio_modulations[current_modulation].modem == MODEM_FSK) ? radio_modulations[current_modulation].fdev : 0, // FSK frequency deviation
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].bandwidth : 0,
      radio_modulations[current_modulation].datarate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      radio_modulations[current_modulation].preambleLen,
      false,    // use explicit mode
      true,     // use crc
      false,
      0,
      false,
      0
  );

  int32_t bandwidth_rx = radio_get_rx_bandwidth(radio_bands[band_index].centerFrequency, radio_modulations[current_modulation].bandwidth);

  Radio.SetRxConfig(
      radio_modulations[current_modulation].modem,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].bandwidth : bandwidth_rx,
      radio_modulations[current_modulation].datarate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      0,
      radio_modulations[current_modulation].preambleLen,
      0,
      false,    // use explicit mode
      max_payload_len,
      true,     // use crc
      false,
      0,
      false
  );
}


void radio_set_config_rxtx(bool lora_mode,
                           uint8_t band_index,
                           int32_t datarate,
                           int8_t power,
                           int32_t bandwidth,         // DSB
                           int32_t preamble_length,
                           uint8_t coderate,
                           uint16_t timeout,
                           bool implicit,
                           uint8_t implicit_len,
                           bool crc)
{
  if (band_index >= RADIO_NUM_BANDS) return;

  // determine fdev from bandwidth and datarate
  uint32_t fdev         = 0;
  int32_t  bandwidth_rx = 0;
  uint32_t freq         = radio_bands[band_index].centerFrequency;
  if (!lora_mode) {
    fdev         = (bandwidth - datarate) / 2;
    bandwidth_rx = radio_get_rx_bandwidth(freq, bandwidth);
  }

  Radio.Standby();
  Radio.SetChannel(freq);

  Radio.SetTxConfig(
      (lora_mode) ? MODEM_LORA : MODEM_FSK,   // modem [0: FSK, 1: LoRa]
      power,                                  // power [dBm]
      (!lora_mode) ? fdev : 0,                // frequency deviation (FSK only)
      (lora_mode) ? bandwidth : 0,            // bandwidth (LoRa only)
      datarate,                               // datarate (FSK: bits/s, LoRa: spreading-factor)
      (lora_mode) ? coderate : 0,             // coderate (LoRa only)
      preamble_length,                        // preamble length (FSK: num bytes, LoRa: symbols (HW adds 4 symbols))
      implicit,                               // fixed length packets [0: variable, 1: fixed]
      crc,                                    // CRC on
      false,                                  // FHSS
      0,                                      // FHSS period
      false,                                  // iqInverted
      0                                       // timeout
  );

  Radio.SetRxConfig(
      (lora_mode) ? MODEM_LORA : MODEM_FSK,   // modem [0: FSK, 1: LoRa]
      (lora_mode) ? bandwidth : bandwidth_rx, // bandwidth
      datarate,                               // datarate (FSK: bits/s, LoRa: spreading-factor)
      (lora_mode) ? coderate : 0,             // coderate (LoRa only)
      0,                                      // AFC Bandwidth (FSK only, not used with SX126x!)
      preamble_length,                        // preamble length (FSK: num bytes, LoRa: symbols (HW adds 4 symbols))
      timeout,                                // RxSingle timeout value
      implicit,                               // fixed length packets [0: variable, 1: fixed]
      (implicit ? implicit_len : RADIO_MAX_PAYLOAD_SIZE),   // no fixed payload length (as it is explicit header mode / variable packet length)
      crc,                                    // CRC on
      false,                                  // FHSS
      0,                                      // FHSS period
      false                                   // iqInverted
  );

  SX126xSetStopRxTimerOnPreambleDetect(false);
}


// symbol TOA in hs ticks
uint32_t radio_get_symbol_toa_hs(uint16_t length, uint8_t modulation)
{
  if (modulation >= RADIO_NUM_MODULATIONS) return 0;

  if (radio_modulations[modulation].modem == MODEM_LORA) {
    return length * radio_lora_symb_times[radio_modulations[modulation].bandwidth][RADIO_LORA_SF_TO_MODULATION_INDEX(radio_modulations[modulation].datarate)] * HS_TIMER_FREQUENCY_MS;
  }
  else {
    return HS_TIMER_FREQUENCY * 8 * (uint32_t)length / radio_modulations[modulation].datarate;
  }
}


uint32_t radio_get_preamble_toa_hs(uint16_t length, uint8_t modulation)
{
  if (modulation >= RADIO_NUM_MODULATIONS) return 0;

  if (!length) {
    length = radio_modulations[modulation].preambleLen;
  }

  if (radio_modulations[modulation].modem == MODEM_LORA) {
    if (radio_modulations[modulation].datarate == 6 || radio_modulations[modulation].datarate == 5) {
      return radio_get_symbol_toa_hs(length + 6.25, modulation);
    }
    else {
      return radio_get_symbol_toa_hs(length + 4.25, modulation);
    }
  }
  else {
    return HS_TIMER_FREQUENCY * 8 * ((uint32_t)length + 3) / radio_modulations[modulation].datarate;
  }
}


void radio_print_message(radio_message_t* message)
{
  bool failure = true;

  uint8_t* payload = message->payload;
  uint16_t size = message->size;
  int16_t rssi = message->rssi;
  int8_t snr = message->snr;

  if (!cli_interactive_mode) {

    cJSON* message = cJSON_CreateObject();
    if (message == NULL) {
      goto end;
    }

    if (cJSON_AddStringToObject(message, "type", "radio_rx_msg") == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(message, "rssi", (double) rssi) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(message, "snr", (double) snr) == NULL) {
      goto end;
    }

    if (cJSON_AddNumberToObject(message, "size", (double) size) == NULL) {
      goto end;
    }

    bool is_printable = cli_string_is_printable((char*) payload, size);

    if(is_printable) {
      if (cJSON_AddStringToObject(message, "text", (char*) payload) == NULL) {
        goto end;
      }
    }
    else {

      cJSON* binary = cJSON_CreateArray();
      if (binary == NULL) {
        goto end;
      }

      cJSON_AddItemToObject(message, "bin", binary);

      int i;
      for (i = 0; i < size; i++) {
        cJSON* byte = cJSON_CreateNumber((uint8_t) payload[i]);

        if (byte == NULL) {
          goto end;
        }

        cJSON_AddItemToArray(binary, byte);
      }
    }

    failure = false;
    cli_log_json(message, "radio", CLI_LOG_LEVEL_DEBUG);

end:
    if (failure) {
      cJSON_Delete(message);
    }
  }
  else {
    char buf[128];

    cli_log("Received message!", "radio", CLI_LOG_LEVEL_INFO);

    snprintf(buf, sizeof(buf), "rssi:\t%d\r\nsnr:\t%d\r\nsize:\t%u", rssi, snr, size);
    cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, true, false, false);

    if(cli_string_is_printable((char*) payload, size)) {
      cli_log_inline("text:\t", CLI_LOG_LEVEL_DEFAULT, false, false, false);
      cli_log_inline((char*) payload, CLI_LOG_LEVEL_DEFAULT, true, false, false);
    }

    cli_log_inline("bin:\t", CLI_LOG_LEVEL_DEFAULT, true, false, false);

    int j;
    for (j = 0; j < size; j++) {
      snprintf(buf, sizeof(buf), "%02x", payload[j]);
      if(j % 8 == 7) {
        strcat(buf, "\r\n");
      }
      else {
        strcat(buf, " ");
      }

      cli_log_inline(buf, CLI_LOG_LEVEL_DEFAULT, false, false, false);
    }

    cli_log_inline("", CLI_LOG_LEVEL_DEFAULT, true, true, false);
  }

  return;
}


void radio_set_cad(uint8_t modulation, bool rx, bool use_timeout)
{
  if (modulation <= RADIO_NUM_CAD_PARAMS) {
    radio_cad_params_t params = radio_cad_params[modulation];
    if (use_timeout) {
      uint32_t timeout = ((uint64_t) radio_get_toa(0, modulation) * 1000U / RADIO_TIMER_PERIOD_NS);
      SX126xSetCadParams(params.symb_num, params.cad_det_peak, params.cad_det_min, rx, timeout);
    }
    else {
      SX126xSetCadParams(params.symb_num, params.cad_det_peak, params.cad_det_min, rx, 0);
    }
    SX126xSetCad();
  }
}


static void radio_channel_free_cb(bool detected)
{
  channel_free = !detected;
}

/* do LoRa CAD */
bool radio_is_channel_free(uint8_t modulation, uint32_t timeout_ms)
{
  if (modulation <= RADIO_NUM_CAD_PARAMS) {
    channel_free = 0xff;
    radio_set_cad_callback(radio_channel_free_cb);
    radio_set_cad(modulation, LORA_CAD_ONLY, false);      // no timeout needed since we do not want to receive the packet
    if (timeout_ms) {
      while ((channel_free == 0xff) && timeout_ms) {     // variable will be updated in the CAD done ISR
        delay_us(1000);
        timeout_ms--;
      }
    } else {
      while (channel_free == 0xff);   // variable will be updated in the CAD done ISR
    }
    radio_standby();
    return (channel_free == 1);
  }
  return false;
}


void radio_set_continuous_preamble(void)
{
  SX126xWriteCommand(RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}


uint32_t radio_get_last_pkt_snr(void)
{
  PacketStatus_t pktStatus;
  SX126xGetPacketStatus(&pktStatus);
  if (pktStatus.packetType == PACKET_TYPE_LORA) {
    return pktStatus.Params.LoRa.SnrPkt;
  }
  return 0;     // TODO implement for FSK
}


uint32_t radio_get_last_pkt_rssi(void)
{
  PacketStatus_t pktStatus;
  SX126xGetPacketStatus(&pktStatus);
  if (pktStatus.packetType == PACKET_TYPE_GFSK) {
    return pktStatus.Params.Gfsk.RssiSync;  // or: .RssiAvg
  }
  return pktStatus.Params.LoRa.RssiPkt;
}


int32_t radio_get_rssi(void)
{
  return SX126xGetRssiInst();   // instantaneous RSSI value -> radio needs to be in RX mode for this
}


RadioState_t radio_get_status(void)
{
  return Radio.GetStatus();
}


RadioOperatingModes_t radio_get_chipmode(void)
{
  uint8_t chipmode = SX126xGetStatus().Fields.ChipMode;
  switch (chipmode) {
  case 0x2:
    return MODE_STDBY_RC;
  case 0x3:
    return MODE_STDBY_XOSC;
  case 0x4:
    return MODE_FS;
  case 0x5:
    return MODE_RX;
  case 0x6:
    return MODE_TX;
  default:
    return MODE_INVALID;
  }
}


void radio_print_status(void)
{
  uint8_t  status = SX126xGetStatus().Value;
  uint8_t  opmode = SX126xGetOperatingMode();
  uint16_t irq    = SX126xGetIrqStatus();
  uint16_t errors = SX126xGetDeviceErrors().Value;
  bool     dio1   = RADIO_READ_DIO1_PIN();
  LOG_INFO("status: 0x%x  opmode: 0x%x  dio1: %u  irq: 0x%x  errors: 0x%x", status, opmode, dio1, irq, errors);
}


uint32_t radio_get_toa_arb(RadioModems_t modem, uint32_t bandwidth,
                        uint32_t datarate, uint8_t coderate,
                        uint16_t preambleLen, bool fixLen, uint8_t payloadLen,
                        bool crcOn)
{
  return Radio.TimeOnAir( modem,
                          bandwidth,
                          datarate,
                          coderate,
                          preambleLen,
                          fixLen,
                          payloadLen,
                          crcOn);
}


uint32_t radio_get_toa(uint8_t payload_len, uint8_t modulation)
{
  if (modulation >= RADIO_NUM_MODULATIONS) return 0;

  return Radio.TimeOnAir( radio_modulations[modulation].modem,
                          radio_modulations[modulation].bandwidth,
                          radio_modulations[modulation].datarate,
                          radio_modulations[modulation].coderate,
                          radio_modulations[modulation].preambleLen,
                          false, // fixLen
                          payload_len,
                          true   // crcOn
  );
}


uint32_t radio_get_toa_hs(uint8_t payload_len, uint8_t modulation)
{
  return ((uint64_t) radio_get_toa(payload_len, modulation)) * HS_TIMER_FREQUENCY / 1000000UL;
}


uint32_t radio_get_rx_bandwidth(uint32_t freq, uint32_t tx_bandwidth)
{
  // NOTE: according to the datasheet afc_bandwidth (automated frequency control bandwidth) variable represents the frequency error (2x crystal frequency error)

  return tx_bandwidth + 2 * (tx_bandwidth / 2 + freq) / (1000000 / RADIO_CLOCK_DRIFT_PPM);
}


uint32_t radio_get_error_count(void)
{
  return SX126xCheckCmdError(false);
}


uint16_t radio_get_error_flags(void)
{
  return SX126xGetDeviceErrors().Value;
}

