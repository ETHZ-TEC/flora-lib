/*
 * radio_helpers.c
 *
 *  Created on: Apr 10, 2018
 *      Author: marku
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
volatile static int32_t override_preamble_length = -1;
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


void radio_set_payload(uint8_t* buffer, uint8_t size)
{
  radio_set_packet_params_and_size(size);
  SX126xWriteBuffer(0, buffer, size);
}


// the last chunk of the payload must be the one with the largest offset
void radio_set_payload_chunk(uint8_t* buffer, uint8_t offset, uint8_t size, bool last_chunk)
{
  if (buffer && size) {
    if (last_chunk) {
      radio_set_packet_params_and_size(offset + size);
    }
    SX126xWriteBuffer(offset, buffer, size);
  }
}


void radio_set_payload_while_transmit(uint8_t* buffer, uint8_t size)
{
  if (size > 0) {
    uint8_t margin = 16;
    uint32_t preamble_toa = radio_get_preamble_toa(SX126x.PacketParams.Params.LoRa.PreambleLength, current_modulation);
    uint32_t header_toa = radio_get_toa_hs(0, current_modulation);
    uint32_t start_time = (uint32_t) hs_timer_get_schedule_timestamp() + preamble_toa + header_toa + RADIO_TIME_STBY_RC_TO_TX * HS_TIMER_FREQUENCY_US;
    volatile uint32_t toa = radio_get_toa_hs(lora_last_payload_size, current_modulation) - preamble_toa - header_toa - RADIO_TIME_STBY_RC_TO_TX * HS_TIMER_FREQUENCY_US; // - lora_get_header_toa(current_modulation);
    uint32_t toa_per_byte = toa / lora_last_payload_size;

    radio_set_packet_params_and_size(size);

    while (true) {
      volatile uint32_t difference = ((uint32_t) hs_timer_get_current_timestamp()) - start_time;
      if (difference < ((uint32_t) 2^31)) {
        break;
      }
    }

    uint16_t i = 0;
    while (i < size)
    {
      volatile uint32_t current_time = (uint32_t) hs_timer_get_current_timestamp();
      volatile uint32_t bytes = (current_time - start_time) / toa_per_byte;

      if ((current_time - start_time) > toa) {
        SX126xWriteBuffer(i, buffer + i, size - i);
        break;
      }
      else if (bytes >= (i + margin)){
        uint8_t size_to_transfer = ((size - i) >= margin) ? margin : (size-i);
        SX126xWriteBuffer(i, buffer + i, size_to_transfer);
        i += size_to_transfer;
      }
      else {
        __NOP();
      }
    }
  }
  else {
    radio_set_packet_params_and_size(0);
  }
}


void radio_set_config_tx(uint8_t modulation_index,
                         uint8_t band_index,
                         int8_t power,
                         int32_t bandwidth,     // DSB
                         int32_t bitrate,
                         int32_t preamble_length,
                         bool implicit,
                         bool crc)
{
  if (modulation_index >= RADIO_NUM_MODULATIONS) return;

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

  if (bitrate == -1) {
    bitrate = radio_modulations[current_modulation].datarate;
  }
  else {
    fdev = (bandwidth - bitrate) / 2;
  }

  Radio.Standby();
  Radio.SetChannel(radio_bands[band_index].centerFrequency);
  Radio.SetTxConfig(
      radio_modulations[current_modulation].modem,
      power,
      (radio_modulations[current_modulation].modem == MODEM_FSK) ? fdev : 0, // FSK frequency deviation
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? bandwidth : 0,
      bitrate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      preamble_length,
      implicit, // explicit header mode
      crc, // CRC on
      false, // no FHSS
      0, // no FHSS
      0, // no FHSS
      0 // no timeout
  );
}


void radio_set_config_rx(uint8_t modulation_index,
                         uint8_t band_index,
                         int32_t bandwidth,           // DSB
                         int32_t bitrate,
                         int32_t preamble_length,
                         uint16_t timeout,
                         bool implicit,
                         uint8_t implicit_length,
                         bool crc,
                         bool stop_rx_on_preamble)
{
  if (modulation_index >= RADIO_NUM_MODULATIONS) return;

  current_modulation = modulation_index;

  if (preamble_length == -1) {
    preamble_length = radio_modulations[current_modulation].preambleLen;
    override_preamble_length = preamble_length;
  }
  else {
    override_preamble_length = -1;
  }

  if (bandwidth == -1) {
    bandwidth = radio_modulations[current_modulation].bandwidth;
  }
  uint32_t afc_bandwidth = 2 * (bandwidth / 2 + radio_bands[band_index].centerFrequency) / RADIO_CLOCK_DRIFT;
  int32_t bandwidth_rx = bandwidth + afc_bandwidth;

  if (bitrate == -1) {
    bitrate = radio_modulations[current_modulation].datarate;
  }

  Radio.Standby();

  Radio.SetChannel(radio_bands[band_index].centerFrequency);

  Radio.SetRxConfig(
      radio_modulations[current_modulation].modem,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? bandwidth : bandwidth_rx,
      bitrate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      0, // AFC Bandwidth (FSK only, not used with SX126x!)
      preamble_length,
      timeout,
      implicit, // explicit header mode
      (implicit ? implicit_length : 0), // no fixed payload length (as it is explicit header mode / variable packet length)
      crc, // CRC on
      false, // no FHSS
      0, // no FHSS
      false, // no FHSS
      false // not continuous rx
  );

  SX126xSetStopRxTimerOnPreambleDetect(stop_rx_on_preamble);
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
                           uint8_t implicit_length,
                           bool crc)
{
  // determine fdev from bandwidth and datarate
  // NOTE: according to the datasheet afc_bandwidth (automated frequency control bandwidth) variable represents the frequency error (2x crystal frequency error)
  uint32_t fdev = 0;
  int32_t  bandwidth_rx = 0;
  uint32_t freq = radio_bands[band_index].centerFrequency;
  if (!lora_mode) {
    uint32_t afc_bandwidth = 2 * (bandwidth / 2 + freq) / RADIO_CLOCK_DRIFT;
    fdev = (bandwidth - datarate) / 2;
    bandwidth_rx = bandwidth + afc_bandwidth;
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
      implicit,                               // Fixed length packets [0: variable, 1: fixed]
      crc,                                    // CRC on
      false,                                  // no FHSS
      0,                                      // no FHSS
      0,                                      // no FHSS
      0                                       // no timeout
  );

  Radio.SetRxConfig(
      (lora_mode) ? MODEM_LORA : MODEM_FSK,   // modem [0: FSK, 1: LoRa]
      (lora_mode) ? bandwidth : bandwidth_rx, // bandwidth
      datarate,                               // datarate (FSK: bits/s, LoRa: spreading-factor)
      (lora_mode) ? coderate : 0,             // coderate (LoRa only)
      0,                                      // AFC Bandwidth (FSK only, not used with SX126x!)
      preamble_length,                        // preamble length (FSK: num bytes, LoRa: symbols (HW adds 4 symbols))
      timeout,                                // RxSingle timeout value
      implicit,                               // Fixed length packets [0: variable, 1: fixed]
      (implicit ? implicit_length : 0),       // no fixed payload length (as it is explicit header mode / variable packet length)
      crc,                                    // CRC on
      false,                                  // no FHSS
      0,                                      // no FHSS
      false,                                  // no FHSS
      false                                   // not continuous rx
  );

  SX126xSetStopRxTimerOnPreambleDetect(true);
}


// calculate the RX timeout in hs ticks
uint32_t radio_calculate_timeout(bool preamble)
{
  if (preamble) {
    if (override_preamble_length != -1) {
      return ((uint64_t) radio_get_preamble_toa(override_preamble_length, current_modulation))  * 1000 / HS_TIMER_FREQUENCY_US / RADIO_TIMER_PERIOD_NS;
    }
    else {
      return ((uint64_t) radio_get_preamble_toa(0, current_modulation)) * 1000 / HS_TIMER_FREQUENCY_US / RADIO_TIMER_PERIOD_NS;
    }
  }
  else {
    return radio_get_toa_hs(0, current_modulation) * 2.0 + (85.2 + 100.0) * HS_TIMER_FREQUENCY_US;
  }
}


// symbol TOA in hs ticks
uint32_t radio_get_symbol_toa(uint16_t length, uint8_t modulation)
{
  if (modulation >= RADIO_NUM_MODULATIONS) return 0;

  if (radio_modulations[modulation].modem == MODEM_LORA) {
    return length * radio_lora_symb_times[radio_modulations[modulation].bandwidth][12 - radio_modulations[modulation].datarate] * HS_TIMER_FREQUENCY_MS;
  }
  else {
    return rint( 8 * ( length / radio_modulations[modulation].datarate ) * HS_TIMER_FREQUENCY);
  }
}


uint32_t radio_get_preamble_toa(uint16_t length, uint8_t modulation)
{
  if (modulation >= RADIO_NUM_MODULATIONS) return 0;

  if (!length) {
    length = radio_modulations[modulation].preambleLen;
  }

  if (radio_modulations[modulation].modem == MODEM_LORA) {
    if (radio_modulations[modulation].datarate == 6 || radio_modulations[modulation].datarate == 5) {
      return radio_get_symbol_toa(length + 6.25, modulation);
    }
    else {
      return radio_get_symbol_toa(length + 4.25, modulation);
    }
  }
  else {
    return rint((double) 8 * (length + 3.0) / (double) radio_modulations[modulation].datarate * HS_TIMER_FREQUENCY);
  }
}


void radio_set_packet_params_and_size(uint8_t size)
{
  lora_last_payload_size = size;

  if( SX126xGetPacketType( ) == PACKET_TYPE_LORA ) {
    SX126x.PacketParams.Params.LoRa.PayloadLength = size;
  }
  else {
    SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
  }

  SX126xSetPacketParams( &SX126x.PacketParams );
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
      uint32_t timeout = ((uint64_t) radio_get_toa(0, modulation)*1000U / RADIO_TIMER_PERIOD_NS);
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
    radio_cad_params_t params = radio_cad_params[modulation];
    SX126xSetCadParams(params.symb_num, params.cad_det_peak, params.cad_det_min, LORA_CAD_ONLY, 0);   // no timeout needed since we do not want to receive the packet
    radio_set_cad_callback(radio_channel_free_cb);
    channel_free = 0xff;
    SX126xSetCad();
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


RadioOperatingModes_t radio_get_opmode(void)
{
  return SX126xGetOperatingMode();
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
                          radio_modulations[modulation].coderate - 4,
                          radio_modulations[modulation].preambleLen,
                          false, // fixLen
                          payload_len,
                          true   // crcOn
  );
}


uint32_t radio_get_toa_hs(uint8_t payload_len, uint8_t modulation)
{
  return ((uint64_t) radio_get_toa(payload_len, modulation))*HS_TIMER_FREQUENCY/1000000UL;
}
