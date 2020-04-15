/*
 * lora_helpers.c
 *
 *  Created on: Apr 10, 2018
 *      Author: marku
 */

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "main.h"

#include "radio/radio_helpers.h"
#include "radio/radio_platform.h"
#include "radio/radio_constants.h"
#include "radio/flora_radio.h"
#include "radio/semtech/radio.h"
#include "radio/semtech/sx126x/sx126x.h"
#include "radio/semtech/boards/sx126x-board.h"
#include "cli/cli.h"
#include "time/hs_timer.h"

extern volatile bool radio_receive_continuous;
extern volatile bool radio_command_scheduled;
extern bool radio_disable_log;
extern radio_sleeping_t radio_sleeping;

extern bool cli_interactive_mode;

extern SX126x_t SX126x;


// See datasheet, https://www.semtech.com/uploads/documents/an1200.22.pdf, or http://www.sghoslya.com/p/lora_6.html for formula

//                            SF12    SF11    SF10    SF9   SF8    SF7   SF6    SF5
static const double radio_lora_symb_times[3][8] = {{ 32.768, 16.384, 8.192, 4.096, 2.048, 1.024, 0.512, 0.256 },  // 125 kHz
                                                { 16.384, 8.192,  4.096, 2.048, 1.024, 0.512, 0.256, 0.128 },  // 250 kHz
                          { 8.192,  4.096,  2.048, 1.024, 0.512, 0.256, 0.128, 0.064 }}; // 500 kHz

volatile static uint8_t lora_last_payload_size = 0;
volatile static uint8_t current_modulation = 0;
volatile static uint8_t current_band = 0;

volatile static int32_t override_preamble_length = -1;

static void radio_execute();

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

void radio_set_payload(uint8_t* buffer, uint8_t offset, uint8_t size) {
  radio_set_packet_params_and_size(size);
  SX126xSetPayload(buffer, size);
}

void radio_set_payload_while_transmit(uint8_t* buffer, uint8_t offset, uint8_t size) {
  if (size > 0) {
    uint8_t margin = 16;
    uint32_t preamble_toa = radio_get_preamble_toa(SX126x.PacketParams.Params.LoRa.PreambleLength);
    uint32_t header_toa = radio_get_header_toa();
    uint32_t start_time = (uint32_t) hs_timer_get_schedule_timestamp() + preamble_toa + header_toa + RADIO_TIME_STBY_RC_TO_TX * HS_TIMER_FREQUENCY_US;
    volatile uint32_t toa = radio_calculate_message_toa(current_modulation, lora_last_payload_size, -1) - preamble_toa - header_toa - RADIO_TIME_STBY_RC_TO_TX * HS_TIMER_FREQUENCY_US; // - lora_get_header_toa(current_modulation);
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



void radio_set_config_tx(uint8_t modulation_index, uint8_t band_index, int8_t power, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, bool implicit, bool crc) {
  current_modulation = modulation_index;
  current_band = band_index;

  if (preamble_length == -1) {
    preamble_length = radio_modulations[current_modulation].preambleLen;
  }

  // Do not allow for illegal power level
  if (radio_bands[current_band].maxPower < power) {
    power = radio_bands[current_band].maxPower;
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
  Radio.SetChannel(radio_bands[current_band].centerFrequency);
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

  set_radio_conf(modulation_index, power);
}

void radio_set_config_rx(uint8_t modulation_index, uint8_t band_index, int32_t bandwidth, int32_t bitrate, int32_t preamble_length, uint16_t timeout, bool implicit, uint8_t implicit_length, bool crc, bool stop_rx_on_preamble) {
  // deafult value of "stop_rx_on_preamble": true

  current_modulation = modulation_index;
  current_band = band_index;

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
  uint32_t afc_bandwidth = 2 * (bandwidth / 2 + radio_bands[current_band].centerFrequency) / RADIO_CLOCK_DRIFT;
  int32_t bandwidth_rx = bandwidth + afc_bandwidth;

  if (bitrate == -1) {
    bitrate = radio_modulations[current_modulation].datarate;
  }

  Radio.Standby();

  Radio.SetChannel(radio_bands[current_band].centerFrequency);

  Radio.SetRxConfig(
      radio_modulations[current_modulation].modem,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? bandwidth : bandwidth_rx,
      bitrate,
      (radio_modulations[current_modulation].modem == MODEM_LORA) ? radio_modulations[current_modulation].coderate : 0,
      0, // AFC Bandwidth (FSK only, not used with SX126x!)
      preamble_length + 2, // Offset due to safety (preamble detector counter would reset otherwise). RxTimeout is not inflicted.
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

void radio_set_config_rxtx(uint8_t modulation, uint32_t freq, int32_t datarate, int8_t power, int32_t bandwidth, int32_t preamble_length, uint8_t coderate, uint16_t timeout, bool implicit,  uint8_t implicit_length, bool crc) {
  // determine fdev from bandwidth and datarate
  // NOTE: according to the datasheet afc_bandwidth (automated frequency control bandwidth) variable represents the frequency error (2x crystal frequency error)
  uint32_t fdev = 0;
  int32_t bandwidth_rx = 0;
  if (modulation == 1) {
    uint32_t afc_bandwidth = 2 * (bandwidth / 2 + freq) / RADIO_CLOCK_DRIFT;
    fdev = (bandwidth - datarate) / 2;
    bandwidth_rx = bandwidth + afc_bandwidth;
  }

  Radio.Standby();
  Radio.SetChannel(freq);

  Radio.SetTxConfig(
      (modulation == 0) ? MODEM_LORA : MODEM_FSK,   // modem [0: FSK, 1: LoRa]
      power,                                        // power [dBm]
      (modulation == 1) ? fdev : 0,                 // frequency deviation (FSK only)
      (modulation == 0) ? bandwidth : 0,            // bandwidth (LoRa only)
      datarate,                                      // datarate (FSK: bits/s, LoRa: spreading-factor)
      (modulation == 0) ? coderate : 0,              // coderate (LoRa only)
      preamble_length,                              // preamble length (FSK: num bytes, LoRa: symbols (HW adds 4 symbols))
      implicit,                                     // Fixed length packets [0: variable, 1: fixed]
      crc,                                          // CRC on
      false,                                        // no FHSS
      0,                                            // no FHSS
      0,                                            // no FHSS
      0                                             // no timeout
  );

  Radio.SetRxConfig(
      (modulation == 0) ? MODEM_LORA : MODEM_FSK,   // modem [0: FSK, 1: LoRa]
      (modulation == 0) ? bandwidth : bandwidth_rx,  // bandwidth
      datarate,                                     // datarate (FSK: bits/s, LoRa: spreading-factor)
      (modulation == 0) ? coderate : 0,             // coderate (LoRa only)
      0,                                            // AFC Bandwidth (FSK only, not used with SX126x!)
      preamble_length,                              // preamble length (FSK: num bytes, LoRa: symbols (HW adds 4 symbols))
      timeout,                                      // RxSingle timeout value
      implicit,                                     // Fixed length packets [0: variable, 1: fixed]
      (implicit ? implicit_length : 0),             // no fixed payload length (as it is explicit header mode / variable packet length)
      crc,                                          // CRC on
      false,                                        // no FHSS
      0,                                            // no FHSS
      false,                                        // no FHSS
      false                                         // not continuous rx
  );

  SX126xSetStopRxTimerOnPreambleDetect(true);
}

void radio_transmit(uint8_t* buffer, uint8_t size, bool schedule)
{
  if (schedule) {
    radio_command_scheduled = true;
    radio_set_payload(buffer, 0, size);
    SX126xSetTxWithoutExecute(0);
  }
  else {
    Radio.Send(buffer, size);
    hs_timer_set_schedule_timestamp((uint32_t) hs_timer_get_current_timestamp());
  }
}

void radio_transmit_at_precise_moment(uint8_t* buffer, uint8_t size, uint32_t time)
{
  radio_command_scheduled = true;
  radio_set_payload(buffer, 0, size);
  SX126xSetTxWithoutExecute(0);

  hs_timer_schedule(time, &radio_execute);
}

void radio_execute_manually(int64_t timer) {
  if (radio_command_scheduled) {
    if (timer == -1) {
      hs_timer_set_schedule_timestamp((uint32_t) hs_timer_get_current_timestamp());
      radio_execute();
    }
    else {
      hs_timer_schedule(timer, &radio_execute);
    }
  }

  return;
}

void radio_set_tx(uint64_t timestamp)
{
  radio_command_scheduled = true;
  SX126xSetTxWithoutExecute(0);
  hs_timer_schedule(timestamp, &radio_execute);
}

void radio_set_rx(uint64_t timestamp, uint32_t timeout)
{
  radio_command_scheduled = true;
  SX126xSetRxBoostedWithoutExecute(timeout);
  hs_timer_schedule(timestamp, &radio_execute);
  radio_start_mcu_timeout(timestamp);
}

void radio_set_continuous_preamble() {
  SX126xWriteCommand(RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0 );
}


void radio_retransmit_at_precise_moment(uint8_t* overwrite_buffer, uint8_t overwrite_size, uint8_t size, uint64_t time)
{
  radio_command_scheduled = true;
  radio_set_packet_params_and_size(size);
  SX126xSetPayload(overwrite_buffer, overwrite_size);
  SX126xSetTxWithoutExecute(0);

  hs_timer_schedule(time, &radio_execute);
}


uint32_t radio_calculate_timeout(bool preamble) {
  if (preamble) {
    if (override_preamble_length != -1) {
      return ((uint64_t) radio_get_preamble_toa(override_preamble_length))  * 1000 / HS_TIMER_FREQUENCY_US / RADIO_TIMER_PERIOD_NS;
    }
    else {
      return ((uint64_t) radio_get_preamble_toa(0)) * 1000 / HS_TIMER_FREQUENCY_US / RADIO_TIMER_PERIOD_NS;
    }
  }
  else {
    return radio_calculate_message_toa(current_modulation, 0, override_preamble_length) * 2.0 + (85.2 + 100.0) * HS_TIMER_FREQUENCY_US;
  }
}

void radio_receive_and_execute(bool boost, uint32_t schedule_timer) {
  radio_receive_continuous = false;
  radio_set_mcu_timeout(radio_calculate_timeout(false));
  radio_command_scheduled = true;

  uint32_t hardware_timeout = radio_calculate_timeout(true);

  if (boost) {
    SX126xSetRxBoostedWithoutExecute(hardware_timeout);
  }
  else {
    SX126xSetRxWithoutExecute(hardware_timeout);
  }

  radio_execute_manually(schedule_timer);
}

void radio_receive(bool schedule, bool boost, uint32_t timeout, uint32_t rx_timeout) {
  // default value for "rx_timeout": 0
  radio_receive_continuous = !schedule;

  if (schedule) {
    if (timeout) {
      radio_set_mcu_timeout(timeout  + (uint32_t) (85.2 * HS_TIMER_FREQUENCY_US));
    }
    else {
      radio_set_mcu_timeout(radio_calculate_timeout(false));
    }

    if (boost) {
      if(rx_timeout) {
        SX126xSetRxBoostedWithoutExecute(rx_timeout);
      }
      else {
        SX126xSetRxBoostedWithoutExecute(radio_calculate_timeout(true));
      }

    }
    else {
      if(rx_timeout) {
        SX126xSetRxWithoutExecute(rx_timeout);
      }
      else {
        SX126xSetRxWithoutExecute(radio_calculate_timeout(true));
      }
    }

    radio_command_scheduled = true;
  }
  else {
    if (boost) {
      SX126xSetRxBoosted(timeout);
    }
    else {
      SX126xSetRx(timeout);
    }
  }
}


void radio_sync_receive() {
  radio_receive_continuous = false;
  SX126xSetRxBoosted(0);
}



void radio_receive_duty_cycle(uint32_t rx, uint32_t sleep, bool schedule) {
  radio_receive_continuous = false;

  rx = (uint64_t) rx; // in 15.625 us steps
  sleep = (uint64_t) sleep; // in 15.625 us steps (see Figure 13-2: "RX Duty Cycle Energy Profile" in SX1262 datasheet)

  if (schedule) {
    SX126xSetRxDutyCycleWithoutExecute(rx, sleep);
    radio_command_scheduled = true;
  }

  else {
    Radio.SetRxDutyCycle(rx, sleep);
  }

}


uint16_t radio_get_preamble_length_from_duration(uint16_t duration, uint8_t modulation) {
  uint16_t preamble_length;

  if (duration && radio_modulations[modulation].modem == MODEM_LORA) {
    preamble_length = duration / radio_lora_symb_times[radio_modulations[modulation].bandwidth][12 - radio_modulations[modulation].datarate];
  }
  else {
    preamble_length = radio_modulations[modulation].preambleLen;
  }

  return preamble_length;
}

uint32_t radio_get_symbol_toa(uint16_t length, uint8_t modulation) {
  if (modulation < 8) {
    return length * radio_lora_symb_times[radio_modulations[modulation].bandwidth][12 - radio_modulations[modulation].datarate] * HS_TIMER_FREQUENCY_MS;
  }
  else {
    return rint( 8 * ( length / radio_modulations[modulation].datarate ) * HS_TIMER_FREQUENCY);
  }
}

uint32_t radio_get_header_toa() {
  return radio_calculate_message_toa(current_modulation, 0, -1);
}

uint32_t radio_get_preamble_toa(uint16_t length) {
  return radio_get_preamble_toa_for_modulation(length, current_modulation);
}

uint32_t radio_get_preamble_toa_for_modulation(uint16_t length, uint8_t modulation) {
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

void radio_set_standby() {
  Radio.Standby();
  radio_sleeping = FALSE;
}

inline uint32_t radio_lookup_toa(uint8_t modulation, uint8_t size) {
  return radio_toas[modulation][size];
}

uint32_t radio_calculate_message_toa(uint8_t modulation, uint8_t size, int32_t preamble) {
  uint32_t airTime = 0;

  if (preamble == -1) {
      preamble = radio_modulations[modulation].preambleLen;
  }

  switch( radio_modulations[modulation].modem )
  {
  case MODEM_FSK:
    {
       airTime = rint( ( 8 * ( ( preamble ) +
                   ( SX126x.PacketParams.Params.Gfsk.SyncWordLength >> 3 ) +
                   ( ( SX126x.PacketParams.Params.Gfsk.HeaderType == RADIO_PACKET_FIXED_LENGTH ) ? 0.0 : 1.0 ) +
                   size +
                   ( ( size && SX126x.PacketParams.Params.Gfsk.CrcLength == RADIO_CRC_2_BYTES && SX126x.PacketParams.Params.Gfsk.HeaderType != RADIO_PACKET_FIXED_LENGTH) ? 2.0 : 0 ) ) /
                   SX126x.ModulationParams.Params.Gfsk.BitRate ) * HS_TIMER_FREQUENCY);
    }
    break;
  case MODEM_LORA:
    {
      double ts = radio_get_symbol_toa(1, modulation);
      double tPreamble = radio_get_preamble_toa(preamble);
      double tmp = (
        (
          ceil(
            (
                8 * size
                - 4 * SX126x.ModulationParams.Params.LoRa.SpreadingFactor
                + 28
                + (16 * SX126x.PacketParams.Params.LoRa.CrcMode)
                - ((SX126x.PacketParams.Params.LoRa.HeaderType == LORA_PACKET_FIXED_LENGTH ) ? 20 : 0)
            )
            / (double)(
                4 * (
                    SX126x.ModulationParams.Params.LoRa.SpreadingFactor
                    - ((SX126x.ModulationParams.Params.LoRa.LowDatarateOptimize > 0) ? 2 : 0)
                  )
            )
          )
        )
        * ((SX126x.ModulationParams.Params.LoRa.CodingRate % 4 ) + 4 )
      );

      double nPayload = 8 + ((tmp > 0) ? tmp : 0);
      double tPayload = nPayload * ts;
      double tOnAir = tPreamble + tPayload;

      airTime = tOnAir;
    }
    break;
  }
  return airTime;
}

inline void radio_set_packet_params_and_size(uint8_t size)
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


void radio_print_message(radio_message_t* message) {
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

void radio_set_cad_params(bool rx, bool use_timeout) {
  radio_cad_params_t params = radio_cad_params[current_modulation];

  if (use_timeout) {
    uint32_t timeout = ((uint64_t) radio_calculate_message_toa(current_modulation, 0, -1) * 1000 / HS_TIMER_FREQUENCY_US / RADIO_TIMER_PERIOD_NS);
    SX126xSetCadParams(params.symb_num, params.cad_det_peak, params.cad_det_min, rx, timeout);
  }
  else {
    SX126xSetCadParams(params.symb_num, params.cad_det_peak, params.cad_det_min, rx, 0);
  }
}

void radio_set_cad() {
  SX126xSetCad();
}


static void radio_execute() {
  radio_set_nss_pin();
  radio_stop_schedule();
  radio_command_scheduled = false;

  switch (Radio.GetStatus()) {
    case RF_RX_RUNNING:
      RADIO_RX_START_IND();
      break;
    case RF_TX_RUNNING:
      RADIO_TX_START_IND();
      break;
    default:
      break;
  }


}

void set_radio_log(bool enable) {
  radio_disable_log = !enable;
}
