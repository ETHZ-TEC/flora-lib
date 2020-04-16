/* THIS FILE HAS BEEN AUTOGENERATED BY FLORA-TOOLS */

#ifndef RADIO_RADIO_CONSTANTS_H_
#define RADIO_RADIO_CONSTANTS_H_


#define HS_TIMER_SCHEDULE_MARGIN 800 // 100.000 us

// SX1262 Timings from datasheet in us
#define RADIO_TIME_SLEEP_COLD_TO_STBY 3500 // cold start (no data retention)
#define RADIO_TIME_SLEEP_WARM_TO_STBY 340 // warm start (with data retention)
#define RADIO_TIME_STBY_RC_TO_STBY_XOSC 31
#define RADIO_TIME_STBY_RC_TO_FS 50
#define RADIO_TIME_STBY_RC_TO_RX 83
#define RADIO_TIME_STBY_RC_TO_TX 126
#define RADIO_TIME_STBY_XOSC_TO_FS 40
#define RADIO_TIME_STBY_XOSC_TO_RX 62
#define RADIO_TIME_STBY_XOSC_TO_TX 105
#define RADIO_TIME_FS_TO_RX 41
#define RADIO_TIME_FS_TO_TX 76
#define RADIO_TIME_RX_TO_FS 15
#define RADIO_TIME_RX_TO_TX 92

#define RADIO_TIME_PA_RAMP_UP 40 // us
#define RADIO_SPI_SPEED 12580000 // 12 MeBit/s

// SX1262 power values
#define RADIO_MAX_POWER 22 // dBm (SX1262)
#define RADIO_MIN_POWER -9 // dBm (SX1262)

#ifndef US915
#define RADIO_DEFAULT_BAND 48
#else
#define RADIO_DEFAULT_BAND 0
#endif

#define RADIO_CLOCK_DRIFT 100000 // 1/(+-10ppm)
#define RADIO_TIMER_PERIOD_NS 15625U
#define RADIO_TIMER_FREQUENCY 64000 // Hz

typedef enum
{
  LORA_SYNCWORD_PUBLIC = LORA_MAC_PUBLIC_SYNCWORD,
  LORA_SYNCWORD_PRIVATE = LORA_MAC_PRIVATE_SYNCWORD,
  LORA_SYNCWORD_PERMASENSE = (LORA_MAC_PUBLIC_SYNCWORD ^ 0xdada)
} radio_lora_syncword_t;

#define RADIO_FSK_SYNCWORD ( uint8_t[] ){ 0x45, 0x54, 0x0x48, 0x00, 0x00, 0x00, 0x00, 0x00 } // "ETH"

typedef struct
{
  RadioModems_t modem;
  uint32_t bandwidth;
  uint32_t datarate;
  uint8_t coderate;
  uint8_t preambleLen; // LoRa: in symbols; FSK: in Bytes (not bits!)
  uint32_t fdev;
} radio_config_t;

typedef struct
{
  uint32_t centerFrequency;
  uint32_t bandwidth;
  uint8_t dutyCycle; // in 0.1 %
  int8_t maxPower;
} radio_band_t;

/**
 * Identifies a group of bands which can be merged (for higher throughput)
 */
typedef struct
{
  // Indices
  uint8_t lower;
  uint8_t upper;
} radio_band_group_t;

typedef struct lora_cad_params_s
{
  // Indices
  RadioLoRaCadSymbols_t symb_num;
  uint8_t cad_det_peak;
  uint8_t cad_det_min;
} radio_cad_params_t;

extern const radio_config_t radio_modulations[];
extern const radio_band_t radio_bands[];
extern const radio_band_group_t radio_band_groups[];
extern const radio_cad_params_t radio_cad_params[];
extern const uint32_t radio_toas[][256];

#endif /* RADIO_RADIO_CONSTANTS_H_ */
