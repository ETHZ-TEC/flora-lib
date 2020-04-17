/* THIS FILE HAS BEEN AUTOGENERATED BY FLORA-TOOLS */

#ifndef PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_
#define PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_


#define GLORIA_NETWORK_ADDRESS 0x53

#define GLORIA_PHY_MAX_PAYLOAD 255  // max lenght of the physical layer payload (given by the hardware)
#define GLORIA_HEADER_LENGTH 4      // length of the gloria header
#define GLORIA_MAX_PAYLOAD_LENGTH  (GLORIA_PHY_MAX_PAYLOAD - GLORIA_HEADER_LENGTH)      // length of the gloria header
#define GLORIA_ACK_LENGTH 1        // length of the gloria ack message
#define GLORIA_TIMESTAMP_LENGTH 8    // length of the timestamp to send with sync floods
#define GLORIA_SCHEDULE_GRANULARITY 1  // sync timestamps get divided by the granularity. can be used to send less bytes for the sync ts.

#define GLORIA_RADIO_SLEEP_TIME 2000 // 250us      TODO: calculate min sleep time for which it's worth going into warm sleep mode
#define GLORIA_RADIO_SLEEP_TIME_COLD 240000000 // 30s  TODO: calculate min sleep time for which it's worth going into cold sleep mode
#define GLORIA_RADIO_WAKEUP_TIME 4180 // 522.500 �s;   time needed to wake up from warm sleep
#define GLORIA_RADIO_WAKEUP_TIME_COLD 36000 // 4.5 ms;   time needed to wake up from cold sleep
#define GLORIA_MIN_RX_TIME 2000   // 250 us;       min time left before rx timeout for which radio is still set in receive mode

#define GLORIA_BLACK_BOX_SYNC_DELAY 16 // 2.000 �s
#define GLORIA_GAP 7200 // 900.000 �s;           time buffer
#define GLORIA_FLOOD_FINISH_OVERHEAD 152 // 19.000 �s
#define GLORIA_RX_TRIGGER_DELAY 682 // 85.250 �s    delay after the rx command has been sent to the radio until it is executed
#define GLORIA_TX_TRIGGER_DELAY 1010 // 126.250 �s    delay after the tx command has been sent to the radio until it is executed
#define GLORIA_RX_SETUP 5326 // 665.750 �s        time needed for the rx radio setup
#define GLORIA_TX_SETUP 10083 // 1.260 ms        time needed for the tx radio setup


typedef struct {
  uint32_t slotOverhead;
  uint32_t slotAckOverhead;
  uint32_t floodInitOverhead;
  uint32_t rxOffset;
  uint32_t txSync;
} gloria_timings_t;

extern const gloria_timings_t gloria_timings[];

extern const uint8_t gloria_modulations[];
extern const int8_t gloria_powers[];
extern const uint8_t gloria_default_power_levels[];
extern const uint8_t gloria_default_retransmissions[];
extern const uint8_t gloria_default_acks[];
extern const uint8_t gloria_default_data_slots[];

#endif /* PROTOCOL_GLORIA_GLORIA_CONSTANTS_H_ */