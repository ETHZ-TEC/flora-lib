/*
 * dozer.h
 *
 *  Created on: Jun 22, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_DOZER_H_
#define PROTOCOL_DOZER_DOZER_H_

/*
 * Typedefs
 */

#ifndef DOZER_ENABLE
#define DOZER_ENABLE    0
#endif /* DOZER_ENABLE */


typedef enum {
    STATE_IDLE,                          //
    STATE_BEACON_SENT,                   //
    STATE_BEACON_SENT_OVERHEARING,       //
    STATE_AWAIT_CONNECTION_REQUEST,      //
    STATE_CONNECTION_REQUEST_SENT,       //
    STATE_HANDSHAKE_TRANSITION,          //
    STATE_HANDSHAKE_SENT,                //
    STATE_AWAIT_HANDSHAKE,               //
    STATE_AWAIT_BEACON,                  //
    STATE_ACTIVATION_TRANSITION,         //
    STATE_CONNECTION_REQUEST_TRANSITION, //
    STATE_AWAIT_RSSI,                    //
    STATE_SEND_BUFFER,                   //
    STATE_SEND_BUFFER_OVERHEARING,       //
    STATE_RECEIVE_BUFFER,                //
}current_state_t;


typedef enum {
  BOOTSTRAP_STATE_OFF,
  BOOTSTRAP_STATE_WAITING,
  BOOTSTRAP_STATE_ACTIVITY_DETECTED,
} bootstrap_state_t;


typedef struct child {
  uint64_t last_rendezvous_time;
  uint16_t id;             // ID of the child
  uint16_t last_seq_nr;        // sequence number of the last sent data message
  uint16_t slot;            // allocated slot
  bool packets_received;        // indicates if data messages have been received
} child_t;

typedef struct parent {
  uint64_t last_rendezvous_time;     // time of the last communication with this node
  uint64_t time_till_next_beacon;    // time till the next beacon
  uint32_t rating;           // the virtual link quality rating
  uint32_t last_estimation_error;   // absolute time how much the last beacon arrival estimation was off
  uint32_t last_seed;          // the seed number included in the last overheard beacon
  int16_t drift_ppm;          // the current drift estimation
  int16_t rssi;             // last measured rssi value of the beacon // TODO: not used at the moment as the radio always returns rssi=0
  uint16_t id;             // TOS Id of the parent
  uint8_t last_load;           // the load indicated in the last received beacon
  uint8_t distance_to_sink;       // hops to the sink
  uint8_t failed_connection_attempts; // number of unsuccessful connection/transfer attempts
  uint8_t slot;            // only used for the active parent. Indicates the reserved upload slot
} parent_t;


typedef struct node_config {
  uint16_t id;      // node ID
  uint8_t role;      // node role; 1=sink, 0=normal node
  uint8_t slot;      // slot allocated from parent
  uint8_t child_count;  // number of connected children
  uint8_t parent_count;  // number of potential parents stored
} node_config_t;

typedef struct radio_config {
  uint8_t modulation_index;
  uint8_t band_index;
  uint8_t power;
} radio_config_dozer_t;


/* include all dozer related files */
#include "protocol/dozer/dozer_messages.h"
#include "protocol/dozer/dozer_config.h"
#include "protocol/dozer/dozer_constants.h"
#include "protocol/dozer/dozer_radio.h"
#include "protocol/dozer/dozer_utils.h"
#include "protocol/dozer/dozer_topology.h"
#include "protocol/dozer/dozer_radio_admin.h"
#include "protocol/dozer/timer_queue.h"
#include "protocol/dozer/circular_queue.h"


/*
 * Functions
 */
void radio_dep_init();
void dozer_run(bool start);



/********************************* Variables *********************************/
char char_buff[100]; // used for serial prints

extern node_config_t node_config;   // node information
extern radio_config_dozer_t radio_config; // radio configurations

#define STM32_UID ((uint32_t *)0x1FFF7590) // 32 bit node id



#endif /* PROTOCOL_DOZER_DOZER_H_ */
