/*
 * dozer_messages.h
 *
 *  Created on: 21.6.2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_DOZER_MESSAGES_H
#define PROTOCOL_DOZER_DOZER_MESSAGES_H


#ifndef MAX_DATA_LENGTH
    #define MAX_DATA_LENGTH    (23)
#endif


typedef enum message_type{
    ACTIVATION_MSG     = 0x30 | PROTOCOL_ID_DOZER,
    ACK_MSG            = 0x40 | PROTOCOL_ID_DOZER,

    BEACON_MSG         = 0x50 | PROTOCOL_ID_DOZER,
    CONNECTION_REQ_MSG = 0x60 | PROTOCOL_ID_DOZER,
    HANDSHAKE_MSG      = 0x70 | PROTOCOL_ID_DOZER,

    DATA_MSG           = 0x80 | PROTOCOL_ID_DOZER,
} dozer_message_type_t;




typedef struct __attribute__((__packed__, __aligned__(1))) {
    dozer_message_type_t type : 8;  // type of the message payload
    uint8_t source;      // message source / sender
    uint8_t dest;      // message destination
    uint8_t ack;      // the lsb is used to signal that an ack is requested; the second bit to signal that no more data messages will be sent
} dozer_header_t;

typedef struct dozer_metadata {
    uint64_t msg_rec_ts;  // time when the message was received
    uint16_t size;      // size of the payload
    int16_t rssi;      // measured rssi value during reception
    int8_t snr;        // measured snr during reception
} dozer_metadata_t;

typedef struct data_msg { // sent from child to parent with final destination sink.
    uint32_t aTime;     // generatio time  // TODO: increase to 64bit without assigning e.x. slot to later byte? maybe 2x32bit?
    uint16_t seqNr;      // sequence number of the data message
    uint16_t originatorID;  // ID of the node that generated this data message
    uint8_t data[MAX_DATA_LENGTH]; // actual data payload
} data_msg_t;

typedef struct beacon_msg {
    uint32_t seed;       // seed used to determine the next beacon time
    uint16_t lsd;       // local sender delay :-). The duration the timer fires too late in regard to the expected time.
    uint8_t hop_count;     // number of hops to the sink
    uint8_t load;       // the current load of the node
} beacon_msg_t;

typedef struct activation_frame {
//  uint8_t* load; // TODO: enable for single rssi sniff
} activation_frame_t;

typedef struct connection_req_msg {
} connection_req_msg_t;

typedef struct handshake_msg {
    uint8_t slot;      // slot number allocated for that child
} handshake_msg_t;

typedef struct ack_msg {
    uint8_t ack_byte;    // byte used to tell child how many messages can still be received
} ack_msg_t;

typedef union payload {
    data_msg_t data_msg;
    beacon_msg_t beacon_msg;
    handshake_msg_t handshake_msg;
    ack_msg_t ack_msg;
    activation_frame_t act_frame;
    connection_req_msg_t con_req_msg;
} payload_t;

typedef struct dozer_send_message {
    dozer_header_t header;
    payload_t payload;
} dozer_send_message_t;

typedef struct dozer_message {
    dozer_header_t header;
    payload_t payload;
    dozer_metadata_t metadata;
} dozer_message_t;


enum {
    BEACON_SIZE = 12,
    ACTIVATION_FRAME_SIZE = 12,
    CON_REQ_SIZE = 4,
    HANDSHAKE_SIZE = 5,
    ACK_SIZE = 5,
  DATA_MSG_SIZE = 36,
}; // TODO: check message sizes


#endif /* PROTOCOL_DOZER_DOZER_MESSAGES_H */
