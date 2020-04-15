/*
 * dozer_radio_admin.c
 *
 *  Created on: Jun 28, 2018
 *      Author: kelmicha
 */


#include "dozer_radio_admin.h"


/********************************* Variables *********************************/
node_config_t node_config;

// timestamps to manage receive timeouts
static uint64_t wait_for_con_req_start_ts;
static uint64_t start_oh_ts;
static uint64_t stop_oh_ts;


// current state
current_state_t current_state = STATE_IDLE;


// overhearing related variables
uint8_t overhearing = 0; // do we overhear a message
uint64_t overhear_time; // length of overhearing phase
message_type_t overhearing_message_type; // the type of messages accepted in the overhearing mode




uint16_t beacon_address; // address of the next awaited beacon (corresponds to the address of the currently connected parent)

uint8_t send_activation = 0; // indicates if an activation frame should be sent after beacon reception

uint16_t num_msgs; // number of messages to be sent to the parent

uint16_t actual_msgs; // number of messages actually sent/received to/from  parent/child

uint16_t comm_partner_id; // parent ID

// last sequence number of this child
uint16_t* last_seq_nr_ptr;
uint16_t last_seq_nr;


uint8_t duplicate_cnt; // counts data messeges that are received twice

// message buffers
dozer_send_message_t smbuf;
dozer_send_message_t* data_msg_buf = &smbuf;
dozer_send_message_t* beacon_ptr;
dozer_message_t* curr_beacon_ptr;


uint8_t buffer_length; // length of the data message

uint8_t activation_frame_pl[] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}; // payload of the activation frame


// acknowledgment control variables
uint8_t cnt_ack_timeout = 0;
bool send_buffer_acked;
uint16_t last_ack_byte;

// statistics related variables
uint16_t txcount=0;
uint16_t rxcount=0;
bool is_radio_on;
uint64_t radio_on = 0; // number of jiffies the radio is activated
uint64_t radio_off = 0; // number of jiffies the radio is shut down
uint64_t radio_ts = 0; // timestamp for a radio switch event. used for both, radio on and off








/*
 *  callback after a successful transmission
 */
void dozer_tx_done(uint64_t tx_ts) {

    txcount++;
    switch (current_state) { // check the current state
        case STATE_BEACON_SENT:
            current_state = STATE_AWAIT_RSSI;
      dozer_print(6, "cs await rssi");

      // check for channel activity after a beacon was sent
      get_rssi(RSSI_LISTEN_TIME_IN_JIFFIES);

            beacon_sent(tx_ts, SUCCESS);
            break;

        case STATE_ACTIVATION_TRANSITION:
            // handle the beacon to the upper layer and wait for a connection request
            current_state = STATE_CONNECTION_REQUEST_TRANSITION;
            dozer_print(6, "cs con req trans");
            // shut radio down until we send the connection request
            radio_shut_down();
            received_bm(curr_beacon_ptr);
            break;

        case STATE_CONNECTION_REQUEST_SENT:
            current_state = STATE_AWAIT_HANDSHAKE;
            dozer_print(6, "cs await hs");

            // connection request sent -> wait for handshake
            dozer_receive(HANDSHAKE_TIMEOUT, 0);
            break;

        case STATE_HANDSHAKE_SENT:
            if (node_config.role) { // the sink accepts multiple new children per interval
                current_state = STATE_AWAIT_CONNECTION_REQUEST;
                dozer_print(6, "cs await con req");

                uint32_t rest_timeout = (uint32_t) ((hs_timer_get_current_timestamp() - wait_for_con_req_start_ts) / HS_TIMER_FREQUENCY_MS) - CONNECTION_REQ_TIMEOUT_IN_MS;
                if (rest_timeout > 0) { // continue listening for connection requests if CONNECTION_REQ_TIMEOUT has not expired yet
                  dozer_receive(rest_timeout, 0);
                }
            }
            else { // all other nodes only accept one new child per interval
                radio_reset_state();
            }
            handshake_sent(SUCCESS);
            break;

        case STATE_SEND_BUFFER:
          // buffer was sent -> wait for acknowledgment
          set_await_ack(true);
      dozer_receive(ACK_RECEIVE_TIMEOUT_IN_MS, 0);
          break;

        default:
          dozer_print(10, "tx done: wrong state!");
            break;
    }
}


/*
 *  callback after a successful message receive
 */
void dozer_rx_done(dozer_message_t* message, uint16_t size) {

  dozer_header_t* msg_header = &(message->header);

    rxcount++;

    switch(current_state) { // check the currrent state
         case STATE_AWAIT_BEACON:
            dozer_print(2, "rx state await bm");
            if (msg_header->type != BEACON_MSG){
                dozer_print(2, "rx callback, state await beacon: wrong type!");
            }
            else if ( msg_header->source != beacon_address ){
                // unexpected beacon arrived -> signal received_overhear_bm
                received_overhear_bm(message);
            }
            else {
                if (send_activation) {
                    // send activation frame
                    send_activation = 0;
                    if (send_activation_frame() == SUCCESS) {
                        current_state = STATE_ACTIVATION_TRANSITION;
                        dozer_print(6, "cs activation trans");
                        curr_beacon_ptr = message;
                        return;
                    }
                }
                // handle the received beacon
                received_bm(message);
            }
            radio_reset_state();
            break;

        case STATE_AWAIT_CONNECTION_REQUEST:
            dozer_print(2, "rx state await con req");
            if (msg_header->type != CONNECTION_REQ_MSG) {
                dozer_print(2, "rx callback, state con req: wrong type!");
                radio_reset_state();
            }
            else {
                current_state = STATE_HANDSHAKE_TRANSITION;
                dozer_print(6, "cs hs trans");

                if (!node_config.role) {
                  // the sink accepts multiple new children per interval
                  // stop the timer for the other nodes
                    tq_stop_timer(CON_REQ_TIMER);
                }

                // handle the received connection request
                received_crm(message);
            }
            break;

        case STATE_AWAIT_HANDSHAKE:
            dozer_print(2, "rx state await hs");
      radio_reset_state();
            if (msg_header->type != HANDSHAKE_MSG) {
                dozer_print(2, "rx callback, state await hs: wrong type!");
            }
            else {
              // handle the handshake
                received_handshake(message);
            }
            break;

        case STATE_RECEIVE_BUFFER:
          dozer_print(2, "rx state receive buffer");
            if (msg_header->type != DATA_MSG) {
              dozer_receive(DATA_TIMEOUT_IN_MS, 0);
                return;
            }

            if (DOZER_REMOVE_DUPLICATES) {
        // check for duplicate
        if ((last_seq_nr_ptr != NULL) && ((data_msg_t *)message)->seqNr == last_seq_nr) {
          dozer_receive(2*DATA_TIMEOUT_IN_MS, 0);
          duplicate_cnt++;
          return;
        }
            }

            actual_msgs++;

      sprintf(char_buff, "data_msg rec; seqNr: %d", message->payload.data_msg.seqNr);
      dozer_print(6, char_buff);

      // try to append received message to queue
            if (cq_append(0, msg_header->type, size-DATA_MSG_HEADER_SIZE, (data_msg_t*) &(message->payload.data_msg)) == SUCCESS) {
              sprintf(char_buff, "dm appended: %X", msg_header->ack);
              dozer_print(5, char_buff);

              if (--num_msgs > 0 && (msg_header->ack & 0x2) != 0x2) { // if second bit (stop bit) is set, child has no more messages
                    last_seq_nr = message->payload.data_msg.seqNr;
                    last_seq_nr_ptr = &last_seq_nr;

                    print_msg(message, 3);

                    // set ack byte to tell child how many messages can still be received
                    set_ack_byte(num_msgs - 1);

                    dozer_receive(DATA_TIMEOUT_IN_MS, 0);
          dozer_print(5, "wait for next msg");
          print_ts(5);

                    return;
                }
            }

            radio_reset_state();

            if (duplicate_cnt > 0) {
              dozer_print(0, "duplicate received");
            }

            // download finished
            receive_messages_to_buffer_done(actual_msgs, last_seq_nr);
            return;
            break;

        case STATE_IDLE:
            dozer_print(0, "rx cs idle");
            if (overhearing) {
                // if the message fits the requested message type or we are just listening for anything
                if ((msg_header->type  == overhearing_message_type) || (overhearing_message_type == 0)) {
                    stop_overhearing();
                    // handle the overheared message
                    received_overhear_bm(message);
                }
            }
            else {
                // normally the radio should be switched off!!! -> should not happen
                radio_shut_down();
            }
            break;

        default:
            dozer_print(10, "rx done: wrong state!");
            break;
    }
}




/*
 *  callback if a transmit failed
 */
void dozer_tx_timeout() {
    dozer_print(0, "tx timeout");

    switch (current_state) { // check state
        case STATE_BEACON_SENT:
            radio_reset_state();
            beacon_sent(0, ERROR);
            break;

        case STATE_CONNECTION_REQUEST_SENT:
            radio_reset_state();
            connect_to_parent();
            break;

        case STATE_HANDSHAKE_SENT:
            if (node_config.role) {
                current_state = STATE_AWAIT_CONNECTION_REQUEST;
                dozer_print(6, "cs await con req");
            }
            else {
                radio_reset_state();
            }
            handshake_sent(ERROR);
            break;

        default:
          dozer_print(10, "tx timeout: wrong state!");
            break;
    }
}


/*
 *  callback in case of a receive timeout
 */
void dozer_rx_timeout() {
    dozer_print(0, "rx timeout");

    if (overhearing) {
        // stop listening for random beacons
        reset_listening_for_beacons();
        stop_overhearing();
        return;
    }

    switch (current_state) { // check state
        case STATE_AWAIT_BEACON:
            radio_reset_state();
      // return null to signal the timeout
      received_bm(NULL);
            break;

        case STATE_AWAIT_CONNECTION_REQUEST:
          radio_reset_state();
          break;

        case STATE_AWAIT_HANDSHAKE:
            radio_reset_state();
            // return null to signal the timeout
            received_handshake(NULL);
            break;

        case STATE_SEND_BUFFER:
          handle_ack(ERROR, 0);
          break;

        case STATE_RECEIVE_BUFFER:
          dozer_print(6, "rx timeout; rec buffer");
            radio_reset_state();
            receive_messages_to_buffer_done(actual_msgs, last_seq_nr);
            break;

        default:
          dozer_print(10, "rx timeout: wrong state!");
            break;
    }

}




/*
 * send beacon message
 * returns SUCCESS if successful and ERROR otherwise
 */
uint8_t send_beacon(dozer_send_message_t* msg) {
    if (current_state == STATE_IDLE) {
        msg->header.ack &= 0xFE;
        if (overhearing) {
            // if the radio is already up we need to wait a little bit
            current_state = STATE_BEACON_SENT_OVERHEARING;
            dozer_print(6 , "cs beacon sent oh");
            beacon_ptr = msg;
            tq_start_timer(WAIT_TIMER, hs_timer_get_current_timestamp() + WAKEUP_TIME_IN_JIFFIES, &wait_timer_fired);
            return SUCCESS;
        }

        if (dozer_send(msg, BEACON_SIZE, 0) == SUCCESS) {
            current_state = STATE_BEACON_SENT;
            dozer_print(6 , "cs beacon sent");
            return SUCCESS;
        }
        else {
          dozer_print(6, "beacon send failed");
            radio_reset_state();
            return ERROR;
        }
    }
    // not in IDLE state, can't send beacon
  dozer_print(10, "send beacon: wrong state!");
    return ERROR;
}

/*
 * send activation frame
 * returns SUCCESS if successful and ERROR otherwise
 */
uint8_t send_activation_frame() {
  dozer_print(5, "send act");
    dozer_send_message_t msg;
    activation_frame_t af;

//    af.load= activation_frame_pl; // TODO: enable for single rssi sniff
    msg.header.dest = beacon_address;
    msg.header.source = node_config.id;
    msg.header.type = ACTIVATION_MSG;
    msg.header.ack = 0;
    msg.payload.act_frame = af;

    return dozer_send(&msg, ACTIVATION_FRAME_SIZE, 0);
}


/*
 * send connection request
 * returns SUCCESS if successful and ERROR otherwise
 */
uint8_t send_connection_request(dozer_send_message_t * msg) {
    if (current_state == STATE_CONNECTION_REQUEST_TRANSITION) {
        if (msg != NULL) {
            current_state = STATE_CONNECTION_REQUEST_SENT;
            sprintf(char_buff, "cs con req sent: dest: %d", msg->header.dest);
            dozer_print(6, char_buff);

            msg->header.ack &= 0xFE;
            if (dozer_send(msg, CON_REQ_SIZE, 0) == SUCCESS) {
                return SUCCESS;
            }
        }
        radio_reset_state();
        return ERROR;
    }
    dozer_print(10, "send con req: wrong state!");
    return ERROR;
}


/*
 * send handshaek
 * returns SUCCESS if successful and ERROR otherwise
 */
uint8_t send_handshake(dozer_send_message_t* msg) {
    if (current_state == STATE_HANDSHAKE_TRANSITION) {

        if (msg != NULL) {
            current_state = STATE_HANDSHAKE_SENT;
            sprintf(char_buff, "cs hs sent: dest: %d", msg->header.dest);
            dozer_print(6, char_buff);
            msg->header.ack &= 0xFE;
            if (dozer_send(msg, HANDSHAKE_SIZE, 0) == SUCCESS) {
                return SUCCESS;
            }
        }
        radio_reset_state();
        return ERROR;
    }
    dozer_print(10, "send handshake: wrong state!");
    return ERROR;
}





/*
 * start overhearing
 */
void start_overhearing(uint8_t continued)  {
    dozer_print(0, "start overhearing");
    if (!overhearing) {
        overhearing = 1;
        dozer_print(6, "start overhearing");
        if (current_state == STATE_IDLE) {
            if (continued) { // if something was received an overhearing should be continued
                overhear_time = overhear_time - (stop_oh_ts - start_oh_ts)/HS_TIMER_FREQUENCY_MS; // compensate for already overheard time
            }
            // start up radio
            start_oh_ts = hs_timer_get_current_timestamp();
            if (dozer_receive(overhear_time, 0) == SUCCESS) {}
        }
    }
}


/*
 * stop overhearing
 */
void stop_overhearing() {
    stop_oh_ts = hs_timer_get_current_timestamp();
    if (overhearing) {
        overhearing = 0;
        dozer_print(6, "stop overhearing");

        if (current_state == STATE_IDLE) {
            // shut down radio
            radio_shut_down();
        }
    }
}

/*
 * set expected message type and time for overhearing
 */
void set_overhear_params(message_type_t oh_msg_type, uint32_t oh_time) {
    if (oh_time) { // only set if not 0, to allow for continued overhearing
        overhear_time = oh_time;
    }
    overhearing_message_type = oh_msg_type;
}

/*
 * listen for beacon from (potential) parent
 */
uint8_t listen_to_beacon(uint16_t address, uint32_t timeout, uint8_t activation) {
    dozer_print(0, "listen to beacon");
    if (current_state == STATE_IDLE) {
        // start up radio
        if (dozer_receive(timeout/HS_TIMER_FREQUENCY_MS, 0) == SUCCESS) {
            current_state = STATE_AWAIT_BEACON;
            dozer_print(6, "cs await bm");
            beacon_address = address;
            send_activation = (activation & 0x1) == 1;
            return SUCCESS;
        }
    }
    dozer_print(10, "listen to beacon: wrong state!");
    return ERROR;
}



/*
 * reset the radio state and shut down radio if not overhearing
 */
void radio_reset_state() {
    current_state = STATE_IDLE;
    dozer_print(6, "cs idle; radio reset");

    if (!overhearing) {
        // shut down radio
        radio_shut_down();
    }
}



/************************** radio statistics ******************************/
uint64_t compute_intervall() {
    uint64_t delta;
    uint64_t current_time = hs_timer_get_current_timestamp();
  if (!radio_ts) { // initialize radio_ts the first time we get here.
    radio_ts = current_time;
  }
    if (current_time >= radio_ts) {
        delta = current_time - radio_ts;
    } else {
        // timer overflow
        delta = (~radio_ts)+current_time+1;
    }
    radio_ts = current_time;
    return delta;
}

void radio_stats_off() {
    if (is_radio_on) {
        radio_on += compute_intervall();
        is_radio_on = false;
    }
}

void radio_stats_on() {
    if (!is_radio_on) {
        radio_off += compute_intervall();
        is_radio_on = true;
    }
}
/********************************************************/

/*
 * wait timer fired, used to wait a short time if radio is overhearing
 */
void wait_timer_fired() {
    switch (current_state) {
        case STATE_BEACON_SENT_OVERHEARING:
            if (dozer_send(beacon_ptr, BEACON_SIZE, 0) == SUCCESS) {
                current_state = STATE_BEACON_SENT;
                dozer_print(6, "cs beacon sent");
            }
            else {
                radio_reset_state();
                beacon_sent(0, ERROR);
            }
            break;

        case STATE_SEND_BUFFER_OVERHEARING:
      if (dozer_send(data_msg_buf, buffer_length, 0) == SUCCESS) {
        sprintf(char_buff, "buf sent, seqNr: %d", data_msg_buf->payload.data_msg.seqNr);
        dozer_print(7, char_buff);
        current_state = STATE_SEND_BUFFER;
        dozer_print(6, "cs send buffer");
      }
      else {
        radio_reset_state();
        send_buffered_messages_done(actual_msgs, send_buffer_acked);
      }
      break;

        default:
            break;
    }
}

/*
 * channel activity detection finished
 */
void rssi_done(bool channel_busy) {
    if (channel_busy) {
      if (current_state == STATE_AWAIT_RSSI || current_state == STATE_AWAIT_CONNECTION_REQUEST) {
      current_state = STATE_AWAIT_CONNECTION_REQUEST;
      dozer_print(6, "cs await con req");

      wait_for_con_req_start_ts = hs_timer_get_current_timestamp();
      dozer_receive(CONNECTION_REQ_TIMEOUT_IN_MS, 0);
      }
    }
    else {
        radio_reset_state();
    }
}


/*
 * download data messages from child
 */
void receive_messages_to_buffer(uint8_t num, uint16_t id, uint16_t * l_seq_nr) {
  dozer_print(5, "rec msgs to buffer");
  print_ts(5);
    if (current_state != STATE_IDLE) {
        dozer_print(0, "rec_msg: wrong state!");
    }
    else {
        current_state = STATE_RECEIVE_BUFFER;
        dozer_print(6, "cs receive buffer");
        actual_msgs = 0;

        // receive as much messages as possible
        num_msgs = cq_cntFreeBuffer();
        if (num != 0 && num < num_msgs) {
            num_msgs = num;
        }

        comm_partner_id = id;
        last_seq_nr_ptr = l_seq_nr;
        duplicate_cnt = 0;

        if (last_seq_nr_ptr != NULL) {
            last_seq_nr = *last_seq_nr_ptr;
        }

        sprintf(char_buff, "rec num: %d", num_msgs);
        dozer_print(1, char_buff);
        // if no slots in queue -> 0xFFFF indicates full queue
        set_ack_byte(num_msgs - 1);

        // start up radio
        if (dozer_receive(DATA_TIMEOUT_IN_MS, 0) == SUCCESS) {}
    }
}


/*
 * copy received message and metadata to buffer
 */
void assign_payload(dozer_message_t* msg, dozer_send_message_t* send_msg, uint16_t size) {
  sprintf(char_buff, "assign pl size: %d", size);
  dozer_print(0, char_buff);

  memmove(msg, send_msg, size);
}


/*
 * upload data messages to parent
 */
uint8_t send_buffered_messages(uint8_t num, uint16_t id) {
  dozer_print(5, "send buffered msgs");
  print_ts(5);
  uint8_t num_used_buf;
  if (current_state != STATE_IDLE) {
    dozer_print(5, "dozer send buffered msgs: wrong state!");
  }
  else {
    actual_msgs = 0;
    cnt_ack_timeout = 0;
    send_buffer_acked = false;
    num_used_buf = cq_cntUsedBuffer();
    if (num == 0) { // TODO: clean up if / else statement
      // send all available packets
      num_msgs = num_used_buf;
    }
    else {
      if (num < num_used_buf) {
        num_msgs = num;
      }
      else {
        num_msgs = num_used_buf;
      }
    }

    comm_partner_id = id;
    last_ack_byte = 0xffff;

    if (fill_buffer(num_msgs) == SUCCESS) { //get next message
      data_msg_buf->header.ack |= 0x01;

      if (overhearing) {
        // if the radio is already up we need to wait a little bit
        current_state = STATE_SEND_BUFFER_OVERHEARING;
        dozer_print(6 , "cs send buffer oh");
        tq_start_timer(WAIT_TIMER, WAKEUP_TIME_IN_JIFFIES, &wait_timer_fired);

        return SUCCESS;
      }
      current_state = STATE_SEND_BUFFER;
      dozer_print(6 , "cs send buffer");
      if (dozer_send(data_msg_buf, buffer_length, 0) == SUCCESS) {
        sprintf(char_buff, "buf sent, seqNr: %d, num: %d", data_msg_buf->payload.data_msg.seqNr, (int) num);
        dozer_print(7, char_buff);
        return SUCCESS;
      }
    }
    radio_reset_state();
    return ERROR;
  }
  return ERROR;
}


/*
 * get first message from data queue
 */
uint8_t fill_buffer(uint8_t num_msgs_remaining) {
  dozer_print(0, "fill buffer");
  uint8_t res;
  uint8_t d_type;
  uint8_t payload_len;
  dozer_header_t* header = &(data_msg_buf->header);

  res = cq_getFirst((data_msg_t*) &(data_msg_buf->payload.data_msg), &d_type, &payload_len);
  if (res == SUCCESS) {
    header->dest = comm_partner_id;
    header->source = node_config.id;
    header->type = d_type;
    header->ack = 0x01;

    if (num_msgs_remaining == 1){
      header->ack |= 0x02; // stop bit
    }
    buffer_length = payload_len + DATA_MSG_HEADER_SIZE;
  }
  else {
    dozer_print(5, "fill buf failed");
  }
  return res;
}

/*
 * handle received acknowledgment or continue if no acknowledgment received
 */
void handle_ack(uint8_t res, uint8_t ack_byte) {
  switch (current_state) {
    case STATE_SEND_BUFFER:
      if (res == SUCCESS) {
        send_buffer_acked = true;
        //last packet was successfully sent and acknowledged
        cnt_ack_timeout = 0;
        // receiver has no more slots in the queue abort transmission without dropping the data message
        if (ack_byte != 0xFF) {

          last_ack_byte = ack_byte;
          actual_msgs++;
          cq_dropFirst();

          if (--num_msgs > 0 && ack_byte > 0) {
            if (fill_buffer(num_msgs) == SUCCESS) {
              rtc_delay(1); // TODO: check if needed

              if (dozer_send(data_msg_buf, buffer_length, 0) == SUCCESS) {
                sprintf(char_buff, "buf sent, seqNr: %d", data_msg_buf->payload.data_msg.seqNr);
                dozer_print(7, char_buff);
                return;
              }
            }
          }
        }
      }
      else { // no ack was received
        cnt_ack_timeout++;
        if ((cnt_ack_timeout < NUM_ACK_TIMEOUT)) {
          if (--num_msgs > 0) {
            if (fill_buffer(num_msgs) == SUCCESS) {
              if (dozer_send(data_msg_buf, buffer_length, 0) == SUCCESS) {
                return;
              }
            }
          }
        }
      }
      radio_reset_state();

      // finish upload
      send_buffered_messages_done(actual_msgs, send_buffer_acked);
    break;

    default:
        dozer_print(10, "handle ack: wrong state!");
      break;
  }
}


/*
 * print radio statistics
 */
void print_radio_stats() {
  sprintf(char_buff, "radio on: %d; radio off: %d", (int)radio_on, (int)radio_off);
  dozer_print(5, char_buff);
}


/*
 * return true if radio is in IDLE state, false otherwise
 */
bool radio_is_idle() {
  return current_state == STATE_IDLE;
}
