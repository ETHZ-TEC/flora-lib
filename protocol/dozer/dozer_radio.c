/*
 * dozer_radio.c
 *
 *  Created on: Jul 23, 2018
 *      Author: kelmicha
 */

#include "flora_lib.h"

#if DOZER_ENABLE


/********************************* Variables *********************************/
uint8_t dozer_timeout_type;

radio_config_dozer_t radio_config;

// timestamp variables
static uint64_t tx_start_ts;
static uint64_t tx_done_ts;
static uint64_t rx_done_ts;
static uint64_t rx_start_ts;

static uint32_t rx_timeout_time; // global rx timeout variable to allow receiving again after a message with wrong address was received


message_type_t send_message_type; // message type of the sent message

// message buffers
static dozer_message_t* message = NULL;
static dozer_header_t* msg_header = NULL;
static dozer_message_t* sent_message = NULL;


static uint16_t payload_size;


// acknowledgment related variables
volatile bool ack_sent = false;
bool ack_sending = false;
uint8_t ack_byte;
bool await_ack = false;


uint8_t rssi_detection = 0; // flag if in rssi detection

bool receive_fct = false; // variable used for debug to check if a timer fired during the receive function

// timeout count variables
static uint16_t to_fails = 0;
static uint16_t to_count = 0;


/*
 * allocate memory for sent and received message buffers
 */
void dozer_alloc_msg() {
    message = malloc(64);
    sent_message = malloc(64);
    if(message == NULL) {
        dozer_print(0, "malloc failed");
    }
    else {
        dozer_print(0, "malloc successful");
    }
    memset(message, 0, 64);
    memset(sent_message, 0, 64);
    msg_header = &(message->header);
}


/*
 * tx configuration
 */
void tx_config() {
    radio_set_config_tx(radio_config.modulation_index, radio_config.band_index, radio_config.power, -1, -1, -1, false, true);
}

/*
 * rx configuration
 */
void rx_config() {
    radio_set_config_rx(radio_config.modulation_index, radio_config.band_index, -1, -1, -1, 0, false, 0, true, false);
}

/*
 * receive function
 * timeout and delay in ms
 */
uint8_t dozer_receive(uint32_t rx_timeout, int64_t rx_delay) {
  rx_timeout_time = rx_timeout;
  receive_fct = true;


    dozer_print(3, "dozer receive");
    print_ts(1);

    if (radio_get_status()) { // check if radio idle
        sprintf(dozer_print_buffer, "receive failed, radio status: %X", radio_get_status());
        dozer_print(3 , dozer_print_buffer);
        return ERROR; // return ERROR if radio not in IDLE state
    }

    int64_t rx_del;
    if (rx_delay > 0) {
        rx_del = hs_timer_get_current_timestamp() + rx_delay*HS_TIMER_FREQUENCY_MS;
    }
    else {
        rx_del = -1; // set to -1 for lora receive function
    }

    dozer_timeout_type = DOZER_RX; // set timeout type to decide what to do in case of a timeout interrupt
    rx_config(); // set radio rx configuration

    if (rx_timeout) {
        // also set mcu timout in case the rx timeout doesn't work (rx_timeout + 500us); update doesn't seem to help
        uint32_t mcu_timeout = rx_timeout*HS_TIMER_FREQUENCY_MS + 500*HS_TIMER_FREQUENCY_US;
        // TODO: use ToA for rxTimout, i.e. rx_timeout==0
        radio_receive(true, false, mcu_timeout, rx_timeout*64); // set time to stay in rx mode / *64 because radio timer has 15.625us timesteps

        // start rx timeout watchdog timer
#ifndef DEVKIT
        tq_start_timer(RX_WATCHDOG, hs_timer_get_current_timestamp() + (rx_timeout+50)*HS_TIMER_FREQUENCY_MS, &rx_to_fail);
#else
    tim5_rx_timeout_watchdog_start(tim5_get_current_timestamp() + rx_timeout+50, &rx_to_fail);
#endif
    }
    else {
      radio_receive(false, false, 0, 0); // set rx without timeout
    }
    radio_stats_on(); // only gives useful results if rx_delay is not used
    radio_execute_manually(rx_del); // set delay for rx start

    rx_start_ts = hs_timer_get_current_timestamp() + rx_delay*HS_TIMER_FREQUENCY_MS;

    receive_fct =  false;
    dozer_print(3, "dozer receive end");
    return SUCCESS;
}

/*
 * transmit function
 * timeout in ms, set 0 to compute automatically
 * size: size of the message to be sent
 */
uint8_t dozer_send(dozer_send_message_t* msg, uint8_t size, uint32_t timeout) {
    tx_start_ts = hs_timer_get_current_timestamp();
    dozer_print(1, "dozer_send");
    print_ts(1);

    if (radio_get_status()) { // check if radio idle
        sprintf(dozer_print_buffer, "send failed, radio status: %X", radio_get_status());
        dozer_print(0, dozer_print_buffer);
        return ERROR; // return ERROR if radio not in IDLE state
    }


    dozer_timeout_type = DOZER_TX; // set timeout type to decide what to do in case of a timeout interrupt
    tx_config(); // set radio tx configuration

    // set packet size
    SX126x.PacketParams.Params.Gfsk.PayloadLength = size;
    SX126xSetPacketParams( &SX126x.PacketParams );

    uint32_t to;
    if (timeout) {
        to = timeout;
    }
    else {
      // FIXME: add all params or use function that only needs mod as additional param
        to = radio_get_toa(MODEM_FSK, size) + 10; // compute tx timeout and add 10ms
    }

    to = to * 64; // convert ms to ticks from radio (15.625us)

    // save sent message type for dozer_tx_callback function
    send_message_type = msg->header.type;

    radio_stats_on();
    SX126xSendPayload((uint8_t*) msg, size, to);

    return SUCCESS;
}


/*
 * callback function for tx interrupt from radio
 */
void dozer_tx_callback() {
  tx_done_ts = hs_timer_get_current_timestamp();
  dozer_print(1, "tx cb");
  print_ts(1);

  radio_standby();

    if(ack_sending) { // check if an acknowledgment was sent
      dozer_print(5, "ack_sent");
      print_ts(5);
      ack_sending = false;
      ack_sent = true;

      // call the rx done function with the message received before the ack was sent
      dozer_rx_done(message, payload_size);
      return;
    }

    // call the tx done function
    dozer_tx_done(tx_done_ts);
}


/*
 * callback function for rx interrupt from radio
 */
void dozer_rx_callback(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr, bool crc_error) {
    rx_done_ts = hs_timer_get_current_timestamp();
#ifndef DEVKIT
    tq_stop_timer(RX_WATCHDOG);

#else
    tim5_rx_timeout_watchdog_stop();
#endif

    sprintf(dozer_print_buffer, "rx cb, rssi: %d", rssi);
    dozer_print(5, dozer_print_buffer);
  print_ts(1);

  radio_standby(); // set radio in standby mode

  // if a message was received during channel activity detection call rssi done
    if (rssi_detection) {
        rssi_detection = 0;
        rssi_done(true);
        return;
    }

    dozer_send_message_t* tdsm = (dozer_send_message_t*) payload;
    message->header = tdsm->header;

    sprintf(dozer_print_buffer, "rx cb, ack: %d", msg_header->ack);
    dozer_print(0, dozer_print_buffer);

    // check if the message was intended for this node
    if (msg_header->dest != BROADCAST_ADDR && msg_header->dest != node_config.id) {
        sprintf(dozer_print_buffer, "rx wrong addr: %d", msg_header->dest);
        dozer_print(4, dozer_print_buffer);

        uint32_t listened_time = (hs_timer_get_current_timestamp() - rx_start_ts) / HS_TIMER_FREQUENCY_MS; // convert to ms as receive timeout is currently in ms
        // continue listening if rx timeout not yet reached
        if(listened_time < rx_timeout_time) {
      rx_timeout_time -= listened_time;
      if (dozer_receive(rx_timeout_time,  0)) {
        return;
      }
        }

        dozer_rx_timeout();
        return;
    }

    if (msg_header->ack) { // sender requested an acknowledgment
      sprintf(dozer_print_buffer, "data rec ts: %llu", rx_done_ts);
      dozer_print(0, dozer_print_buffer);
        send_ack();
    }
    else if (await_ack) { // waiting for acknowledgment
      if (msg_header->type == ACK_MSG) {

        dozer_send_message_t* dsm = (dozer_send_message_t*) payload;
        ack_byte = dsm->payload.ack_msg.ack_byte;
        sprintf(dozer_print_buffer, "ack rec: %d", ack_byte);
        dozer_print(5, dozer_print_buffer);
        print_ts(5);
        handle_ack(SUCCESS, ack_byte);
      }
      else {
        handle_ack(ERROR, 0);
      }
    }


    // copy received message payload
  assign_payload(message, tdsm, size);

    // set metadata
    message->metadata.rssi = rssi;
    message->metadata.size = size;
    message->metadata.snr = snr;
    message->metadata.msg_rec_ts = rx_done_ts;

    payload_size = size;

    // call dozer_rx_done if no ack is sent
    if(!ack_sending) {
      dozer_rx_done(message, payload_size);
    }
}


/*
 * callback function for timeout interrupts from radio
 */
void dozer_timeout_callback(bool crc_error) {
    dozer_radio_shutdown();
#ifndef DEVKIT
    tq_stop_timer(RX_WATCHDOG);
#else
    tim5_rx_timeout_watchdog_stop();
#endif

    to_count++;

    if (dozer_timeout_type == DOZER_TX) {
        dozer_tx_timeout();
    }
    else if (dozer_timeout_type == DOZER_RX) {
        dozer_rx_timeout();
    }
}


/*
 * send acknowledgment
 */
uint8_t send_ack() {
  sprintf(dozer_print_buffer, "send ack: %d", ack_byte);
  dozer_print(5, dozer_print_buffer);
    dozer_send_message_t msg;
    ack_msg_t ack;

    ack.ack_byte = ack_byte;
    msg.header.dest = msg_header->source;
    msg.header.source = node_config.id;
    msg.header.type = ACK_MSG;
    msg.header.ack = 0;
    msg.payload.ack_msg = ack;

    ack_sending = true;
    ack_sent = false;

    return dozer_send(&msg, ACK_SIZE, 0);
}


/*
 * set ack byte, that gets sent with an acknowledgment
 */
void set_ack_byte(uint8_t ab) {
  ack_byte = ab;
  sprintf(dozer_print_buffer, "ackb set: %d", ab);
  dozer_print(3, dozer_print_buffer);
}

/*
 * signal that an acknowledgment is awaited
 */
void set_await_ack(bool aw_ack) {
  await_ack = aw_ack;
}

/*
 * check for channel activity
 */
void get_rssi(uint32_t max_sense_time) {
    dozer_print(5, "get rssi");
    rssi_detection = 1;

    bool status = false;
    volatile int16_t rssi = 0;
    uint64_t carrier_sense_time = 0;

    rx_config();
    SX126xSetRx(0xFFFFFFFF);

    carrier_sense_time = hs_timer_get_current_timestamp();

    // Perform carrier sense for max_sense_time
    while( (hs_timer_get_current_timestamp() - carrier_sense_time) < max_sense_time )
    {
        rssi = SX126xGetRssiInst();
        if ( rssi > RSSI_THRESHOLD ) {
            status = true;
            break;
        }

    }

// TODO: enable for one time rssi sniff (needs less energy than coninuous sniff)
//    rtc_delay(WAIT_BEFORE_RSSI_SNIFF);
//    rssi = SX126xGetRssiInst();
//    if ( rssi > RSSI_THRESHOLD ) {
//    status = true;
//  }



  sprintf(dozer_print_buffer, "rssi det: %d", rssi);
  dozer_print(5, dozer_print_buffer);
  radio_standby();

    // if rssi detection is 0, a message has been received and the rx callback called the rssi_done function
    if (rssi_detection) {
      rssi_detection = 0;
      rssi_done(status);
    }
}


/*
 * check if currently in the dozer_receive function
 */
bool in_rec_fct() {
  return receive_fct;
}


/*
 * rx timeout watchdog fired, that means the radio rx timeout failed
 */
void rx_to_fail() {
  to_fails++;
  // print number of timeout fails and successful radio timeouts
  sprintf(dozer_print_buffer, "rx timeout failed. fails: %d; count: %d", to_fails, to_count);
  dozer_print(10, dozer_print_buffer);
  dozer_timeout_callback(false);
}

/*
 * get timout fails
 */
uint16_t get_to_fails() {
  return to_fails;
}

/*
 * get number of timeouts
 */
uint16_t get_to_count() {
  return to_count;
}


void dozer_radio_shutdown() {
  radio_sleep(true);
}


#endif // DOZER_ENABLE
