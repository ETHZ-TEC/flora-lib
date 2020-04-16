/*
 * dozer_topology.c
 *
 *  Created on: Jun 28, 2018
 *      Author: kelmicha
 */

#include "flora_lib.h"

#if DOZER_ENABLE

/********************************* Defines *********************************/
#define INVALID 0xFFFFFFFF
#define EMPTY 0xFFFF


/********************************* Variables *********************************/

// Watchdog
uint8_t beacon_timer_cnt;
uint8_t beacon_send_failed;
uint8_t receive_bm_cnt;
uint8_t pot_dead_rec_bm_timer;

// support variables
parent_t parent_buffer; // buffer for overheard beaconMessages

// variables used for connecting to a parent
uint8_t c_parent_idx;
uint8_t missed_intervals;
uint8_t connecting; // flag indicating that node is trying to connect
uint8_t listening_for_beacons; // flag indicating if node is listening for beacons

// message buffers
dozer_send_message_t dsmsg;
dozer_send_message_t* connection_msg = &dsmsg;
static dozer_message_t dmsg;
static dozer_message_t* message = &dmsg;

// beacon related variables
uint64_t last_beacon_time;  // stores the time when the last beacon was received to compute the download slots
uint64_t next_beacon_time;  // stores the time the next beacon message is expected to be sent
uint32_t beacon_seed; // random value used to determine next beacon send time.

// parent related stuff
uint8_t connected; // 1 = connected 0 = not connected
parent_t* c_parent; // pointer to the current parent
parent_t parents[MAX_PARENTS]; // list with all known potential parents
uint8_t num_reconnect;
uint8_t num_connection_attempts;
uint8_t new_potential_parent_available; // flag indicating if a new potential parent was found.
uint8_t congestion_flag; // set if a hm is missed to check in the next beacon if the parent has accepted new children


// children stuff
uint8_t child_count; // number of connected children
child_t slot[MAX_CHILDREN]; // array of connected children

// helper variables to manage the download from the children
uint8_t next_child_slot;
uint8_t start_slot;
uint8_t empty_index;


// bootstrap
bootstrap_state_t bootstrap_state;
uint32_t overhear_time;
uint16_t overhear_slot;

// potential parent update
uint8_t current_parents_index_to_update;
uint8_t update_cnt;
uint8_t missed_intervals_update;
uint8_t parents_upate_beacon_expected;


// drift compensation
uint64_t current_time_at_beginning_of_precise_next_beacon_time; // value needs to be global to be able to compensate the computation time of the next beacon

uint64_t bmtxdelay, bmrxdelay, bmrxtime;

static uint8_t lock = 0; // lock for sorting algorithm







/****************************** Initialization and start  ***********************/

void dozer_init(){
  uint8_t i;

  // Get node ID
  node_config.id = *STM32_UID >> 16;
  node_config.role = node_config.id == SINK_ID;

  sprintf(char_buff, "Node ID: %d, Sink: %d", node_config.id, node_config.role);
  dozer_print(10, char_buff);

  // watchdog counters
  beacon_timer_cnt = 0;
  receive_bm_cnt = 0;
  pot_dead_rec_bm_timer = 0;

  //initialize the parents
  for(i=0; i<MAX_PARENTS; i++){
    parents[i].id = EMPTY;
    parents[i].rating = INVALID;
  }

  // initialize the reception slots
  for(i=0; i<MAX_CHILDREN;i++){
    slot[i].id = EMPTY;
  }

  connected = 0;
  connecting = 0; // flag indicating if the node currently tries to connect to a parent
  child_count = 0;
  next_child_slot = 0;
  num_reconnect = 0;
  num_connection_attempts = 0;

  new_potential_parent_available= 0;

  // bootstrap variables
  overhear_time = INTERVAL_TIME_IN_MS; // initially listen for a full slot
  bootstrap_state = BOOTSTRAP_STATE_WAITING;

  dozer_alloc_msg();
  cq_init();

  // set the radio configurations
  radio_config.band_index = BAND_INDEX;
  radio_config.modulation_index = MODULATION_INDEX;
  radio_config.power = TX_POWER;
}

void dozer_start(){

  uint32_t next_beacon_rel;

  srand(hs_timer_get_current_timestamp());
  beacon_seed = rand();

  if (!node_config.role) {
    // node starts protocol with 3 to 5 seconds delay
    srand(hs_timer_get_current_timestamp());
    uint32_t random_num = rand();
    next_beacon_rel = random_num % 2*HS_TIMER_FREQUENCY + 3*HS_TIMER_FREQUENCY;
    next_beacon_time = hs_timer_get_current_timestamp() + next_beacon_rel;
  }
  else {
    // sink starts after 5 seconds
    next_beacon_time = hs_timer_get_current_timestamp() + 5*HS_TIMER_FREQUENCY;
  }

  // start the timer for beacon sending.
  tq_start_timer(BEACON_SEND_TIMER, next_beacon_time, &beacon_timer_fired);

  // start the watchdog with an interval of > 2*interval time
  tq_start_timer(WATCHDOG_TIMER, hs_timer_get_current_timestamp() + 2*(INTERVAL_TIME_IN_JIFFIES+MAX_JITTER_IN_JIFFIES), &watchdog_timer_fired);

}



void beacon_timer_fired() {
  dozer_print(0, "beacon timer fired");
  print_ts(0);

    beacon_msg_t* beacon_msg;
    uint8_t num_pot_parents;
    uint64_t lsd;  // local sender delay
    uint32_t jitter;
    uint64_t current_time;
    dozer_header_t* header;

    jitter = beacon_seed % (uint32_t)MAX_JITTER_IN_JIFFIES;
    beacon_timer_cnt++;  // increase the counter to show the watchdog that everything works fine
    current_time = hs_timer_get_current_timestamp();
  lsd = current_time - next_beacon_time;

    // set the next timer after the interval time + some random value
    next_beacon_time += (uint32_t)INTERVAL_TIME_IN_JIFFIES + jitter;
    tq_start_timer(BEACON_SEND_TIMER, next_beacon_time, &beacon_timer_fired);

    bmtxdelay = lsd;

    // set the header infos
    header = &(message->header);
    header->source = node_config.id;
    header->dest = BROADCAST_ADDR;
    header->type = BEACON_MSG;

    beacon_msg = &(message->payload.beacon_msg);    // get a reference to the data field of the message
//    beacon_msg->temperature = msptemp;


    /************************* SINK START ***********************************/
  if (node_config.role){

    //set the beacon data
    beacon_msg->lsd = (uint16_t)lsd;
    beacon_msg->seed = beacon_seed;
    beacon_msg->hop_count = 0;
    beacon_msg->load = child_count;

    // send the beacon
    if (send_beacon((dozer_send_message_t*) message) == ERROR){
      dozer_print(0, "beacon not sent");
      set_download_timer(0); // set the download timer to trigger the data generation
      beacon_send_failed++; // increase so the watchdog can reset the radio state if to many fails occurred
    }
    else {
      beacon_send_failed = 0;
    }

    // prepare the next jitter value
    beacon_seed = next_rand(beacon_seed);
    return;
    }
    /************************* SINK END ************************************/


    if (bootstrap_state == BOOTSTRAP_STATE_WAITING) { // no activity -> stay in bootstrap mode
    if ((++overhear_slot * MIN_OVERHEAR_TIME) / 2 > INTERVAL_TIME_IN_MS) { // vary the overhear slots
      overhear_slot=0;
    }
    if (overhear_time > (OVERHEAR_TIMEOUT_DECREASE + MIN_OVERHEAR_TIME)) {
      overhear_time -= OVERHEAR_TIMEOUT_DECREASE;
    }
    else { // prevent the listening time from becoming too short
      overhear_time = MIN_OVERHEAR_TIME;
    }
    // start overhearing after some time
    set_overhear_params(0, overhear_time);
    uint64_t overhear_start_ts = hs_timer_get_current_timestamp() + (MIN_OVERHEAR_TIME * overhear_slot * HS_TIMER_FREQUENCY_MS) / 2;
    tq_start_timer(OVERHEAR_TIMER, overhear_start_ts, &overhear_timer_fired);

    // prepare the next jitter value
    beacon_seed = next_rand(beacon_seed);
    set_download_timer(0); // set the download timer to trigger the data generation
    return;
  }
    else if (bootstrap_state == BOOTSTRAP_STATE_ACTIVITY_DETECTED){ // a message was intercepted -> try to get some beacons
      dozer_print(6, "bss off");
    bootstrap_state = BOOTSTRAP_STATE_OFF; // stop the bootstrap mode
    overhear_time = INTERVAL_TIME_IN_MS; // reset the bootstrap listening times

    listening_for_beacons = 1;    // flag indicating that we are in an overhearing phase looking for beacons

    // start listening for beacons
    set_overhear_params(BEACON_MSG, INTERVAL_TIME_IN_MS);
    start_overhearing(0);

    // prepare the next jitter value
    beacon_seed = next_rand(beacon_seed);
    set_download_timer(0); // set the download timer to trigger the data generation
    return;
    }



    if (connected == 0) { // not connected -> check for a suitable cached parent
    num_pot_parents = count_potential_parents();

    if ((connecting == 0) && num_pot_parents > 0){ // if there is at least one potential parent and the node is not trying to connect as of now
      c_parent_idx = 0; // reset the pointer for the next parent to check to the one at the beginning of the array
      connecting = 1;
      connect_to_parent();
    }
    else if (num_pot_parents == 0){ // no more parents available
      dozer_print(6, "bss waiting");
      bootstrap_state = BOOTSTRAP_STATE_WAITING;
    }

    // prepare the next jitter value
    beacon_seed = next_rand(beacon_seed);
    set_download_timer(0); // set the download timer to trigger the data generation
    return;
    }

    // set the data for the beacon message
    beacon_msg->lsd = (uint16_t)lsd;
    beacon_msg->seed = beacon_seed;
    beacon_msg->hop_count = c_parent->distance_to_sink+1;
    beacon_msg->load = child_count;

    // initiate the transmission of the beacon
    if (send_beacon((dozer_send_message_t*) message) == ERROR) {
      set_download_timer(0);
    beacon_send_failed++; // increase so the watchdog can reset the radio state if to many fails occurred
    }
  else {
    beacon_send_failed = 0;
  }

    // prepare the next jitter value
    beacon_seed = next_rand(beacon_seed);
}


/*
 * overhear timer fired -> start listening for activity
 */
void overhear_timer_fired() {
   if (bootstrap_state == BOOTSTRAP_STATE_WAITING) {
     start_overhearing(0);
   }
}


/**
 * A Beacon was overheared -> Potentially a new parent was found
 */
void received_overhear_bm(dozer_message_t* message) {
  if (bootstrap_state != BOOTSTRAP_STATE_OFF){     // in the  bootstrap phase, activity was detected
    dozer_print(6, "bss activity detected");
    bootstrap_state = BOOTSTRAP_STATE_ACTIVITY_DETECTED;
  }
  else if (message->header.type == BEACON_MSG) { // listening for random beacons

    print_msg(message, 1);

    memset(&parent_buffer, 0, sizeof(parent_buffer));
    beacon_to_potential_parent(message, &parent_buffer); // get the beacon information and copy it into a potential parent buffer
    overhear_beacon_task();
  }
}


/**
 * A beacon we have waited for arrived
 */
void received_bm(dozer_message_t* msg) {
    int64_t driftcomp;
    uint64_t calccomp;
    parent_t* c_prt;
    dozer_header_t* header;


    // the beacon did not arrive in time -> timeout triggered
    if (msg == NULL) {
      dozer_print(0, "beacon timeout");
    if (parents_upate_beacon_expected  == 1){ // this is not a common beacon but we are updating our parents list
      parents_upate_beacon_expected = 0;
      c_prt = &parents[current_parents_index_to_update];
      c_prt->failed_connection_attempts++;
      rate_parent(c_prt);
      current_parents_index_to_update++; // update the next node if set_timer_for_next_parents_update is called
      set_timer_for_next_parents_update();
      return;
    }
//
    if (connecting == 1){ // currently trying to connect to a parent
      c_prt = &parents[c_parent_idx];
      if (congestion_flag == 1) { // we missed a handshake and expected it might be congestion but now also the beacon is missing
        // -> one penalty point for the missing handshake
        congestion_flag= 0;
        c_prt->failed_connection_attempts++;
        rate_parent(c_prt);
      }
      c_prt->failed_connection_attempts++;
      if (c_prt->failed_connection_attempts > MAX_CONNECTION_ATTEMPTS) {
        // too many retries -> ignore this neighbor for a while
        rate_parent(c_prt);  // let the rating function mark this node as invalid
      }
      connect_to_parent(); // try the next parent
    }
    else { // we are connected
      // missed the beacon from the parent
      if (++c_parent->failed_connection_attempts > MAX_TRANSMISSION_FAILURES) {
        // the parent does not seem to respond any more -> back to not connected
        rate_parent(c_parent); // update the parents rating
        connected = 0;
        sprintf(char_buff, "disconnected: %d", 2);
        dozer_print(6, char_buff);
      }
      else {
        // try again to receive the parent beacon after one interval
        // probably we miscalculated a guard time -> go back to max guard time for the next try
        c_parent->last_estimation_error = MAX_DRIFT_IN_PPM*(INTERVAL_TIME_IN_JIFFIES/1000000);
        set_timer_for_beacon(c_parent);
      }
    }
    }

    // the expected beacon arrived
    else {
      header = &(msg->header);
    if (parents_upate_beacon_expected == 1){ // this is not a common beacon but we are updating our parentsList
      dozer_print(0, "pudbe");
      c_prt = &parents[current_parents_index_to_update];
      parents_upate_beacon_expected = 0;
      beacon_to_potential_parent(msg, c_prt); // copy all data from the beacon to the parent
      c_prt->failed_connection_attempts = 0;      // this potential parent is as good as new
      rate_parent(c_prt);  // get a new rating for the parent
      current_parents_index_to_update++; // update the next node if set_timer_for_next_parents_update is called
      set_timer_for_next_parents_update();
      return;
    }

    if (connecting == 1) { // currently trying to connect to a parent
      c_prt = &parents[c_parent_idx];
      beacon_to_potential_parent(msg, c_prt);

      if (congestion_flag == 1){   // we missed a handshake msg of this node
        congestion_flag = 0;
        c_prt->failed_connection_attempts += 1; // 1 penalty point
        rate_parent(c_prt);
      }

      if (c_prt->last_load == MAX_CHILDREN){     // potential parent does not have a free slot -> connect to someone else
        rate_parent(c_prt); // let the rating function disable this information for a while
        // inform the radio admin so that it can update its state properly
        send_connection_request(NULL);
        connect_to_parent();
        return;
      }
      // generate a connection request message
      // set the header infos
      header = &(connection_msg->header);
      header->type = CONNECTION_REQ_MSG;
      header->dest = c_prt->id;
      header->source = node_config.id;

      // wait for some short time before sending the connection request message
      srand(hs_timer_get_current_timestamp());
      uint64_t tmp_delay = ((uint32_t) rand()) % CRM_BACKOFF_IN_JIFFIES;
      tq_start_timer(SEND_CRM_WAIT_TIMER, hs_timer_get_current_timestamp() + tmp_delay, &send_crm_wait_timer_fired);
    }
    else {
      // The parent beacon arrived
      if (header->source == c_parent->id)  {
        // update the drift estimation of the parent
        update_drift_estimation(c_parent, msg);

        // update the parent with the info from the beacon
        beacon_to_potential_parent(msg, c_parent);

        // prevent count to infinity
        if (c_parent->distance_to_sink >= MAX_HOPS){
          connected = 0;
          sprintf(char_buff, "disconnected: %d", 3);
          dozer_print(6, char_buff);

          c_parent->failed_connection_attempts = MAX_TRANSMISSION_FAILURES+1;
          rate_parent(c_parent);
          return;
        }

        // set the timer for the next data upload to the parent
        // parent.slot+1 since slot starts counting at 0 and we first want to have the connection slot
        driftcomp = (int64_t)(c_parent->slot + 1) * (int64_t)SLOT_TIME_IN_JIFFIES * c_parent->drift_ppm / 1000000;
        calccomp = hs_timer_get_current_timestamp() - c_parent->last_rendezvous_time;

        uint64_t ts = hs_timer_get_current_timestamp() + (c_parent->slot+1)*(uint32_t)SLOT_TIME_IN_JIFFIES + driftcomp - calccomp;
        tq_start_timer(UPLOAD_TIMER, ts, &upload_timer_fired);

      }

      // prepare to listen for next parent beacon
      set_timer_for_beacon(c_parent);

    }
    }
    return;
}



/**
 * takes a message_t containing a beacon message and copies the information to a potential parent buffer
 */
void beacon_to_potential_parent(dozer_message_t* msg, parent_t* prt) {

  beacon_msg_t* bm = &(msg->payload.beacon_msg);
  dozer_header_t* header = &(msg->header);

  // get a copy of the relevant data
  // compensate the delay of the beacon sender
  prt->last_rendezvous_time = msg->metadata.msg_rec_ts - bm->lsd;
  prt->id  = header->source;
  prt->distance_to_sink  = bm->hop_count;
  prt->last_seed  = bm->seed;
  prt->last_load  = bm->load;
  prt->rssi = msg->metadata.rssi;

}


/**
 * stores the new information in the array of potential parents
 */
void overhear_beacon_task() {

  uint8_t i, cur_index, max_rating_index;
  uint32_t max_rating;

  // rate the node in the buffer
  rate_parent(&parent_buffer);

  cur_index = MAX_PARENTS;
  max_rating = 0; // used to determine the worst parent (small values indicate a good parent)
  max_rating_index = 0xFF; // not necessary but the compiler doesn't like the uninitialized variable

  // go through the array of potential parents and clear out all outdated ones
  i = MAX_PARENTS;
  do {
    i--;
    if (parents[i].id == EMPTY){
    // the field is empty and may be used for storing the beacon
    cur_index = i;
    }
    else if (parents[i].id == parent_buffer.id) { //  if this field contains old information for the same node replace it
      memmove(&(parents[i]), &parent_buffer, sizeof(parent_t));
      parent_buffer.id = EMPTY;  // mark the parent_buffer as free
      parent_buffer.rating = INVALID;
      break;
    }
    else if ((parents[i].distance_to_sink > MAX_HOPS)) {
      // if there seems to be a count to infinity -> remove it
      parents[i].id = EMPTY;
      parents[i].rating = INVALID;
      cur_index = i;
    }
    else if ((parents[i].rating > max_rating) && (&(parents[i]) != c_parent)) {
      // if this is the worst stored parent  as of now and we are not connected to it remember its  rating and index in the array
      max_rating = parents[i].rating;
      max_rating_index = i;
    }
  } while (i > 0);

  // if this is a new node store it in the first free field if available
  if (parent_buffer.id != EMPTY) {
    if (cur_index < MAX_PARENTS) {
      //there is at least one free field
      memmove(&(parents[cur_index]), &parent_buffer, sizeof(parent_t));
    }
    else if (parent_buffer.rating < max_rating) {
      // the new parent is better than the currently known worst one
      memmove(&(parents[max_rating_index]), &parent_buffer, sizeof(parent_t));
    }
    if (listening_for_beacons == 1) {  // we are currently in a longer overhearing phase -> start listening again
      set_overhear_params(BEACON_MSG, 0);
      start_overhearing(1);
    }
  }
  else {
    if (listening_for_beacons == 1) { // we are currently in a longer overhearing phase -> start listening again
      // this parent was known from before -> keep listening
      set_overhear_params(BEACON_MSG, 0);
      start_overhearing(1);
    }
  }
  new_potential_parent_available = 1;
}

/*
 * rate the node according to hop count, #children and ID
 * a small value is good
 */
void rate_parent(parent_t* prt) {
  uint32_t prt_rating;

  if (prt->id == EMPTY){  // no node information stored at this address
    // update struct
    prt->rating = INVALID;
    return;
  }

  prt_rating =
    prt->distance_to_sink * MAX_CHILDREN * MAX_CHILDREN + 1 +
       prt->last_load * MAX_CHILDREN +
       prt->id % MAX_CHILDREN;

// ***
// TODO: enable if radio returns rssi values other than 0
//  // if the rssi value is to low increase rating
//  if (prt->rssi < RSSI_COMM_THRESHOLD) {
//    prt_rating = prt_rating << 2;
//  }
// ***

  if (prt != c_parent) { // not the current parent
    if (prt->failed_connection_attempts > MAX_CONNECTION_ATTEMPTS+1) { // the node was disabled and also a parents update failed -> put this node at the end of the list
      prt_rating = INVALID;
    }
    else if ((prt->last_load == MAX_CHILDREN) || (prt->failed_connection_attempts > MAX_CONNECTION_ATTEMPTS)) { // if the node is full or several connections failed ignore it for a while
      prt_rating |= ((uint32_t) 0x80000000); // set the msb
    }
  }
  else if (prt->failed_connection_attempts > MAX_TRANSMISSION_FAILURES) { // this is the parent but the connection does not work any more
    prt_rating |= ((uint32_t) 0x80000000);  // set the msb
  }

  // update struct
  prt->rating = prt_rating;
}




/**
 * counts the number of potential parents in the parents array
 */
uint8_t count_potential_parents() {
  uint8_t i;
  uint8_t cnt = 0;

  for(i=0; i<MAX_PARENTS; i++){
    if ((parents[i].rating >> 31) != 1){ // if the msb is one do not regard this node as a potential parent
      cnt++;
    }
  }
  return cnt;
}


/**
 * this task is repeatedly called as long as the node is not connected to a
 * parent but does have some potential parents stored
 */
void connect_to_parent() {
  dozer_print(0, "connect to parent");

  if ((connected == 0) && (count_potential_parents() > 0)) {
    if (new_potential_parent_available == 1){
      sort_potential_parents();
      c_parent_idx = 0;
      new_potential_parent_available = 0;
    }
    // find the next non-free slot
    while((parents[c_parent_idx].id ==  EMPTY) || ((parents[c_parent_idx].rating & 0x80000000) > 0)) {
      c_parent_idx = (c_parent_idx + (uint8_t)1) % (uint8_t)MAX_PARENTS;
    }

    parents[c_parent_idx].drift_ppm = 0;
    parents[c_parent_idx].last_estimation_error = MAX_DRIFT_IN_PPM * (INTERVAL_TIME_IN_JIFFIES / 1000000);

    set_timer_for_beacon(&(parents[c_parent_idx]));
  }
  else {
    // there are no more potential parents available and we are still not connected
    // give up for now and wait for new beacons
    // OR we are connected and do not need any further connection attempts
    connecting = 0;
  }
}


/**
 * Simple in place sorting algorithm (Bubble Sort)
 */
void sort_potential_parents() {
  uint8_t flag = 1;
  parent_t tmp_prt;

  while(flag == 1){
    flag = 0;
    for (uint8_t i=1; i<MAX_PARENTS; i++){
      // a low value for rating is better -> move these to the front
      if (parents[i].rating < parents[i-1].rating){
        // another round will be necessary
        flag = 1;

        if (!lock || lock >= 2) {
          lock = 1;

          // if we move the current parent also update the "parent" pointer
          if (&(parents[i]) == c_parent){
            c_parent = &(parents[i-1]);
          } else if (&(parents[i-1]) == c_parent){
            c_parent = &(parents[i]);
          }

          // swap two array entries
          memmove(&tmp_prt, &(parents[i]), sizeof(parent_t));
          memmove(&(parents[i]), &(parents[i-1]), sizeof(parent_t));
          memmove(&(parents[i-1]), &tmp_prt, sizeof(parent_t));

          lock = 0;
        }
        else {
          lock++;
        }
      }
    }
  }
}



/**
 * sets a timer for the next expected beacon time
 * includes guard times according to drift prediction and uncertainty interval
 */
void set_timer_for_beacon(parent_t* prt) {
  dozer_print(0, "set timer for beacon");

    uint64_t precise_time;
    uint64_t now;
    uint64_t time_since_last_beacon;
    uint64_t computation_time;

    precise_time = precise_next_beacon_time(prt->last_rendezvous_time,
           prt->last_seed,
           &missed_intervals,
           prt->last_estimation_error,
           prt->drift_ppm);


  time_since_last_beacon = current_time_at_beginning_of_precise_next_beacon_time - prt->last_rendezvous_time; // equal to _delta in "preciseNextBeaconTime(..)"

    // compute the relative timespan
    prt->time_till_next_beacon = precise_time + time_since_last_beacon;

    // compute the relative time for the next beacon including drift compensation
    precise_time = precise_time - WAKEUP_TIME_IN_JIFFIES - missed_intervals*prt->last_estimation_error;

    // measure the computing overhead for the drift compensation
    now = hs_timer_get_current_timestamp();
    computation_time = now - current_time_at_beginning_of_precise_next_beacon_time;
    if (computation_time < precise_time) {
      // start timer
    bmrxtime = now + precise_time - computation_time;
      tq_start_timer(RECEIVE_BEACON_TIMER, bmrxtime, &receive_beacon_timer_fired);
    }
    else {
      // calculate the beacon time again
      set_timer_for_beacon(prt);
    }

}



/**
 * Calculates the precise next RELATIVE time when the beacon is to be expected. It does not incorporate any wakeup times
 * or guards to make sure that the receiver is ready to receive but is used to calculate the estimation error
 * after the next beacon reception
 */
uint64_t precise_next_beacon_time(uint64_t last_rendezvous_time,
                  uint32_t last_seed,
                  uint8_t* missed_intervals,
                  uint32_t last_estimation_error,
                  int16_t drift_ppm) {
  uint16_t abs_drift_ppm;
  uint64_t delta, delta2;

  current_time_at_beginning_of_precise_next_beacon_time = hs_timer_get_current_timestamp();

  delta = current_time_at_beginning_of_precise_next_beacon_time - last_rendezvous_time;

  delta += WAKEUP_TIME_IN_JIFFIES; // delta Time in jiffies till now + necessary wakeup times (which will be removed afterwards)
  delta2 = 0;

  *missed_intervals = 0;    // amount of intervals between the last beacon reception and now

  abs_drift_ppm = abs16(drift_ppm);

  // get the next future beacon time of the potential parent
  // make sure the new time value is far enough in the future to compensate drift and estimation error
  while (delta2 <= delta+(*missed_intervals)*(abs_drift_ppm*(INTERVAL_TIME_IN_JIFFIES/1000000) + last_estimation_error)) {
    delta2 += (uint64_t)INTERVAL_TIME_IN_JIFFIES + (last_seed%(uint32_t)MAX_JITTER_IN_JIFFIES); // add the interval length plus the random delay
    last_seed = next_rand(last_seed);
    if (*missed_intervals < 0xFFFF){ // there should not be a case where we lose more than 255 beacons but just to be sure...
      (*missed_intervals) += 1;
    }
  }

  delta2 = delta2 - delta + WAKEUP_TIME_IN_JIFFIES + ((int32_t)*missed_intervals)*(drift_ppm*(int32_t)INTERVAL_TIME_IN_JIFFIES/(int32_t)1000000);

  return delta2;
}


/**
 * Updates the estimated clock drift to the parent based on the last beacon reception time
 */
void update_drift_estimation(parent_t* prt, dozer_message_t* msg) {
  uint64_t beacon_reception_time, estimated_reception_time;
  uint32_t missed_interv;
  int64_t current_est_error;
  beacon_msg_t* bm;

  int32_t error10e6, res;
  uint32_t passed_time;


  bm = (beacon_msg_t*) &(msg->payload.beacon_msg);

  beacon_reception_time = msg->metadata.msg_rec_ts - bm->lsd;

  // TODO this is only an approximation of the missed intervals since it does not incorporate the // tinyos comment
  // random delay. However, this should not be a problem in most cases.
  // as beacon_reception_time is always later than lastRendezvousTime (both uint32_t), we do not have to do
  // an overflow check à la "if (beacon_reception_time > prt->lastRendezvousTime)"
  missed_interv = (beacon_reception_time - prt->last_rendezvous_time) / INTERVAL_TIME_IN_JIFFIES;

  if (missed_interv == 0){
    // actually this should not happen. However if there is some serious drift it might be to wakeup twice
    // for the same message leading to a 0 here.
    missed_interv = 1;
  }

  // recalculate when the message should have come
  estimated_reception_time = prt->last_rendezvous_time + prt->time_till_next_beacon;

  // calculate the estimation error
  current_est_error = (int64_t) ((int64_t)beacon_reception_time - (int64_t)estimated_reception_time);
  sprintf(char_buff, "c est err: %llu", current_est_error);
  dozer_print(1, char_buff);

  // handle potential timer overflows between the reception time and the predicted reception time
  if (current_est_error > 0xFFFFFL) { // the beacon came too early and the overflow happened between beacon_reception_time and estimated_reception_time
    current_est_error = 0-(0xFFFFFFFF-beacon_reception_time + 1 + estimated_reception_time);
  }
  else if (current_est_error < -0xFFFFFL) { // the beacon came too late and the overflow happened between estimated_reception_time and beacon_reception_time
    current_est_error = 0xFFFFFFFF-estimated_reception_time + 1 + beacon_reception_time;
  } // TODO: overflow correction needed?

  // update the last estimation error. Do not go below MIN_GUARD_TIME_FOR_BEACONS_IN_JIFFIES
  prt->last_estimation_error = abs32(current_est_error);
  if (prt->last_estimation_error < MIN_GUARD_TIME_FOR_BEACONS_IN_JIFFIES) {
    prt->last_estimation_error = MIN_GUARD_TIME_FOR_BEACONS_IN_JIFFIES;
  }
  error10e6 = current_est_error*1000000;
  passed_time = INTERVAL_TIME_IN_JIFFIES * missed_interv;
  res = error10e6/(int32_t)passed_time;

  prt->drift_ppm += res;
}


/**
 * triggered when a beacon message of a potential parent is expected
 */
void receive_beacon_timer_fired() {
  dozer_print(0, "rec beacon timer fired");
    uint32_t on_time;

    receive_bm_cnt++; // increase the counter to show the watchdog that everything is fine

    bmrxdelay = hs_timer_get_current_timestamp() - bmrxtime;

    if (connected){
//    // calculate how long the receiver needs to be active.
    on_time = 2*missed_intervals*c_parent->last_estimation_error + WAKEUP_TIME_IN_JIFFIES + BEACON_TRANSMISSION_TIME_IN_JIFFIES;

    if ((bmrxdelay > MIN_GUARD_TIME_FOR_BEACONS_IN_JIFFIES / 2) || (listen_to_beacon(c_parent->id, on_time, 0) == ERROR)) {
      //expected to listen to beacon from parent but radio is busy -> try again at the next beacon time
      set_timer_for_beacon(c_parent);
    }
    }
    else if (connecting){
    //listen for the beacon and indicate an interest in sending a connection request

      uint32_t to = 2 * missed_intervals*parents[c_parent_idx].last_estimation_error + WAKEUP_TIME_IN_JIFFIES + BEACON_TRANSMISSION_TIME_IN_JIFFIES;
    if (listen_to_beacon(parents[c_parent_idx].id, to, 1) == ERROR) {
      // if the radio is busy try next neighbor
      // no penalty for this node, though
      connect_to_parent();
    }
    }
}


/**
 * beacon was sent
 */
void beacon_sent(uint64_t ts, uint8_t result) {
  dozer_print(3, "beacon sent");

    // if there is a child, wake-up to receive data.
    if (result == SUCCESS){
    last_beacon_time = ts;
    set_download_timer(1); // the beacon was sent -> children might upload data
  }
    else {
    set_download_timer(0); // no beacon was sent -> set timer for random data generation
    }
}

/*
 *  set timer to listen for a beacon to update a potential parent
 */
void set_timer_for_next_parents_update(){
    uint8_t i;
    uint8_t available_parents = 0;
    uint64_t precise_time;

    if (update_cnt != MAX_UPDATE_CNT){  // we are allowed to update at least one more parents entry

    // get the number of available parents (including disabled ones)
    for(i = 0; i<MAX_PARENTS; i++) {
      if (parents[i].id != EMPTY) {
        available_parents++;
      }
    }

    while(current_parents_index_to_update < MAX_PARENTS) {   // find the next valid entry
      if ((parents[current_parents_index_to_update].id != EMPTY) && (&(parents[current_parents_index_to_update]) != c_parent) && (parents[current_parents_index_to_update].rating != INVALID)) { // this is not the current parent
        if ((update_cnt < FORCED_UPDATE_CNT) || // not yet min amount of update parents found
          (available_parents <= MAX_UPDATE_CNT) || // or we can update all parents
          (((rand()%1000) * (available_parents - FORCED_UPDATE_CNT)) > (1000*(MAX_UPDATE_CNT-FORCED_UPDATE_CNT)))) {  // random selection of the remaining parents

          precise_time = precise_next_beacon_time(parents[current_parents_index_to_update].last_rendezvous_time,
                              parents[current_parents_index_to_update].last_seed,
                              &missed_intervals_update,
                              MAX_DRIFT_IN_PPM*(INTERVAL_TIME_IN_JIFFIES/1000000),
                              0);

          // for the wakeup times use worst case guard times
          precise_time = precise_time - WAKEUP_TIME_IN_JIFFIES - missed_intervals_update*MAX_DRIFT_IN_PPM*INTERVAL_TIME_IN_JIFFIES/1000000L;
          tq_start_timer(PARENTS_UPDATE_TIMER, precise_time, &parents_update_timer_fired);
          update_cnt++;
          return;
        }
      }
      current_parents_index_to_update++;
    }
    }
    // if we reach this line the parents update cycle is completed
    update_cnt=0;
    current_parents_index_to_update = MAX_PARENTS + 1;  // set the value to an illegal value. -> when the timer is triggered it knows it has to start a new update cycle
  tq_start_timer(PARENTS_UPDATE_TIMER, hs_timer_get_current_timestamp() + (uint64_t)PARENTS_UPDATE_INTERVAL_TIME_IN_MS*(uint64_t)HS_TIMER_FREQUENCY_MS, &parents_update_timer_fired);
}


/*
 * parents update timer fired -> listen for the beacon
 */
void parents_update_timer_fired() {
  if (current_parents_index_to_update > MAX_PARENTS) { // this is the beginning of a new update cycle
    current_parents_index_to_update = 0;
    set_timer_for_next_parents_update();
    return;
  }

  if (listen_to_beacon(parents[current_parents_index_to_update].id, (missed_intervals_update*GRACE_PERIOD_IN_JIFFIES*2 + WAKEUP_TIME_IN_JIFFIES + BEACON_TRANSMISSION_TIME_IN_JIFFIES), 0) != SUCCESS) {
    current_parents_index_to_update++; // update the next node if set_timer_for_next_parents_update is called
    set_timer_for_next_parents_update();
  }
  else {
    parents_upate_beacon_expected = 1;
  }
}


void send_crm_wait_timer_fired() {
    if (send_connection_request(connection_msg) == ERROR) { // if the radio is busy try next
      connect_to_parent();
    }
}



/**
 * connection request received -> allocate and send slot number for new child
 */
void received_crm(dozer_message_t* msg) {

  dozer_header_t* header;
  dozer_header_t* crm_header;
//  uint8_t i;
  handshake_msg_t* hs_msg;
  crm_header =  &(msg->header);

  empty_index = MAX_CHILDREN;
  for (uint8_t i=MAX_CHILDREN; i>0; i--){
    if (slot[i-1].id == EMPTY) {
      empty_index = i-1;
    } // remember the empty slot index
    else if (slot[i-1].id == crm_header->source) {
      // the requesting node is already a child. -> It must have lost sync -> drop the old entry
      slot[i-1].id = EMPTY;
      empty_index = i-1;
      child_count--; // reduce the child count to prevent doubles
    }
  }

  // is there an empty slot or have we seen the child already?
  if (empty_index < MAX_CHILDREN) {
    // prepare the handshake msg
    header = &(connection_msg->header);
    header->type = HANDSHAKE_MSG;
    header->dest = crm_header->source;
    header->source = node_config.id;

    hs_msg = (handshake_msg_t*) &(connection_msg->payload.handshake_msg);
    hs_msg->slot = empty_index;

    sprintf(char_buff, "send slot: %d", hs_msg->slot);
    dozer_print(0, char_buff);

    if (send_handshake(connection_msg) == SUCCESS) {
      // reserve the slot for the child if the message was sent
      slot[empty_index].id = crm_header->source;
      slot[empty_index].last_rendezvous_time = hs_timer_get_current_timestamp();
      slot[empty_index].packets_received = false;
      child_count++; // increase the child count
    }
  }
  else {
    // inform the radio admin  that it can update its state properly
    send_handshake(NULL);
  }
  return;
}


/*
 * checks if the handshake is valid
 * if it is, save slot and set timer for next beacon
 * else try again to connect to a parent
 */
void received_handshake(dozer_message_t* msg) {

    handshake_msg_t* hs_msg;
    dozer_header_t* header;

    if (msg == NULL){
    congestion_flag = 1; // sent crm but did not get hm -> possibly congestion related -> check next beacon
    num_connection_attempts++;
    connect_to_parent();
    return;
    }

    header = &(msg->header);
    hs_msg = (handshake_msg_t*) &(msg->payload.handshake_msg);

    // if a wrong handshake message arrives
    if (header->source != parents[c_parent_idx].id) {
    connect_to_parent();
    return;
    }

    parents[c_parent_idx].slot = hs_msg->slot;
    parents[c_parent_idx].failed_connection_attempts = 0;

    c_parent = &(parents[c_parent_idx]);

    // initialize the drift compensation values
    c_parent->drift_ppm = 0;
    c_parent->last_estimation_error = MAX_DRIFT_IN_PPM*(INTERVAL_TIME_IN_JIFFIES/1000000);  //assume maximal error since we have no idea about the drift yet

    // calculate the time till the next beacon of the parent
    set_timer_for_beacon(c_parent);

    connected = 1; // finally found a parent and got an upload slot
    sprintf(char_buff, "connected: prt: %d; slot: %d", header->source, hs_msg->slot);
  dozer_print(6, char_buff);

    connecting = 0; // no longer trying to connect to a potential parent
    num_reconnect++;
    num_connection_attempts++;

    return;
}

/*
 * function called after a successful or failed handshake send
 */
void handshake_sent(uint8_t result) {
    // if the transmission of the handshake message failed free the allocated child slot
    if (result == ERROR){
    slot[empty_index].id = EMPTY;
    child_count--;
    }
    else {
      dozer_print(0, "hs sent");
    }
}


/*
 * set the timer to download data from the children or to generate data if no more children available
 */
void set_download_timer(uint8_t active) {
  dozer_print(3, "set download timer");
  uint64_t current_time;
  uint64_t delta, bm_receive_time;
  start_slot = next_child_slot;

  if (active == 1) { // if we have sent a beacon and want to listen to the children

    while (next_child_slot < MAX_CHILDREN) {
      // if there is a registered child in this slot, then wake-up
      if (slot[next_child_slot].id != EMPTY) {
        current_time = hs_timer_get_current_timestamp();
        delta = current_time - last_beacon_time;
        delta = (next_child_slot+1)*(uint32_t)SLOT_TIME_IN_JIFFIES-delta;

        if (tq_is_running(RECEIVE_BEACON_TIMER)) {
          bm_receive_time = tq_get_fire_ts(RECEIVE_BEACON_TIMER) - current_time;

          // check if beacon receive and download collide
          // TODO: find a more reasonable threshold
          if (bm_receive_time < delta + (20 * DATA_TRANSMISSION_TIME_IN_JIFFIES) && bm_receive_time > delta) {
            next_child_slot++;
            continue;
          }
        }
        tq_start_timer(DOWNLOAD_TIMER, hs_timer_get_current_timestamp() + delta - DOWNLOAD_GUARD_TIME_IN_JIFFIES, &download_timer_fired);
        next_child_slot++;
        return;
      }
      next_child_slot++;
    }
    // no more children in this interval
  }
  next_child_slot = 0;


  current_time = hs_timer_get_current_timestamp();
  delta = ((uint64_t)MAX_CHILDREN - start_slot + 1) * (uint64_t)SLOT_TIME_IN_JIFFIES;

  if (tq_is_running(RECEIVE_BEACON_TIMER)) {
    bm_receive_time = tq_get_fire_ts(RECEIVE_BEACON_TIMER) - current_time;
    // receive beacon in first half of processing slots
    if (bm_receive_time > delta && bm_receive_time < delta + NUM_PROCESSING_SLOTS * (uint64_t)SLOT_TIME_IN_JIFFIES / 2) {
      delta = bm_receive_time + BEACON_TRANSMISSION_TIME_IN_JIFFIES;
    }
    // receive beacon in second half of processing slots
    else if (bm_receive_time > delta + NUM_PROCESSING_SLOTS * (uint64_t)SLOT_TIME_IN_JIFFIES / 2 && bm_receive_time < delta + NUM_PROCESSING_SLOTS * (uint32_t)SLOT_TIME_IN_JIFFIES) {
      uint64_t skew;
      if ((delta + NUM_PROCESSING_SLOTS * (uint64_t)SLOT_TIME_IN_JIFFIES) - bm_receive_time > SLOT_TIME_IN_JIFFIES) {
        skew = SLOT_TIME_IN_JIFFIES;
      }
      else {
        skew = (delta + NUM_PROCESSING_SLOTS * (uint64_t)SLOT_TIME_IN_JIFFIES) - bm_receive_time;
      }
      if (skew > delta) {
        skew = delta - 1;
      }
      delta = delta - skew;
    }
  }
  tq_start_timer(DOWNLOAD_TIMER, hs_timer_get_current_timestamp() + delta, &download_timer_fired);
}

/*
 * download timer fired -> wait for data from children or trigger the random data generation
 */
void download_timer_fired() {
  dozer_print(3, "download timer fired");

    uint8_t this_child_index;
    uint16_t* last_seq_nr = NULL;

    // special case at the end of the interval
    if (next_child_slot == 0) {
    rand_data_gen(); // generate data for tests
    return;
    }

    // get the index of this child
    this_child_index = next_child_slot-1;

    // reset the timer for the next child
    // if we reach this code the beacon must have been sent
    // so "active" can be set to 1
    set_download_timer(1);

    // receive the data messages from the child
    if (slot[this_child_index].packets_received) {
    last_seq_nr = &(slot[this_child_index].last_seq_nr);
    }

    receive_messages_to_buffer(0, slot[this_child_index].id, last_seq_nr);
}

/*
 * message download finished
 */
void receive_messages_to_buffer_done(uint8_t n_msgs, uint16_t l_seq_nr) {
  dozer_print(5, "rec msgs to buffer done");
  print_ts(5);

  uint64_t current_time = hs_timer_get_current_timestamp();
  uint64_t delta_time;

  if (n_msgs == 0) {
    if (current_time >= slot[start_slot-1].last_rendezvous_time) { // normal case
      delta_time = current_time - slot[start_slot-1].last_rendezvous_time;
    }
    else {  // timer overflow
      delta_time = (~slot[start_slot-1].last_rendezvous_time) + current_time + 1;
    }
    if (delta_time > (uint64_t) SENSOR_SAMPLING_INTERVAL_IN_JIFFIES * (uint64_t) MAX_SENSOR_MISSES){ // Too much time passed since the last update
      sprintf(char_buff, "dropped child: %d; delta: %llu", slot[start_slot-1].id, delta_time);
      dozer_print(6, char_buff);
      slot[start_slot-1].id = EMPTY; // drop this child
      child_count--;
    }
  }
  else {
    slot[start_slot-1].last_rendezvous_time = current_time;
    slot[start_slot-1].last_seq_nr = l_seq_nr;
    slot[start_slot-1].packets_received = true;
  }
}

/*
 * message upload finished
 */
void send_buffered_messages_done(uint8_t num, bool acked) {
  dozer_print(5, "send buf msgs done");
  print_ts(5);
    if (!acked) {
    // the queue was not empty but no messages could be sent -> 1 penalty point for this parent
    if (++c_parent->failed_connection_attempts > MAX_TRANSMISSION_FAILURES) {
      // the parent does not seem to respond any more -> back to not connected
      connected = 0;
      sprintf(char_buff, "disconnected: %d", 1);
      dozer_print(6, char_buff);

      rate_parent(c_parent);

      // stop the beacon timer
      tq_stop_timer(RECEIVE_BEACON_TIMER);
    }
    }
    else {
    // the transmission of at least one message succeeded -> reset the penalty points
    c_parent->failed_connection_attempts = 0;
    }
}

/*
 * upload timer fired -> send data messages
 */
void upload_timer_fired() {
  uint16_t max_msgs;
    if (connected == 1){
    // limit the number of messages to send to the actual slot time
    max_msgs = ((uint32_t)SLOT_TIME_IN_JIFFIES) / (uint32_t)DATA_TRANSMISSION_TIME_IN_JIFFIES;
    send_buffered_messages(max_msgs > 0xff ? 0xff : max_msgs, c_parent->id);
    }
}

/*
 * watchdog timer fired -> check if everything is ok
 */
void watchdog_timer_fired() {
    uint8_t i;
    bool crashed = false;
    if (beacon_timer_cnt == 0) {  // beacon timer died
      // reset the node state but keep the potential parents and restart the timer
      connected = 0;
    connecting = 0;
    child_count = 0;
    next_child_slot = 0;
    num_reconnect = 0;
    num_connection_attempts = 0;
    new_potential_parent_available = 0;
    crashed = true;
    // initialize the reception slots
    for(i=0; i<MAX_CHILDREN;i++){
      slot[i].id = EMPTY;
    }
    tq_start_timer(BEACON_SEND_TIMER, hs_timer_get_current_timestamp() + 100*HS_TIMER_FREQUENCY_MS, &beacon_timer_fired);// restart the timer with 100ms delay
    }
    else {
      beacon_timer_cnt = 0;
    }

    if (beacon_send_failed > 0) { // beacon send failed to many times
      // reset radio state
      beacon_send_failed = 0;
      radio_reset_state();
      crashed = true;
    }

    if ((connected == 1) && (receive_bm_cnt == 0)) { // the timer for receiving beacons from the parent has died

    if (pot_dead_rec_bm_timer == 1){
      pot_dead_rec_bm_timer = 0;
      connected = 0;
      crashed = true;
      tq_stop_timer(RECEIVE_BEACON_TIMER);
    }
    else {  // possibly we just connected to the parent and did not yet have a chance to receive a beacon -> just remember that there was no beacon reception
      pot_dead_rec_bm_timer = 1;
    }
    }
    else {
    receive_bm_cnt = 0;
    pot_dead_rec_bm_timer = 0;
    }

    if (crashed) {
      dozer_print(10, "watchdog fired. dozer crashed");
    }

    // restart watchdog timer
    dozer_print(5, "wd");
  tq_start_timer(WATCHDOG_TIMER, hs_timer_get_current_timestamp() + 2*((uint32_t)INTERVAL_TIME_IN_JIFFIES+MAX_JITTER_IN_JIFFIES), &watchdog_timer_fired);
}

/*
 * gets called if overhearing times out -> set listening_for_beacons to 0
 */
void reset_listening_for_beacons() {
  listening_for_beacons = 0;
}

/*
 * generate random data
 * the sink prints out all data messages in the queue and deletes them
 */
void rand_data_gen() {

  if (radio_is_idle()) {
    dozer_print(5, "rand data gen");

    if (node_config.role) {

      uint8_t p = 7;

      uint8_t u_buf = cq_cntUsedBuffer();
      data_msg_t* buf = malloc(sizeof(data_msg_t));
      uint8_t* dtype = NULL;
      uint8_t* plength = NULL;
      dozer_print(p, "**********");
      sprintf(char_buff, "ubuf: %d", u_buf);
      dozer_print(p, char_buff);
      for (int i = 0; i < u_buf; ++i) {
        if (cq_getFirst(buf, dtype, plength)) {
          // print message origin and sequence number
          sprintf(char_buff, "origin: %d, seqNr: %d", buf->originatorID, buf->seqNr);
          dozer_print(p, char_buff);
          // delete message to prevent a full queue
          cq_dropFirst();
        }
      }
      dozer_print(p, "**********");
      free(buf);
    }
    else {
      static uint16_t seqNr = 0;

      data_msg_t tdmsg;

      // generate data messages with random data
      for (int k = 0; k < 10; ++k) {

        tdmsg.aTime = hs_timer_get_current_timestamp();
        tdmsg.originatorID = node_config.id;
        tdmsg.seqNr = seqNr++;

        for (int i = 0; i < 23; ++i) {
          tdmsg.data[i] = seqNr+i;
        }

        if (cq_append(0, DATA_MSG, sizeof(tdmsg), &tdmsg)) {
  //        sprintf(char_buff, "seqNr: %d", seqNr-1);
  //        dozer_print(5, char_buff);
        }
      }

      sprintf(char_buff, "msgs generated until seqNr: %d", seqNr-1);
      dozer_print(6, char_buff);
    }

    // print radio statistics
    print_radio_stats();
  }
  else  {
    dozer_print(5, "rand data gen busy");
  }

}

#endif /* DOZER_ENABLE */
