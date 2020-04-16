/*
 * dozer_topology.h
 *
 *  Created on: Jun 28, 2018
 *      Author: kelmicha
 */

#ifndef PROTOCOL_DOZER_DOZER_TOPOLOGY_H_
#define PROTOCOL_DOZER_DOZER_TOPOLOGY_H_


/*
 * start the protocol
 */
void dozer_init();
void dozer_start();




/*
 * beacon estimation
 */
void update_drift_estimation(parent_t* prt, dozer_message_t* msg);
uint64_t precise_next_beacon_time(uint64_t last_rendezvous_time,
                  uint32_t last_seed,
                  uint8_t* missed_intervals,
                  uint32_t last_estimation_error,
                  int16_t drift_ppm);


/*
 * parent related functions
 */
void connect_to_parent();
void beacon_to_potential_parent(dozer_message_t* msg, parent_t* pat);
uint8_t count_potential_parents();
void rate_parent(parent_t* pat);
void sort_potential_parents();


/*
 * message sent
 */
void beacon_sent(uint64_t ts, uint8_t result);
void handshake_sent(uint8_t);


/*
 * process received messages
 */
void received_bm(dozer_message_t* msg);
void overhear_beacon_task();
void received_overhear_bm(dozer_message_t* message);
void received_crm(dozer_message_t* msg);
void received_handshake(dozer_message_t* msg);


/*
 * data messages send and receive
 */
void receive_messages_to_buffer_done(uint8_t n_msgs, uint16_t l_seq_nr);
void send_buffered_messages_done(uint8_t num, bool acked);


/*
 * set new timers
 */
void set_timer_for_beacon(parent_t *potParent);
void set_download_timer(uint8_t active);
void set_timer_for_next_parents_update();


/*
 * timers fired
 */
void beacon_timer_fired();
void overhear_timer_fired();
void overhear_timeout_timer_fired();
void receive_beacon_timer_fired();
void send_crm_wait_timer_fired();
void download_timer_fired();
void upload_timer_fired();
void parents_update_timer_fired();
void watchdog_timer_fired();


/*
 * other
 */
void reset_listening_for_beacons();
void rand_data_gen();




#endif /* PROTOCOL_DOZER_DOZER_TOPOLOGY_H_ */
