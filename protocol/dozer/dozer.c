/*
 * dozer.c
 *
 *  Created on: Jun 22, 2018
 *      Author: kelmicha
 */

#include "dozer.h"
#include "dozer_topology.h"



extern node_config_t node_config;
extern current_state_t current_state;
extern bool send_activation;


void radio_dep_init() {
//  Set callback functions
  radio_set_rx_callback(&dozer_rx_callback);
  radio_set_tx_callback(&dozer_tx_callback);
  radio_set_timeout_callback(&dozer_timeout_callback);

//  Set address filtering
  uint8_t buffer[2] = {node_config.id, 0xFF};
  SX126xWriteRegisters(0x06CD, buffer, 2);
//  TODO: enable address filtering / send to address on radio side

}


void dozer_run(bool start) {
  radio_dep_init();
  dozer_init();

  if(start) {
    dozer_start();
  }
}

