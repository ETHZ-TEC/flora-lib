/*
 * radio_platform.h
 *
 *  Created on: 02.08.2018
 *      Author: marku
 */

#ifndef RADIO_RADIO_PLATFORM_H_
#define RADIO_RADIO_PLATFORM_H_

// Platform specific commands

void radio_set_nss_pin();
bool radio_read_busy_pin();
bool radio_read_dio1_pin();


#ifndef RADIO_TX_START_IND
  #define RADIO_TX_START_IND()
  #define RADIO_TX_STOP_IND()
#endif /* RADIO_TX_START_IND */
#ifndef RADIO_RX_START_IND
  #define RADIO_RX_START_IND()
  #define RADIO_RX_STOP_IND()
#endif /* RADIO_RX_START_IND */

#endif /* RADIO_RADIO_PLATFORM_H_ */
