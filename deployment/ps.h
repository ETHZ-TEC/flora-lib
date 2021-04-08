/*
 * ps.h
 *
 * DPP message handling for the permasense deployment
 */

#ifndef DEPLOYMENT_PS_H_
#define DEPLOYMENT_PS_H_


#ifndef BASEBOARD
#define BASEBOARD         0
#endif /* BASEBOARD */

#ifndef PS_TIMESTAMP
#define PS_TIMESTAMP()    lptimer_now()       /* function that returns a 64-bit timestamp */
#endif /* PS_TIMESTAMP */

#ifndef PS_DEVICE_ID
#define PS_DEVICE_ID      NODE_ID
#endif /* PS_DEVICE_ID */


bool    ps_validate_msg(dpp_message_t* msg);
uint8_t ps_compose_msg(uint16_t recipient,
                       dpp_message_type_t type,
                       const uint8_t* data,
                       uint8_t len,
                       dpp_message_t* out_msg);
void    ps_update_msg_crc(dpp_message_t* msg);
void    ps_compose_nodeinfo(dpp_message_t* out_msg,
                            uint16_t rst_cnt,
                            uint32_t cfg_field);


#endif /* DEPLOYMENT_PS_H_ */
