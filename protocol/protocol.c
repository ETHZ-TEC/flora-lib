/*
 * protocol.c
 *
 *  Created on: 09.05.2018
 *      Author: marku
 */

#include "flora_lib.h"


protocol_config_t protocol_config = {0};


bool protocol_inititialized = false;


void protocol_init()
{
#if CONFIG_ENABLE
  protocol_config.uid = config_get()->uid;
  protocol_config.role = config_get()->role;
#else /* CONFIG_ENABLE */
  protocol_config.uid = 0;
  protocol_config.role = 0;
#endif /* CONFIG_ENABLE */

  protocol_inititialized = true;
  return;
}

void protocol_update()
{
  return;
}

void protocol_run()
{
  return;
}
