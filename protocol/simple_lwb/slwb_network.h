/*
 * Copyright (c) 2018 - 2021, ETH Zurich, Computer Engineering Group (TEC)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PROTOCOL_SIMPLE_LWB_SLWB_NETWORK_H_
#define PROTOCOL_SIMPLE_LWB_SLWB_NETWORK_H_


typedef enum {
  CLUSTER_NODE,
  LONG_RANGE_NODE
} slwb_node_type_t;

typedef struct {
  uint8_t n_nodes;
  uint8_t nodes[];
} slwb_node_list_t;

typedef struct {
  slwb_node_list_t* cluster_nodes;
  slwb_node_list_t* lora_nodes;
} slwb_cluster_t;

typedef struct {
  uint8_t n_clusters;
  slwb_cluster_t clusters[];
}slwb_network_t;


void slwb_network_init();
void slwb_network_add_node(slwb_cluster_t* cluster, uint8_t node_id, slwb_node_type_t type);
void slwb_network_print(uint8_t prio, slwb_network_t* network);
slwb_network_t* slwb_get_network();

#endif /* PROTOCOL_SIMPLE_LWB_SLWB_NETWORK_H_ */
