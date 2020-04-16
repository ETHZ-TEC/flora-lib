/*
 * slwb_network.h
 *
 *  Created on: Feb 21, 2019
 *      Author: kelmicha
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
