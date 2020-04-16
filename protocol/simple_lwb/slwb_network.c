/*
 * slwb_network.c
 *
 *  Created on: Feb 21, 2019
 *      Author: kelmicha
 */

#include "flora_lib.h"

#if FLOCKLAB
slwb_node_list_t cl_0 = {.n_nodes = 8, .nodes = {2, 3, 15, 18, 22, 23, 28, 33}};
//slwb_node_list_t lr_0 = {.n_nodes = 2, .nodes = {6, 16}};
slwb_node_list_t cl_1 = {.n_nodes = 5, .nodes = {13, 19, 20, 25, 26}};
//slwb_node_list_t lr_1 = {.n_nodes = 2, .nodes = {7, 11}};

slwb_node_list_t* cluster_nodes[] = {&cl_0, &cl_1};
//slwb_node_list_t* lora_nodes[] = {&lr_0, &lr_1};

uint8_t n_clusters = 2;
#else /* FLOCKLAB */
slwb_node_list_t cl_0 = {.n_nodes = 1, .nodes = {2}};
//slwb_node_list_t lr_0 = {.n_nodes = 1, .nodes = {3}};

slwb_node_list_t* cluster_nodes[] = {&cl_0};
//slwb_node_list_t* lora_nodes[] = {&lr_0};

uint8_t n_clusters = 1;
#endif /* FLOCKLAB */

slwb_network_t* slwb_network;

void slwb_network_init() {
  slwb_network = malloc(n_clusters * sizeof(slwb_cluster_t) + 1);
  slwb_network->n_clusters = n_clusters;

  for (int i = 0; i < n_clusters; ++i) {
    slwb_network->clusters[i].cluster_nodes = malloc(cluster_nodes[i]->n_nodes + 1);
    memcpy(slwb_network->clusters[i].cluster_nodes, cluster_nodes[i], cluster_nodes[i]->n_nodes + 1);

//    slwb_network->clusters[i].lora_nodes = malloc(lora_nodes[i]->n_nodes + 1);
//    memcpy(slwb_network->clusters[i].lora_nodes, lora_nodes[i], lora_nodes[i]->n_nodes + 1);
    slwb_network->clusters[i].lora_nodes = malloc(1);
    slwb_network->clusters[i].lora_nodes->n_nodes = 0;
  }
}

void slwb_network_add_node(slwb_cluster_t* cluster, uint8_t node_id, slwb_node_type_t type) {
  switch (type) {
    case CLUSTER_NODE:
      if(array_find((uint8_t*) cluster->cluster_nodes->nodes, cluster->cluster_nodes->n_nodes, node_id) == 0xFF) {
        cluster->cluster_nodes = realloc((void*) cluster->cluster_nodes, cluster->cluster_nodes->n_nodes + 2);
        cluster->cluster_nodes->nodes[cluster->cluster_nodes->n_nodes] = node_id;
        cluster->cluster_nodes->n_nodes += 1;
      }
      break;
    case LONG_RANGE_NODE:
      if(array_find((uint8_t*) cluster->lora_nodes->nodes, cluster->lora_nodes->n_nodes, node_id) == 0xFF) {
        cluster->lora_nodes = realloc((void*) cluster->lora_nodes, cluster->lora_nodes->n_nodes + 2);
        cluster->lora_nodes->nodes[cluster->lora_nodes->n_nodes] = node_id;
        cluster->lora_nodes->n_nodes += 1;
      }
      break;
    default:
      break;
  }
}

slwb_network_t* slwb_get_network() {
  return slwb_network;
}

void slwb_network_print(uint8_t prio, slwb_network_t* network) {
  slwb_cluster_t* cluster = NULL;

  for (int i = 0; i < network->n_clusters; ++i) {
       cluster = &(network->clusters[i]);
       sprintf(char_buff, "cluster %d:\n", i);
       print(prio, char_buff);

       sprintf(char_buff, "\tcluster nodes %d:\n", cluster->cluster_nodes->n_nodes);
       print(prio, char_buff);
       for (int j = 0; j < cluster->cluster_nodes->n_nodes; ++j) {
               sprintf(char_buff, "\t\t%d\n", cluster->cluster_nodes->nodes[j]);
               print(prio, char_buff);
       }

       sprintf(char_buff, "\tlora nodes %d:\n", cluster->lora_nodes->n_nodes);
       print(prio, char_buff);
       for (int k = 0; k < cluster->lora_nodes->n_nodes; ++k) {
               sprintf(char_buff, "\t\t%d\n", cluster->lora_nodes->nodes[k]);
               print(prio, char_buff);
       }
  }
}
