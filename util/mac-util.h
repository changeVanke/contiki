/*
 * This is a util tool for MAC layer and RF layer.
 * Author is Vanke.
 * */

#ifndef MAC_UTIL_H
#define MAC_UTIL_H
#endif

#include "contiki.h"
#include "os/contiki-net.h"
#include "net/netstack.h"

/* Signed integer. */
typedef int sync_times;

struct time_RTT {
    uint32_t start_time;
    uint32_t end_time;
    uint32_t RTT;
};

/* For time synchronization transmission.
 * If the transmission is sent by the child node, times = -1, and the other situation is times >= 0
 * which is meaning that transmission is sent by the Parent node.
 *  */

struct all_time_pointb {
    sync_times times;
    uint32_t b0;
    uint32_t b1;
};

struct all_time_pointa {
    uint32_t a0;
    uint32_t a1;
    uint32_t a2;
};

/*
 * We set the channel from 0 to 9, and 0 is default channel for the network to maintain the topology and control information.
 * Then, 1 is just for the CCO node(root).
 * Finally, 2-9 channels are for the other nodes transmitting the data.
 *
 * */
void get_node_key_channel_by_rand(void);

/*
 * Get the number between 2 to 8 in a pseudo-random way.
 *
 * */
int get_rand_num(uint16_t source);

/*
 * parse the frame of 802.15.4
 * */
void catch_frame(void);

/**
 * Get the parent's address;
 * Return uip_ipaddr_t parent.
 **/

uip_ipaddr_t get_parent_node();

/**
 * Get the present time.
 *
 */
uint32_t get_present_time(void);

/*
 * Set the node as a root;
 * Return int is_root.
 * */




