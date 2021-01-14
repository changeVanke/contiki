/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "os/contiki-net.h"
#include <stdio.h> /* For printf() */
#include <stdlib.h>
#include "dev/button-hal.h"
#include "dev/leds.h"

#include "sys/log.h"
#define LOG_MODULE "Hello"
#define LOG_LEVEL LOG_LEVEL_DBG

#include "util/mac-util.h"

/*
 * setting;
 * Macro definition.
 */
#define SEND_INTERVAL (20 * CLOCK_SECOND)
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678
#define OFFSET 0
#define ROUND 5

/* parent node ipv6 address. */
static uip_ipaddr_t parent_addr;
/* declare the simple udp connection for client and server. */
static struct simple_udp_connection client_conn, server_conn;
/* real timer -- Rtimer*/
static struct rtimer r_timer;
/* synchronization times */
static int stime;
static struct time_RTT rTT;
/* offset count algorithm. */
static struct all_time_pointb time_point_b;
static struct all_time_pointa time_point_a;
static long offset;
static uint32_t dtx0, dtx1;

//process_event_t button_hal_press_event;
/*---------------------------------------------------------------------------------------------*/
/**
 * handle the time synchronization. Callback function.
 */
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen) {
    /* static variable number for offset in function*/
    static long offset_in;
    /* remote time tick b0,b1. */
    struct all_time_pointb remote;
    uint32_t remote_time;
//    static uint32_t target = 99999999;
    uint32_t target_in;

    time_point_b.b0 = get_present_time();
    time_point_a.a1 = get_present_time() - offset;

    /* Core Handle. */
    if(datalen >= sizeof(remote) && uip_ipaddr_cmp(&parent_addr, sender_addr)) {
        /* if this is a time synchronization response from parent node.
         * Handle: set the own present time as the time of response from parent node.
         * */
        /* memory copy from remote UDP packets. */
        memcpy(&remote, data, sizeof(remote));

        /* RTT */
        stime = remote.times;
        rTT.end_time = time_point_a.a1;
        rTT.RTT = rTT.end_time - rTT.start_time;
        LOG_INFO("#%d RTT = %lu\n", stime, rTT.RTT);

        /*
         *  offset count.
         */
        /* dtx0 is m0~m1 or m2~m3 time. */
        dtx0 = (rTT.RTT - remote.b1 + remote.b0) / 2;
        LOG_INFO("b0 = %lu, b1 = %lu, dtx0 = %lu, a0 = %lu, a1 = %lu\n", remote.b0, remote.b1, dtx0, time_point_a.a0, time_point_a.a1);

        /* deal with offset */
//        if(stime != 1) {
//            offset_in = time_point_a.a0 - (remote.b0 - ((time_point_a.a1 - time_point_a.a0 - remote.b1 + remote.b0) / 2)) + offset_in + target;
//        } else {
//            offset_in = time_point_a.a0 - (remote.b0 - ((time_point_a.a1 - time_point_a.a0 - remote.b1 + remote.b0) / 2)) + offset_in;
//        }
        offset_in = time_point_a.a0 - (remote.b0 - ((time_point_a.a1 - time_point_a.a0 - remote.b1 + remote.b0) / 2)) + offset_in;
        time_point_a.a2 = get_present_time();
        dtx1 = time_point_a.a2 - time_point_b.b0;

        /* dtx1 is s3~s4 time. */
        target_in = (remote.b1 + dtx0 + dtx1) - (time_point_a.a2 - offset_in + dtx1);
        /* whether to keep the offset value. */
//        if(abs(target) > abs(target_in)) {
//            target = target_in;
//            LOG_INFO("dtx1 = %lu, a2 = %lu, b1+dtx0+dtx1 - (a2-offset+dtx1) = %ld\n", dtx1, time_point_a.a2, target);
//            LOG_INFO("#%d Set the present time by adding offset(%ld), new time = %lu\n", stime, offset_in, (unsigned long)(time_point_a.a2 - offset_in));
//            /* deliver to global variable offset */
//            if(stime == 1) {
//                offset = offset_in + OFFSET;
//            } else if(stime < ROUND){
//                offset = offset_in;
//            }
//        } else {
//            LOG_INFO("target_in = %ld, target = %ld\n", target_in, target);
//        }
        LOG_INFO("dtx1 = %lu, a2 = %lu, b1+dtx0+dtx1 - (a2-offset+dtx1) = %ld\n", dtx1, time_point_a.a2, target_in);
        LOG_INFO("#%d Set the present time by adding offset(%ld), new time = %lu\n", stime, offset_in, (unsigned long)(time_point_a.a2 - offset_in + target_in));
        if(stime == 1) {
            offset = offset_in - target_in + OFFSET;
        } else {
            offset = offset_in - target_in;
        }
    } else if(datalen >= sizeof(remote_time) && !uip_ipaddr_cmp(&parent_addr, sender_addr)){
        /* Else this is a time synchronization request from child node.
         * Handle: return a new present time to the child node which is the sender_addr.
         * */
        /* memory copy from remote UDP packets. */

        memcpy(&remote_time, data, sizeof(remote_time));

        LOG_INFO("Received from ");
        LOG_INFO_6ADDR(sender_addr);
        LOG_INFO_(" Request, child node time at %lu, now at %lu\n", remote_time, get_present_time());

        /* return response to sender(child node). */
        stime++;
        time_point_b.times = stime;
        time_point_b.b1 = get_present_time();
        //LOG_INFO("b0 = %lu, b1 = %lu\n", time_point_b.b0, time_point_b.b1);
        simple_udp_sendto(&client_conn, &time_point_b, sizeof(time_point_b), sender_addr);
        LOG_INFO("Send network time synchronization Response time point b to ");
        LOG_INFO_6ADDR(receiver_addr);
        printf("\n");

    } else {
        LOG_INFO("Time synchronization request or response failed.\n");
    }
}

/* rtimer callback function */
void rtimer_call(struct rtimer *timer, void *ptr) {
    LOG_INFO("Rtimer callback function called\n");

    /* re-set the rtimer */
    rtimer_set(&r_timer, RTIMER_NOW() + RTIMER_SECOND / 2, 0, rtimer_call, NULL);
}

/*---------------------------------------------------------------------------*/
PROCESS(hello_world_process, "Hello world process");
AUTOSTART_PROCESSES(&hello_world_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hello_world_process, ev, data)
{

  int is_root = 0;
  static int value;
  static int channel_max;
  static int request_times;
  static struct etimer p_timer;
  //static struct etimer timer;
  button_hal_button_t *btn;
  int round;

  PROCESS_BEGIN();

  leds_set(LEDS_ALL);
  LOG_INFO("wait for setting for root.\n");
  while(0) {
      /* yield the process. */
      PROCESS_YIELD();
      if(ev == button_hal_press_event) {
          btn = (button_hal_button_t *)data;
          if(btn->pin == (uint8_t)14) {
              LOG_INFO("set as a root.\n");
              is_root = 1;
              leds_on(LEDS_ALL);
              break;
          } else if(btn->pin == (uint8_t)13) {
              LOG_INFO("set as a ordinary node.\n");
              is_root = 0;
              leds_off(LEDS_ALL);
              break;
          } else {
              LOG_INFO("Wrong set root.\n");
          }
      }
  }

  /* Initialize DAG root */
  if(is_root) {
      NETSTACK_ROUTING.root_start();
  }

  simple_udp_register(&server_conn, UDP_SERVER_PORT, NULL, UDP_CLIENT_PORT, udp_rx_callback);
  simple_udp_register(&client_conn, UDP_CLIENT_PORT, NULL, UDP_SERVER_PORT, NULL);

  //rtimer_set(&r_timer, RTIMER_NOW() + RTIMER_SECOND / 2, 0, rtimer_call, NULL);
  etimer_set(&p_timer, random_rand() % SEND_INTERVAL);

  /* get the max channel quantity. */
  NETSTACK_RADIO.get_value(RADIO_CONST_CHANNEL_MAX, &value);
  channel_max = value;
  LOG_INFO("Channel max = %d\n", channel_max);
  /* get the cc1310 device's working channel at present. */
  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, 10);
  NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &value);
  LOG_INFO("Radio working channel = %d\n", value);

  /* -1 indicates NULL for synchronization times */
  stime = 0;


  while(1) {
      /* Wait for the periodic timer to expire and then restart the timer. */
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&p_timer));

      if(NETSTACK_ROUTING.node_is_reachable() && !NETSTACK_ROUTING.node_is_root()) {
          if(request_times < ROUND) {

              parent_addr = get_parent_node();
              uint32_t now = get_present_time() - offset;
              rTT.start_time = now;
              /* request the time synchronization from the parent node. */
              simple_udp_sendto(&client_conn, &now, sizeof(now), &parent_addr);

              /* new present time */
              time_point_a.a0 = now;
              LOG_INFO("Send network time synchronization Requests to ");
              LOG_INFO_6ADDR(&parent_addr);
              printf("\n");

              round = 0;
              while(round < 100) {
                  LOG_INFO("Network time = %lu\n ", (get_present_time() - offset));
                  etimer_set(&p_timer, SEND_INTERVAL + (random_rand() % (2 * CLOCK_SECOND)) - CLOCK_SECOND);
                  //etimer_set(&p_timer, SEND_INTERVAL/(CLOCK_SECOND));
                  round++;
              }
              /* Add some jiiter, reset etimer */
              //etimer_set(&p_timer, SEND_INTERVAL/5 + (random_rand() % (2 * CLOCK_SECOND)) - CLOCK_SECOND);
              etimer_set(&p_timer, SEND_INTERVAL + (random_rand() % (2 * CLOCK_SECOND)) - CLOCK_SECOND);
              request_times++;
          } else if(request_times >= ROUND && (get_present_time() - offset) >= 1000000){
              LOG_INFO("Network time = %lu\n ", (get_present_time() - offset));
              etimer_set(&p_timer, SEND_INTERVAL/(CLOCK_SECOND * 20));
          } else {
              LOG_INFO("present time < 1000000\n");
              etimer_set(&p_timer, SEND_INTERVAL/(CLOCK_SECOND * 10));
          }
      } else if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.node_is_root()) {

          /* time synchronization times add 1 */
          LOG_INFO("#%d Network root time = %lu\n ", stime, get_present_time());
          //etimer_set(&p_timer, SEND_INTERVAL/(20 * 100) + (random_rand() % (2 * CLOCK_SECOND)) - CLOCK_SECOND);
          etimer_set(&p_timer, SEND_INTERVAL/CLOCK_SECOND);
      } else {
          /* Add some jiiter, reset etimer */
          etimer_set(&p_timer, SEND_INTERVAL + (random_rand() % (2 * CLOCK_SECOND)) - CLOCK_SECOND);
          LOG_INFO("Not reachable yet\n");
      }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
