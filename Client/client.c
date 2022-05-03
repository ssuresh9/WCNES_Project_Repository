/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * Copyright (c) 2018, University of Bristol - http://www.bristol.ac.uk
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
 */
/**
 * \file
 *         A RPL+TSCH node demonstrating application-level time syncrhonization.
 *
 * \author Atis Elsts <atis.elsts@bristol.ac.uk>
 *         Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "net/routing/routing.h"
#include "net/netstack.h"
#include "net/ipv6/simple-udp.h"
#include "net/mac/tsch/tsch.h"
#include "lib/random.h"
#include "sys/node-id.h"
#include "nrf_radio.h"

#include "sys/log.h"
#define LOG_MODULE "App"
#define LOG_LEVEL LOG_LEVEL_INFO

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define SEND_INTERVAL		  ( CLOCK_SECOND/2)

#define FILE_SIZE 10000
#define PKT_SIZE 32
#define HDR_SIZE 6
/*---------------------------------------------------------------------------*/
static struct simple_udp_connection client_conn, server_conn;
static int8_t last_rssi;

PROCESS(node_process, "RPL Node");
AUTOSTART_PROCESSES(&node_process);

/* including a header to the packet:
 * - 1B sequence number
 * - 4B tx_timestamp
 * 
 * packet: buffer to be updated with the header
 * seq: sequence number of the packet
 */
static void add_header(uint8_t *packet, uint16_t seq) {
    static uint32_t timestamp = 0;
  //  uint16_t dummy;
    /* use first byte of the packet as sequence number. */
    packet[0] = (uint8_t)(seq & 0x00ff);
    packet[1] = (uint8_t)((seq>>8)& 0x00ff);
  //  dummy=((uint16_t)(packet[1]<<8)|(uint16_t)(packet[0]));

  //  printf("packet print:%i ",dummy);
    /* four bytes of timestamp */
    timestamp = (uint32_t)RTIMER_NOW();
    memcpy(&packet[2], &timestamp, 4);
}

/* generaion of a random sequence.
 * 
 * packet: buffer to be filled with the random sequence
 * seed: seed to initialize the random generator. 
 *       (to have unique and reproducable sequences for every packet. 
          Use the sequence number as seed!)
 * length: length of the sequence
 */
static  void compute_sequence(uint8_t *packet, uint8_t seed, uint8_t length) {
    const uint32_t A1 = 1664525;
    const uint32_t C1 = 1013904223;
    const uint32_t RAND_MAX1 = (((uint32_t)1 << 31) - 1);
    const uint8_t MAX_BYTE = ((1<<8) - 1);      // one byte
    uint8_t num = seed | (1<<4);                // seed (4 bites) is part of the seq number
    uint8_t i;
    for (i=0; i<length; i++) {          // generate the random payload byte by byte
        num = (num * A1 + C1) & RAND_MAX1;
        packet[i] = (uint8_t)(num & MAX_BYTE);

    }                 
}
static void print_packet(const void *data, uint8_t len) {
    /* print sequence number, timestam, data (in hex) and RSSI of received data */
    last_rssi = -(nrf_radio_rssi_sample_get());
    printf("%u|%lu|%lu|", *((uint16_t *)data), *((uint32_t *)(data+1)), ((uint32_t)RTIMER_NOW())) ;
    uint8_t i;
    for (i=0; i<len-HDR_SIZE; i++) {
      printf("%02x ", *((uint8_t *)(data+i+HDR_SIZE)));
    }
    printf("|%d\n", last_rssi);

}
/*---------------------------------------------------------------------------*/
static void
udp_rx_callback(struct simple_udp_connection *c,
         const uip_ipaddr_t *sender_addr,
         uint16_t sender_port,
         const uip_ipaddr_t *receiver_addr,
         uint16_t receiver_port,
         const uint8_t *data,
         uint16_t datalen)
{
  print_packet(data,datalen);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(node_process, ev, data)
{
  static struct etimer periodic_timer;
  int is_coordinator;
  uip_ipaddr_t dest_ipaddr;
  static uint8_t buffer[PKT_SIZE + HDR_SIZE];  // include 5 header bytes
    static uint16_t counter = 0;

  PROCESS_BEGIN();

  is_coordinator = 0;

  is_coordinator =0;//= (node_id == 1);


  if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
  }

  /* Initialize UDP connections */
  simple_udp_register(&server_conn, UDP_SERVER_PORT, NULL,
                      UDP_CLIENT_PORT, udp_rx_callback);
  simple_udp_register(&client_conn, UDP_CLIENT_PORT, NULL,
                      UDP_SERVER_PORT, NULL);

  NETSTACK_MAC.on();

  etimer_set(&periodic_timer, random_rand() % SEND_INTERVAL);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&periodic_timer));

    if(tsch_is_coordinator) {
      break;
    }

    if(NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr)) {
      /* Loop forever. */

       // printf("Channel:%i",value);

        /* add header (3 bytes) */
        add_header(&buffer[0], counter);
        /* include random sequence as payload */
        compute_sequence(&buffer[HDR_SIZE], counter, PKT_SIZE);
        /* send data */

        counter++;
      simple_udp_sendto(&client_conn, &buffer, sizeof(buffer), &dest_ipaddr);
      LOG_INFO("Sent data to server ");
      LOG_INFO_6ADDR(&dest_ipaddr);
      LOG_INFO_("\n");
    }
     else {
      LOG_INFO("\rNot reachable yet\n");
    }

    /* Add some jitter */
    etimer_set(&periodic_timer, SEND_INTERVAL);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/