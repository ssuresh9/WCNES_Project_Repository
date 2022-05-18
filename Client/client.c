#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "net/ipv6/uip-ds6-route.h"
#include "net/ipv6/uip-sr.h"
#include "net/mac/tsch/tsch.h"
#include "net/routing/routing.h"
#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"
#include "sys/node-id.h"

/* Declare our "main" process, the basestation_process */
PROCESS(client_process, "Client");
/* The client process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&client_process);

//CLIENT
static struct etimer et;
static uip_ipaddr_t addr;
#define SEND_INTERVAL		2*CLOCK_SECOND

#define FILE_SIZE 10000
#define PKT_SIZE 64
#define HDR_SIZE 6

static void add_header(uint8_t *packet, uint16_t seq);
static  void compute_sequence(uint8_t *packet, uint8_t seed, uint8_t length);

static uint16_t counter = 0;
static uint16_t ack_counter = 0;

#if UIP_CONF_ROUTER
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;
  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
}
#endif /* UIP_CONF_ROUTER */

static uint8_t buf[PKT_SIZE + HDR_SIZE];

static void
timeout_handler(void)
{
    // if the last packet was not acknowledged, resend it 
    if(counter>0 && ack_counter < counter-1){
        counter--;
    }

    /* add header (3 bytes) */
    add_header(&buf[0], counter);
    /* include random sequence as payload */
    compute_sequence(&buf[HDR_SIZE], counter, PKT_SIZE);

    printf("Client sending: ");
    uip_send(buf, sizeof(buf));
    printf("%d\n", counter);
    
    counter++;
}

/* including a header to the packet:
 * - 1B sequence number
 * - 4B tx_timestamp
 * 
 * packet: buffer to be updated with the header
 * seq: sequence number of the packet
 */
static void add_header(uint8_t *packet, uint16_t seq) 
{
    static uint32_t timestamp = 0;
    /* use first byte of the packet as sequence number. */
    packet[0] = (uint8_t)(seq & 0x00ff);
    packet[1] = (uint8_t)((seq>>8)& 0x00ff);

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
static  void compute_sequence(uint8_t *packet, uint8_t seed, uint8_t length) 
{
    const uint32_t A1 = 1664525;
    const uint32_t C1 = 1013904223;
    const uint32_t RAND_MAX1 = (((uint32_t)1 << 31) - 1);
    const uint8_t MAX_BYTE = ((1<<8) - 1);      // one byte
    uint8_t num = seed | (1<<4);                // seed (4 bites) is part of the seq number
    uint8_t i;
    for (i=0; i<length; i++) 
    {          // generate the random payload byte by byte
        num = (num * A1 + C1) & RAND_MAX1;
        packet[i] = (uint8_t)(num & MAX_BYTE);
    }                 
}

/* Our main process. */
PROCESS_THREAD(client_process, ev, data) {
    
    static bool client_connected=0;
    uip_ipaddr_t dest_ipaddr;

	PROCESS_BEGIN();

    NETSTACK_MAC.on();

    //client
      printf("TCP client process started\n\r");

      #if UIP_CONF_ROUTER
          set_global_address();
      #endif

      uip_ip6addr(&addr, 0xaaaa, 0, 0, 0, 0x0212, 0x4b00, 0x0260, 0xd0e4);
    
      etimer_set(&et, SEND_INTERVAL);

      while(1) {
        if((0==client_connected)&&NETSTACK_ROUTING.node_is_reachable() && NETSTACK_ROUTING.get_root_ipaddr(&dest_ipaddr))
        {
            tcp_connect(&dest_ipaddr, UIP_HTONS(1010), NULL);

            printf("Connecting...\n\r");
            
            PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
            if(uip_aborted() || uip_timedout() || uip_closed()) 
            {
                printf("Could not establish connection\n\r");
            } 
            else if(uip_connected()) 
            {
                printf("Connected\n\r");
                client_connected=1;
            }
        }
                          
        PROCESS_YIELD();
        
        if(etimer_expired(&et)) 
        {
            if(1==client_connected)
            {
                if(counter==0 || ack_counter < counter-1){
                     timeout_handler();  
                }           
            }
            etimer_restart(&et);
        } 

        else if(ev == tcpip_event) 
        {
            if(1==client_connected)
            {
                if(uip_newdata())
                {
                    timeout_handler();
                    ack_counter++;
                }
            }
        }             
    }
    
	PROCESS_END();
}