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
PROCESS(basestation_process, "Clicker basestation");
/* The basestation process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&basestation_process);

#define HDR_SIZE 6

//#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static int is_coordinator;
#if UIP_CONF_ROUTER
static uip_ipaddr_t ipaddr;
#endif /* UIP_CONF_ROUTER */
//CLIENT
#define SEND_INTERVAL		CLOCK_SECOND*3
#define MAX_PAYLOAD_LEN		40




/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
 * As the client does not need to receive, the function does not do anything
 */




static void print_packet(const void *data, uint8_t len) 
{
    int8_t last_rssi;
    printf("I'm here\n");
    /* print sequence number, timestam, data (in hex) and RSSI of received data */
    last_rssi = 0;//-(nrf_radio_rssi_sample_get());
    printf("%u|%lu|%lu|", *((uint16_t *)data), *((uint32_t *)(data+1)), ((uint32_t)RTIMER_NOW())) ;
    uint8_t i;
    for (i=0; i<len-HDR_SIZE; i++) {
      printf("%02x ", *((uint8_t *)(data+i+HDR_SIZE)));
    }
    printf("|%d\n", last_rssi);

}

static void
tcpip_handler(void)
{

        if(uip_newdata()) {
            ((char *)uip_appdata)[uip_datalen()] = 0;
          
            print_packet(uip_appdata,uip_datalen());
        }

    
}

/* Our main process. */
PROCESS_THREAD(basestation_process, ev, data) {
    

	PROCESS_BEGIN();

    is_coordinator = 1;

    if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
    }
    NETSTACK_MAC.on();

      #if UIP_CONF_ROUTER
      uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
      uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
      uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);
      #endif /* UIP_CONF_ROUTER */

      tcp_listen(UIP_HTONS(1010));
      while(1) {
          PROCESS_YIELD();
          if(ev == tcpip_event) {
          tcpip_handler();
          }
      }
        
	PROCESS_END();
}