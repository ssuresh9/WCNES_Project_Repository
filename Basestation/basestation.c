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
#if DEBUG
#define PUTSTRING(...) putstring(__VA_ARGS__)
#define PUTHEX(...) puthex(__VA_ARGS__)
#define PUTBIN(...) putbin(__VA_ARGS__)
#define PUTDEC(...) putdec(__VA_ARGS__)
#define PUTCHAR(...) putchar(__VA_ARGS__)
#else
#define PUTSTRING(...)
#define PUTHEX(...)
#define PUTBIN(...)
#define PUTDEC(...)
#define PUTCHAR(...)
#endif

//#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

static int is_coordinator;
#if UIP_CONF_ROUTER
static uip_ipaddr_t ipaddr;
#endif /* UIP_CONF_ROUTER */
//CLIENT
static struct etimer et;
static uip_ipaddr_t addr;
static char *str;
#define SEND_INTERVAL		CLOCK_SECOND*3
#define MAX_PAYLOAD_LEN		40



/* Holds the number of packets received. */
#if 0
static int count = 0;

static int8_t last_rssi;


static void print_packet(const void *data, uint8_t len) {
    /* print sequence number, timestam, data (in hex) and RSSI of received data */
    last_rssi = -(nrf_radio_rssi_sample_get());
    printf("%u|%lu|%lu|", *((uint16_t *)data), *((uint32_t *)(data+1)), ((uint32_t)RTIMER_NOW())) ;
    for (uint8_t i=0; i<len-HDR_SIZE; i++) {
      printf("%02x ", *((uint8_t *)(data+i+HDR_SIZE)));
    }
    printf("|%d\n", last_rssi);

}
#endif
/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
 * As the client does not need to receive, the function does not do anything
 */
#if 0
static void recv(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest) {
    
    count++;
    /* 0bxxxxx allows us to write binary values */
    /* for example, 0b10 is 2 */
    leds_off(LEDS_ALL);
    leds_on(count & 0b1111);
    
    print_packet(data, len);
}
#endif
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

static char buf[MAX_PAYLOAD_LEN] = "hello, world";
//static int seq_id;
static void
timeout_handler(void)
{

  printf("Client sending to: ");
  /* sprintf(buf, "Hello %d from the client", ++seq_id); */
  printf(" (msg: %s)\n\r", buf);
  uip_send(buf, strlen(buf));
}


static void
tcpip_handler(void)
{
    if(1==is_coordinator)
    {//SERVER
        static int seq_id;
        static char buf[MAX_PAYLOAD_LEN];

        if(uip_newdata()) {
            ((char *)uip_appdata)[uip_datalen()] = 0;
            PRINTF("Server received: ");
            PRINTF(((char *)uip_appdata));
            PRINTF(" from ");
            PRINT6ADDR(&UIP_IP_BUF->srcipaddr);
            PRINTF("\n\r");

            PRINTF("Responding with message: ");
            sprintf(buf, "Hello from the server! (%d)", ++seq_id);
            PRINTF(buf);
            PRINTF("\n\r");

            uip_send(buf, strlen(buf));
        }

    }
    else{//CLIENT

            if(uip_newdata()) {
            str = uip_appdata;
            str[uip_datalen()] = '\0';
            printf("Response from the server: '%s'\n\r", str);
                }

    }
}

/* Our main process. */
PROCESS_THREAD(basestation_process, ev, data) {
    
    static bool client_connected=0;
    uip_ipaddr_t dest_ipaddr;

	PROCESS_BEGIN();

    is_coordinator = 0;

    #if CONTIKI_TARGET_COOJA || CONTIKI_TARGET_Z1
    is_coordinator = (node_id == 1);
    #endif
    //is_coordinator=1;

    if(is_coordinator) {
    NETSTACK_ROUTING.root_start();
    }
    NETSTACK_MAC.on();

    if(1==is_coordinator)
    {
        //server

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
        
    }
    else
    {
        //client
            PRINTF("TCP client process started\n\r");

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
                            if(uip_aborted() || uip_timedout() || uip_closed()) {
                            printf("Could not establish connection\n\r");
                            } 
                            else if(uip_connected()) {
                            printf("Connected\n\r");
                            client_connected=1;
                            }
                    }
                        

                        PROCESS_YIELD();
                        if(etimer_expired(&et)) {
                            if(1==client_connected)
                            {
                                timeout_handler();
                            }
                        etimer_restart(&et);
                        } else if(ev == tcpip_event) {
                        tcpip_handler();
                        }
                    

            

            

             }
    }

    #if 0
    #if WITH_PERIODIC_ROUTES_PRINT
    {
        static struct etimer et;
        /* Print out routing tables every minute */
        etimer_set(&et, CLOCK_SECOND * 10);
        while(1) {
        /* Used for non-regression testing */
        #if (UIP_MAX_ROUTES != 0)
       // PRINTF("Routing entries: %u\n", uip_ds6_route_num_routes());
        #endif
        #if (UIP_SR_LINK_NUM != 0)
        //PRINTF("Routing links: %u\n", uip_sr_num_nodes());
        #endif
        PROCESS_YIELD_UNTIL(etimer_expired(&et));
        etimer_reset(&et);
        }
    }
    #endif /* WITH_PERIODIC_ROUTES_PRINT */

    static struct etimer timer;
	/* Initialize NullNet */
	//nullnet_set_input_callback(recv);
    

    /* Setup a periodic timer that expires after 10 seconds. */
    etimer_set(&timer, CLOCK_SECOND * 10);
    
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
        etimer_restart(&timer);
        /* print statistics */
        // printf("\n--- statistics: received %d packets / 10s\n", count);
        count = 0;
    }
 #endif   
    
	PROCESS_END();
}
