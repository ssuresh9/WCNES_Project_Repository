#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "dev/leds.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "nrf_radio.h"


/* Declare our "main" process, the basestation_process */
PROCESS(basestation_process, "Clicker basestation");
/* The basestation process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&basestation_process);

#define HDR_SIZE 6

/* Holds the number of packets received. */
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

/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
 * As the client does not need to receive, the function does not do anything
 */
static void recv(const void *data, uint16_t len,
  const linkaddr_t *src, const linkaddr_t *dest) {
    
    count++;
    /* 0bxxxxx allows us to write binary values */
    /* for example, 0b10 is 2 */
    leds_off(LEDS_ALL);
    leds_on(count & 0b1111);
    
    print_packet(data, len);
}

/* Our main process. */
PROCESS_THREAD(basestation_process, ev, data) {
	PROCESS_BEGIN();

    static struct etimer timer;
	/* Initialize NullNet */
	nullnet_set_input_callback(recv);
    

    /* Setup a periodic timer that expires after 10 seconds. */
    etimer_set(&timer, CLOCK_SECOND * 10);
    
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
        etimer_restart(&timer);
        /* print statistics */
        // printf("\n--- statistics: received %d packets / 10s\n", count);
        count = 0;
    }
    
    
	PROCESS_END();
}
