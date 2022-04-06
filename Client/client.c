#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"

/* Declare our "main" process, the client process*/
PROCESS(client_process, "Clicker client");
/* The client process should be started automatically when
 * the node has booted. */
AUTOSTART_PROCESSES(&client_process);

#define FILE_SIZE 10000
#define PKT_SIZE 32
#define HDR_SIZE 5


static linkaddr_t dest_addr =        {{ 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }};

/* including a header to the packet:
 * - 1B sequence number
 * - 4B tx_timestamp
 * 
 * packet: buffer to be updated with the header
 * seq: sequence number of the packet
 */
static void add_header(uint8_t *packet, uint8_t seq) {
    static uint32_t timestamp = 0;
    /* use first byte of the packet as sequence number. */
    packet[0] = seq & 0xff;
    /* four bytes of timestamp */
    timestamp = (uint32_t)RTIMER_NOW();
    memcpy(&packet[1], &timestamp, 4);
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
    for (uint8_t i=0; i<length; i++) {          // generate the random payload byte by byte
        num = (num * A1 + C1) & RAND_MAX1;
        packet[i] = (uint8_t)(num & MAX_BYTE);

    }                 
}


/* Callback function for received packets.
 *
 * Whenever this node receives a packet for its broadcast handle,
 * this function will be called.
 *
 * As the client does not need to receive, the function does not do anything
 */
static void send(const void* data, uint16_t len) {
    memcpy(nullnet_buf, data, len);
    nullnet_len = len;
    NETSTACK_NETWORK.output(&dest_addr);
}

static void recv(const void *data, uint16_t len,
    const linkaddr_t *src, const linkaddr_t *dest) {
}

/* Our main process. */
PROCESS_THREAD(client_process, ev, data) {
    static uint8_t buffer[PKT_SIZE + HDR_SIZE];  // include 5 header bytes
    static uint8_t counter = 0;
    static struct etimer timer;

	PROCESS_BEGIN();
    
	/* Initialize NullNet */
	nullnet_buf = (uint8_t *)&buffer;
	nullnet_len = sizeof(buffer);
	nullnet_set_input_callback(recv);
    NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL,20);

    /* Setup a periodic timer that expires after 2 seconds. */
    etimer_set(&timer, CLOCK_SECOND / 1);
    
	/* Loop forever. */
	while (1) {
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
        etimer_restart(&timer);

        /* add header (3 bytes) */
        add_header(&buffer[0], counter);
        /* include random sequence as payload */
        compute_sequence(&buffer[HDR_SIZE], counter, PKT_SIZE);
        /* send data */
        send(&buffer, sizeof(buffer));

        counter++;
    }

	PROCESS_END();
}
