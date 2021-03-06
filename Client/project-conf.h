/*
 * Copyright (c) 2015, SICS Swedish ICT.
 * Copyright (c) 2017, University of Bristol - http://www.bristol.ac.uk
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
 * \author Simon Duquennoy <simonduq@sics.se>
 *         Atis Elsts <atis.elsts@bristol.ac.uk>
 */

#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__

/* Set to enable TSCH security */
#ifndef WITH_SECURITY
#define WITH_SECURITY 0
#endif /* WITH_SECURITY */

/* USB serial takes space, free more space elsewhere */
#define SICSLOWPAN_CONF_FRAG 0
#define UIP_CONF_BUFFER_SIZE 160

/*******************************************************/
/******************* Configure TSCH ********************/
/*******************************************************/
#define MAC_CONF_WITH_TSCH 1

/* IEEE802.15.4 PANID */
#define IEEE802154_CONF_PANID 0x81a5

/* Do not start TSCH at init, wait for NETSTACK_MAC.on() */
#define TSCH_CONF_AUTOSTART 0

/* 6TiSCH minimal schedule length.
 * Larger values result in less frequent active slots: reduces capacity and saves energy. */
#define TSCH_SCHEDULE_CONF_DEFAULT_LENGTH 3

#if WITH_SECURITY

/* Enable security */
#define LLSEC802154_CONF_ENABLED 1

#endif /* WITH_SECURITY */

/* Enable TSCH statistics */
#define TSCH_STATS_CONF_ON 1

/* Enable periodic RSSI sampling for TSCH statistics */
#define TSCH_STATS_CONF_SAMPLE_NOISE_RSSI 1

/* Reduce the TSCH stat "decay to normal" period to get printouts more often */
#define TSCH_STATS_CONF_DECAY_INTERVAL (60 * CLOCK_SECOND)
#define TSCH_CONF_ASSOCIATION_POLL_FREQUENCY 5

/*Enable TCP*/
#define UIP_CONF_UDP 0
#define UIP_CONF_TCP 1
#define UIP_CONF_ICMP6 1

/*******************************************************/
/************* Other system configuration **************/
/*******************************************************/

/* Logging */
#define LOG_CONF_LEVEL_RPL                         LOG_LEVEL_INFO
#define LOG_CONF_LEVEL_TCPIP                       LOG_LEVEL_WARN
//#define LOG_CONF_LEVEL_IPV6                        LOG_LEVEL_WARN
//#define LOG_CONF_LEVEL_6LOWPAN                     LOG_LEVEL_WARN
//#define LOG_CONF_LEVEL_MAC                         LOG_LEVEL_DBG
//#define LOG_CONF_LEVEL_FRAMER                      LOG_LEVEL_INFO
//#define TSCH_LOG_CONF_PER_SLOT                     1

#define TSCH_CONF_EB_PERIOD (CLOCK_SECOND/16)
#define TSCH_CONF_MAX_EB_PERIOD (CLOCK_SECOND/2)
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE TSCH_HOPPING_SEQUENCE_8_8
#define TSCH_CONF_CCA_ENABLED 1
#define TSCH_CONF_CHANNEL_SCAN_DURATION (CLOCK_SECOND/4)
#define TSCH_CONF_ASSOCIATION_POLL_FREQUENCY 5
#define TSCH_CONF_KEEPALIVE_TIMEOUT (60*CLOCK_SECOND)
#define TSCH_CONF_MAX_KEEPALIVE_TIMEOUT (120*CLOCK_SECOND)

#define RPL_CONF_WITH_PROBING 0
#define RPL_CONF_PREFERENCE 0

//DIO REDUNDANCY.
//DAG TIMER LIFETIME.
//MAKE CLIENT a LEAF NODE

//INCREASE DIS TIMER NODE SPECIFICALLY.
//DIO INTERVAL MIN


#endif /* __PROJECT_CONF_H__ */
