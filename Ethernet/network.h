/*
 * network.h
 *
 *  Created on: 2018Äê10ÔÂ25ÈÕ
 *      Author: zhengruihui
 */

#ifndef INTERNET_NETWORK_H_
#define INTERNET_NETWORK_H_

#include "stdint.h"
#define DEVICE_ID 0

/* UDP port numbers for DHCP */
#if DEVICE_ID==0
#define DHCP_CLIENT_PORT      	5500	
#endif
#if DEVICE_ID==1
#define DHCP_CLIENT_PORT      	5501	      
#endif
#if DEVICE_ID==2
#define DHCP_CLIENT_PORT      	5502	      
#endif

#define DHCP_SERVER_PORT        5507	

#define READ_BUFFER_SIZE            128
#define WRITE_BUFFER_SIZE           1024



struct network
{
    unsigned char allocated_ip[4];
    unsigned char allocated_sn[4];
    unsigned char allocated_gw[4];
    unsigned char allocated_dns[4];
	  unsigned char allocated_mac[6];

    uint8_t network_state;

    uint8_t desip[4];
    unsigned char vsip[4];
    uint16_t *dport;

    uint8_t read_buffer[READ_BUFFER_SIZE];
    uint8_t write_buffer[WRITE_BUFFER_SIZE];

};

//
// Function Prototypes
//
void network_init(void);
void network_run(void);

extern struct network net_work;

#endif /* INTERNET_NETWORK_H_ */
