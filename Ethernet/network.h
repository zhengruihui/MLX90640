/*
 * network.h
 *
 *  Created on: 2018Äê10ÔÂ25ÈÕ
 *      Author: zhengruihui
 */

#ifndef INTERNET_NETWORK_H_
#define INTERNET_NETWORK_H_

#include "stdint.h"

#define READ_BUFFER_SIZE            128
#define WRITE_BUFFER_SIZE           1544



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
