/*
 * network.h
 *
 *  Created on: 2018��10��25��
 *      Author: zhengruihui
 */

#ifndef __network_H
#define __network_H

#ifdef __cplusplus
 extern "C" {
#endif


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

#ifdef __cplusplus
}
#endif


#endif /* INTERNET_NETWORK_H_ */

