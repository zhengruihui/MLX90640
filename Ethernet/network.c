/*
 * network.c
 *
 *  Created on: 2018Äê10ÔÂ25ÈÕ
 *      Author: zhengruihui
 */






#include "network.h"
#include "w5500.h"
#include "socket.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "delay.h"



struct network net_work;




void network_init(void)
{
    //hardware reset wiz5500 module
	unsigned char tx_mem_conf[8] = {16,0,0,0,0,0,0,0};           // for setting TMSR register
	unsigned char rx_mem_conf[8] = {16,0,0,0,0,0,0,0};          // for setting RMSR register
	
	
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);


	wizchip_init(tx_mem_conf,rx_mem_conf);

	socket(0,Sn_MR_UDP,DHCP_CLIENT_PORT,0);


	  net_work.allocated_ip[0] = 192;
	  net_work.allocated_ip[1] = 168;
	  net_work.allocated_ip[2] = 1;
	  net_work.allocated_ip[3] = 161;

	
	  net_work.allocated_sn[0] = 255;
	  net_work.allocated_sn[1] = 255;
	  net_work.allocated_sn[2] = 255;
	  net_work.allocated_sn[3] = 0;
	
	  net_work.allocated_gw[0] = 192;
	  net_work.allocated_gw[1] = 168;
	  net_work.allocated_gw[2] = 1;
	  net_work.allocated_gw[3] = 1;
		
	  net_work.allocated_mac[0] = 0;
	  net_work.allocated_mac[1] = 8;
	  net_work.allocated_mac[2] = 220;
	  net_work.allocated_mac[3] = 0;
	  net_work.allocated_mac[4] = 111;
	  net_work.allocated_mac[5] = 210;
		
		
	  setSHAR(net_work.allocated_mac);
	  setGAR (net_work.allocated_gw);
      setSUBR(net_work.allocated_sn);
	setSIPR(net_work.allocated_ip);



	
}







