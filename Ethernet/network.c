/*
 * network.c
 *
 *  Created on: 2018Äê10ÔÂ25ÈÕ
 *      Author: zhengruihui
 */

///*
// * @brief return value of @ref DHCP_run()
// */
//enum
//{
//   DHCP_FAILED = 0,  ///< Processing Fail
//   DHCP_RUNNING,     ///< Processing DHCP protocol
//   DHCP_IP_ASSIGN,   ///< First Occupy IP from DHPC server      (if cbfunc == null, act as default default_ip_assign)
//   DHCP_IP_CHANGED,  ///< Change IP address by new ip from DHCP (if cbfunc == null, act as default default_ip_update)
//   DHCP_IP_LEASED,   ///< Stand by
//   DHCP_STOPPED      ///< Stop processing DHCP protocol
//};



#include "network.h"
#include "dhcp.h"
#include "w5500.h"
#include "socket.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "delay.h"
//uint8_t dhcp_buffer[1024];





struct network net_work;


void network_ip_assign(void)
{
    getIPfromDHCP(net_work.allocated_ip);
    getSNfromDHCP(net_work.allocated_sn);
    getGWfromDHCP(net_work.allocated_gw);

    getDNSfromDHCP(net_work.allocated_dns);

    setSIPR(net_work.allocated_ip);
    setSUBR(net_work.allocated_sn);
    setGAR (net_work.allocated_gw);
}

void network_ip_update(void)
{

}

void network_ip_conflict(void)
{

}


void network_init(void)
{
    //hardware reset wiz5500 module
	unsigned char tx_mem_conf[8] = {16,0,0,0,0,0,0,0};           // for setting TMSR register
	unsigned char rx_mem_conf[8] = {16,0,0,0,0,0,0,0};          // for setting RMSR register
	
	
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100);
	HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

    //reg_dhcp_cbfunc(network_ip_assign, network_ip_update, network_ip_conflict);

    //DHCP_init(0, dhcp_buffer);
	
	  net_work.allocated_ip[0] = 192;
	  net_work.allocated_ip[1] = 168;
	  net_work.allocated_ip[2] = 1;
	  net_work.allocated_ip[3] = 79;

	
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
	  net_work.allocated_mac[5] = 202;
		
		
		setSHAR(net_work.allocated_mac);
		setGAR (net_work.allocated_gw);
    setSUBR(net_work.allocated_sn);
		setSIPR(net_work.allocated_ip);

		
		

	wizchip_init(tx_mem_conf,rx_mem_conf);

	socket(0,Sn_MR_UDP,DHCP_SERVER_PORT,0);
	
	net_work.desip[0] = 192;
    net_work.desip[1] = 168;
    net_work.desip[2] = 1;
    net_work.desip[3] = 139;
	
}

void network_run(void)
{
    if(DHCP_run() == DHCP_IP_LEASED)
    {
        net_work.network_state = DHCP_IP_LEASED;
    }
}








