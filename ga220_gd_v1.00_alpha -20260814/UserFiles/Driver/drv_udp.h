#ifndef __NET_CFG_H
#define __NET_CFG_H
#include "Common/base_inc.h"
#if ETH_INCLUDE
/*=============================================================
*FUNCTION NAME:
*DISCRIPTION:网络创建PCB
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void Eth_NetWork_Handle(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络报文的发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_senddata(struct udp_pcb *upcb,uint8_t *data,uint16_t Len);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MONITOR_UDP_Config_Init(void);
#endif
#endif