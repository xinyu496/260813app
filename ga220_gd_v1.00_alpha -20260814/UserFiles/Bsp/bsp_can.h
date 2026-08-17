#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#if defined (STM32H743xx)
#include "stm32h7xx_hal.h"
#endif

#if defined (STM32F405xx)
#include "stm32f4xx_hal.h"
#endif

#include "can.h"

#define CAN_ID 0x00008000

#define CANFD_ID1 0x00008000
#define CANFD_ID2 0x00008000


/************普通帧can数据***************/
__packed typedef struct  
{
	uint8_t 	Data1;
	uint8_t 	Data2;
	uint8_t 	Data3;
	uint8_t 	Data4;
	uint8_t 	Data5;
	uint8_t 	Data6;
	uint8_t 	Data7;
	uint8_t 	Data8;
}RX_CAN;

__packed typedef struct  
{
	uint8_t 	Data1;
	uint8_t 	Data2;
	uint8_t 	Data3;
	uint8_t 	Data4;
	uint8_t 	Data5;
	uint8_t 	Data6;
	uint8_t 	Data7;
	uint8_t 	Data8;
}TX_CAN;


__packed typedef struct  
{
	uint32_t Number_Frame:3;//帧序号
	uint32_t Total_Quantity:3;//多帧总数量
	uint32_t Marking_Frame:1;//多帧标识
	uint32_t Address_Identification:3;//识别地址
	uint32_t Node_Goal:14;//目的节点
	uint32_t Node_Source:4;//源节点
	uint32_t Marking_Precedence:1;//优先标识 0008
	uint32_t RxFlag:3;//接收数据标识
}UDID_CanBus;//CAN总线接收标识符分段定义(32位)




void COM_Rcv_CanPort_Init(void);

void CAN_Send(uint8_t com_can_id , uint8_t *sendbuff , uint32_t id , uint8_t len);

#if CANFD_ENABLE
void CANFD_Send(uint8_t com_can_id , uint8_t *sendbuff , uint32_t id , uint8_t len);
#endif

#endif

