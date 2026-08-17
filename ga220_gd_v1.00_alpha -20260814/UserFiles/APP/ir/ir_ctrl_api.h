#ifndef __IR_CTRL_API_H
#define __IR_CTRL_API_H
#include "Common/base_inc.h"

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void Infrared_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Infrared_API_Ctrl_SendHandle(uint8_t ir_type,uint8_t send_type,uint8_t *data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Infrared_API_Period_Handle(void);
#endif

