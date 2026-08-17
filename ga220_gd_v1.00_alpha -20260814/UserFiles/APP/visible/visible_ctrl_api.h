#ifndef __VISIBLE_CTRL_H
#define __VISIBLE_CTRL_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "option/opt_module.h"
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:控制指令；
			ctrl_type:指令类型；
			ctrl_data:数据内容；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Visible_API_Ctrl_SendHandle(uint8_t vl_type,uint8_t send_type,uint8_t *ctrl_data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t VISIABLE_API_Period_Handle(void);
#endif
