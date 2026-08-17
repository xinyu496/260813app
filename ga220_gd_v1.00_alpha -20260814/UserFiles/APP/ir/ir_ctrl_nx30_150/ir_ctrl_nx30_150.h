#ifndef __IR_CTRL_NX30_150_H
#define __IR_CTRL_NX30_150_H

#include "Common/base_inc.h"
#if (IR_CTRL_INCLUDE&IR_NX30_150)
//协议格式
#if 1
#pragma pack(1)
typedef struct  
{
	uint8_t Header1; 
	uint8_t Header2;
	uint8_t ctrl_cmd1;
	uint8_t ctrl_cmd2;
	uint16_t ctrl_data;
	uint8_t add;
}IR_NX30_CTRL_T;
#define IR_NX30_CTRL_LEN  sizeof(IR_NX30_CTRL_T)	
#endif

typedef enum
{
	IR_NX30_SELFCHECK_ON = 0x1,	/*实时反馈开*/
	IR_NX30_SELFCHECK_OFF = 0x3,/*实时反馈关*/
	IR_NX30_CUR_ZOOM_RSP = 0x81,	/*实时变倍电压反馈*/
	IR_NX30_CUR_FOCUS_RSP = 0x83,		/*实时聚焦电压返回格式*/
	IR_NX30_FOCUS_AUTO = 0x5d,/*单次触发自动聚焦*/
	IR_NX30_VIEW_FOCUS = 0x15,/*视场变倍*/
	IR_NX30_CONTINUE_ZOOM = 0x08,/*连续变倍*/
	IR_NX30_SLIGHT_ZOOM_MINUS = 0x5500,/*微动变倍加*/
	IR_NX30_SLIGHT_ZOOM_ADD = 0x5600,/*微动变倍减*/
	IR_NX30_SLIGHT_FOCUS_MINUS = 0x5700,/*微动聚焦加*/
	IR_NX30_SLIGHT_FOCUS_ADD = 0x5800,/*微动聚焦减*/
	IR_NX30_CMD_END,
}IR_NX30_SEND_ENUM;
#if 0
typedef enum
{
	IRFOCUS_STEP = 2,
	IRFZOOM_STEP = 2,

	IRLIGHT_RESET = 100,
	IRCONTRST_RESET = 100,
	IRDDE_RESET = 100,

	IRLIGHT_STEP = 1,
	IRCONTRST_STEP = 1,
	IRDDE_STEP = 1,

	IR_PARA_END,
}IR_CTRL_PARA_TYPE;
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_NX30_API_Serial_Data_Init(void);
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
uint8_t Ir_Ctrl_Nx30_SendHandle(SYS_IR_CMD_CTRL send_type,uint8_t *data);
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
uint8_t IR_API_Nx30_Period_Handle(void);
#endif
#endif