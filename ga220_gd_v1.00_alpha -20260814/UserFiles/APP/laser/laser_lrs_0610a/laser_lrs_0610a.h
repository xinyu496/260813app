#ifndef __LASER_IRS_0610A_H
#define __LASER_IRS_0610A_H
#include "Common/base_inc.h"
#include "Common/config.h"


#if (LASER_LRS_0610A||LASER_LSP_0410)
//接收主控数据
//与主控板通讯处理
typedef enum
{
	LASER_DATA_STOP = 0x0,			/*测距停止*/
	LASER_SINGLE_DETECT = 0x1,		/*单次测距*/
	LASER_DETECT_CONTINUE = 0x2,	/*连续测距*/
	LASER_PERIOD_SELF_CHECK = 0x3,	/*自检*/
	LASER_RANGE_CFG = 0x4,			/*选通值设置*/
	LASER_LIGHT_CNT_REQ = 0x6,		/*累计出光次数查询*/
	LASER_HIGH_POWER_BASE_VALUE,	/*高压基准值设置*/
	LASER_FAR_DISANCE = 0xB,		/*最远距离设置*/
	LASER_FAR_DISANCE_REQ = 0xD,	/*最远距离查询*/
	LASER_APD_POWER_ENABLE = 0x11,	/*APD电源开*/
	LASER_APD_POWER_DISENABLE = 0x12,/*APD电源关*/
	LASER_LIGHT_CNT_RES = 0x1a,		/*出光次数清0*/
	LASER_DETECT_PLUSE = 0x1b,		/*激光触发脉冲测试*/ 
	LASER_UPGRADE = 0x1c,			/*程序升级*/
	LASER_TIMEOUT_VALUE = 0x20,   /*激光连续工作超时时间设置*/
	LASER_WORK_MODE_CHOOSE = 0x22,/*工作模式设置-首末目标設置 0：单目标；1：三目标，0010：首末目标*/
	LASER_CFG_TPG = 0x23,
	LASER_DEECT_PARA_REQ = 0x28,	/*测距参数查询*/
	LASER_LOCALID_REQ = 0xEB,				/*编号查询*/
/*LASER_LSP_0410*/
	LASER_DETECT_5HZ = 0x5,				/*连续测距(5Hz)*/
	LASER_SET_AIM = 0xEA,					/*设置目标*/
	/*回报报文指令*/
	LASER_RSP_WORK_OUT = 0xED,		/*工作超时*/
	LASER_RSP_CHECK_ERR = 0xEE,		/*效验错误*/
	LASER_RSP_CONECT_OUT = 0xEF,	/*串口通信超时*/

	LASER_CMD_END,
}LASER_DATA_IN_TYPE_INFO;
//发送和接收报文的格式一致，共用一个结构体
#pragma pack(1)
typedef struct  
{
	uint8_t Header;        
	uint8_t Ctrl_Cmd;
	uint8_t data_len;
	uint16_t data_para;
	uint8_t Xor;
}LASER_DATA_LRS_T;
#pragma pack()
#define LASER_LRS_DATA_LEN  sizeof(LASER_DATA_LRS_T)
	
/*激光目标状态*/
typedef struct	
{
	uint8_t last_dest:1;//1：有后目标
	uint8_t first_dest:1;	//1：有前目标
	uint8_t apd_sta:1; 	//1：APD正常
	uint8_t res:1;
	uint8_t time_out:1;	//1：正常
	uint8_t sys_sta:1;	//1：激光正常
	uint8_t main_lobe:1; //1：有主波
	uint8_t echo_lobe:1;//1：有回波
}LASER_0610_RECV_T;

/*激光目标状态*/
typedef struct	
{
	uint8_t last_dest:1;//1：有后目标
	uint8_t first_dest:1;	//1：有前目标
	uint8_t apd_sta:1; 	//1：APD正常
	uint8_t overhang:1;//1:超距
	uint8_t res:1;	//1：正常
	uint8_t sys_sta:1;	//1：激光正常
	uint8_t main_lobe:1; //1：有主波
	uint8_t echo_lobe:1;//1：有回
}LASER_0410_RECV_T;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
RECV_DATA_ERR_STA LASER_API_Period_Handle(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void laser_process_data_in(uint8_t *data_in,uint32_t length);
#endif
#endif
