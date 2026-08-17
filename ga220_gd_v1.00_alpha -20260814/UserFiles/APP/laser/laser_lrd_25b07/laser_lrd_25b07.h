#ifndef __LASER_LRD_25B07_H
#define __LASER_LRD_25B07_H
#include "Common/base_inc.h"
#include "Common/config.h"

/*模块开关*/
#if LASER_LRD_25B07

#define LASER_HEAD1   0xEB
#define LASER_HEAD2   0x90

typedef enum
{
    LASER_CMD_NULL = 0x0,							 /*空指令*/
    LASER_CMD_SELF_CHECK = 0x1,				 /*维护自检*/
	LASER_CMD_STOP = 0x2,							 /*停止测照*/	
    LASER_CMD_SINGLE = 0x3,						 /*单次发射*/
    LASER_CMD_DETECT_FREQ_1 = 0x4,		 /*1hz测距*/
    LASER_CMD_DETECT_MARK = 0x5,			 /*Mark测距 5hz测距（工作2S左右即休息） 参数1：0x64或默认0时，发射1064波长激光*/
    LASER_CMD_DETECT_FREQ_5 = 0xF,		 /*5hz测距（工作5min休息30s） 参数1：0x64或默认0时，发射1064波长激光*/
	LASER_CMD_DETECT_FREQ_20 = 0x1D,	 /*20hz测距*/
	LASER_CMD_DETECT_FREQ_10 = 0x1F,	 /*10hz测距*/
	LASER_CMD_LIGHT1 = 0x6,            /*激光照射1*/
	LASER_CMD_LIGHT2 = 0x26,           /*激光照射2*/
	LASER_CMD_CODE_SET = 0x7,          /*编码装订*/	
	LASER_CMD_LIGHT_QUERY = 0x8,       /*打光次数查询*/		
	LASER_CMD_ENABLE = 0x9,            /*激光使能*/			
	LASER_CMD_DISABLE = 0xA,           /*激光禁止*/		
	LASER_CMD_LOW_POWER_ON = 0xB,      /*低功耗开启*/	
	LASER_CMD_LOW_POWER_OFF = 0xC,     /*低功耗关闭*/	
	LASER_CMD_RX_ON = 0xD,     				 /*激光接收开启*/	
	LASER_CMD_RX_OFF = 0xE,     			 /*激光接收关闭*/	
	LASER_CMD_INNER_SYNC = 0x10,     	 /*切换到内时统*/	
	LASER_CMD_OUTER_SYNC = 0x11,     	 /*切换到内时统*/		
	LASER_CMD_NORMAL_EN = 0x12,     	 /*到正常能量*/	
	LASER_CMD_LOW_EN = 0x13,     	 		 /*到小能量*/	
	LASER_CMD_BLIND_SET = 0x14,     	 /*设置盲区*/	
	LASER_CMD_AVG_EN = 0x1C,     			 /*距离平均使能/禁用*/	
} LASER_CMD_E;

#pragma pack(1)
typedef struct
{
    uint8_t header1;        	/*0帧头1 0xEB*/
    uint8_t header2;        	/*1帧头2 0x90*/
    uint8_t cmd;					    /*2控制字*/
    uint8_t param1;					  /*3参数1 编码序号L*/
    uint8_t code_val_L;				/*4编码值L*/
    uint8_t code_val_M;				/*5编码值M*/
    uint8_t code_val_H;				/*6编码值H*/
    uint8_t param2;					  /*7参数2 编码序号H*/
    uint8_t sync_code;				/*8时统码*/
    uint8_t param3;					  /*9参数3 编码数量*/
    uint8_t param4;					  /*10参数4 编码组号*/
    uint8_t sum;					    /*11校验字 2-10字节累加*/
	
} LASER_TxFrame_t; /*发送帧结构体（发送至激光）*/
#pragma pack()
#define LASER_TX_FRAME_LEN  sizeof(LASER_TxFrame_t)

#pragma pack(1)
typedef struct
{
    uint8_t header1;        	/*0帧头1 0xEB*/
    uint8_t header2;        	/*1帧头2 0x90*/
    uint8_t status1;					/*2状态字1*/
    uint8_t status2;					/*3状态字2*/
    uint16_t first_dist;			/*4-5首距离 （LSB=1m）低字节在前*/
    uint8_t work_time[3];			/*6-8打光次数/工作时长*/
    int8_t temp;				      /*9温度 -128~127° 分辨率：1度*/
    uint8_t light_code;				/*10照射编码*/
    uint8_t apd_voltage;			/*11探测器高压 分辨率：10V*/
    uint8_t status3;					/*12状态字3*/
    uint8_t reserved1[2];			/*13-14备用*/
    uint8_t sync_code;				/*15时统码*/
    uint8_t reserved2[3];			/*16-18备用*/	
    uint16_t second_dist;			/*19-20次距离 （LSB=1m）*/
    uint16_t third_dist;			/*21-22末距离 （LSB=1m）*/
    uint8_t sum;					    /*23校验字 2-22字节累加*/	
} LASER_RxFrame_t; /*接收帧结构体（收激光数据）*/
#pragma pack()
#define LASER_RX_FRAME_LEN sizeof(LASER_RxFrame_t)   

extern LASER_RxFrame_t g_laser_rx;



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
void LASER_API_Period_Handle(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Laser_SendCommand(LASER_CMD_E cmd,uint8_t param1,uint8_t param2,uint8_t code_L,uint8_t code_M,uint8_t code_H,uint8_t sync,uint8_t cnt,uint8_t group);
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E master_cmd,uint16_t para1,uint16_t para2);

#endif
#endif
