#ifndef __LASER_LRD_15H_H
#define __LASER_LRD_15H_H
#include "Common/base_inc.h"
#include "Common/config.h"

/*模块开关*/
#if LASER_15H

typedef enum
{
    LASER_DATA_STOP = 0x0,					/*测距停止*/
    LASER_PERIOD_SELF_CHECK = 0x1,			/*周期自检*/
    LASER_SINGLE_DETECT = 0x2,				/*单次测距*/
    LASER_DETECT_FREQ_1 = 0x3,				/*1hz测距*/
    LASER_DETECT_FREQ_5 = 0x4,				/*5hz测距*/
    LASER_DETECT_PRECISION = 0x5,			/*精频码照射*/
    LASER_DETECT_CHANGE_INTERVAL = 0x6,		/*变间隔码照射*/
    LASER_DETECT_PLUSE = 0x7,				/*脉冲序列码照射*/
    LASER_DEST_MODE_CHOOSE = 0x8,			/*目标模式选择*/
    LASER_RANGE_CFG = 0x9,					/*选通值设置*/
    LASER_LIGHT_TIME_REQ = 0xa,				/*出光次数查询*/
    LASER_LIGHT_EXT = 0xb,					/*外触发照射*/
    LASER_APD_POWER_ENABLE = 0xc,			/*APD电源开关*/
    LASER_CODE_ENABLE = 0x0D,				/*编码装订开关*/
    LASER_SELF_DESTROY = 0x0E,				/*编码自毁命令*/
    LASER_VERSION_CHECK = 0x10,				/*版本查询指令*/
    LASER_DETECT_DELAYSET = 0x11,			/*照射延时设置*/
    LASER_ENERGY_CFG = 0x12,				/*能量设置*/
    LASER_STATIC_DYNAMIC_CFG = 0x13,		/*动静态设置*/
    LASER_RESET = 0xFE,						/*恢复出厂设置*/
    LASER_UPGRADE = 0xFF,					/*程序升级*/

    /*二级指令交互*/
    LASER_CHANGE_INTERVAL_CFG = 0x30,		/*变间隔码设置*/
    LASER_CHANGE_INTERVAL_ACK = 0x31,		/*变间隔码查询回复*/

    LASER_PULSE_CFG = 0x32,		/*脉冲序列写入*/
    LASER_PULSE_ACK = 0x33,		/*脉冲序列写入回复*/

    LASER_PRECISSION_CFG = 0x34, /*精确频率码写入*/
    LASER_PRECISSION_ACK = 0x35, /*精确频率码写入回复*/

    LASER_CMD_END,/*枚举边界*/
} LASER_DATA_IN_TYPE_INFO;

#pragma pack(1)
typedef struct
{
    uint8_t Header;        	/*帧头*/
    uint8_t Ctrl_Cmd;		/*控制命令*/
	uint8_t len;			/*长度*/
    uint8_t CtrlData[10];	/*控制数据*/
    uint8_t Xor;			/*校验（异或）*/
} LASER_DATA_IN_EB_T; /*激光接收报文结构体*/
#pragma pack()
#define LASER_IN_DATA_LEN  sizeof(LASER_DATA_IN_EB_T)

#pragma pack(1)
typedef struct
{
    uint8_t apd_err:1;		/*apd错误*/
    uint8_t power_err:1;
    uint8_t dest_type:1;
    uint8_t apd_lock:1;
    uint8_t ldest_status:1;
    uint8_t echo_status:1;
    uint8_t fdest_status:1;
    uint8_t main_lobe:1;
} LASER_SELF_CHECK_1_T; /*激光自检状态*/
#pragma pack()

typedef struct
{
    uint8_t laser_status:2;
    uint8_t static_status:1;
    uint8_t light_status:1;
    uint8_t pd_err:1;
    uint8_t temp_err:1;
    uint8_t high_temp_alarm:1;
    uint8_t system_status:1;
} LASER_SELF_CHECK_2_T;

#pragma pack(1)
typedef struct
{
    LASER_SELF_CHECK_1_T laser_status1;
    LASER_SELF_CHECK_2_T laser_status2;
    uint16_t choose_range_value;
    uint8_t environ_temp;
    uint8_t ld_temp;
    uint16_t apd_vol;
    uint8_t power_rate;
    uint16_t light_delay;
} LASER_SELF_CHECK_T;
#pragma pack()

#pragma pack(1)
typedef struct
{
    LASER_SELF_CHECK_1_T laser_status1;
    LASER_SELF_CHECK_2_T laser_status2;
    uint16_t dest_distance;
    uint8_t environ_temp;
    uint8_t ld_temp;
    uint16_t light_cnt;
    uint8_t laser_power;
    uint8_t ld_pulse;
    uint8_t res;
} LASER_RECV_DETECT_T;
#pragma pack()

#pragma pack(1)
typedef struct
{
    uint8_t Header;        	/*帧头*/
    LASER_DATA_IN_TYPE_INFO Ctrl_Cmd;
    uint8_t CtrlData[5];	/*控制数据*/
    uint8_t Xor;			/*异或校验*/
} LASER_DATA_OUT_TO_LSP_T;
#pragma pack()
#define LASER_OUT_DATA_TO_LSP_LEN  sizeof(LASER_DATA_OUT_TO_LSP_T)

//二级指令发送结构体
#pragma pack(1)
typedef struct
{
    uint8_t Header;
    uint8_t Ctrl_Cmd;
    uint8_t code_class;
    uint16_t code_cnt;
    uint8_t send_page;
    uint8_t page_data_cnt;
    uint32_t code_number[44];
    uint8_t Xor;
} LASER_DATA_OUT_CODE_T;
#pragma pack()

//二级指令解算结构体
#pragma pack(1)
typedef struct
{
    uint8_t Header;
    uint8_t Ctrl_Cmd;
    uint8_t code_sta;
    uint16_t code_cnt;
    uint8_t base_freq;
    uint8_t max_data_cnt;
    uint8_t Xor;
} LASER_DATA_RECV_CODE_T;
#pragma pack()

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
uint8_t LASER_API_Period_Handle(void);
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
uint8_t Laser_Ctrl_Send_Lrd_Handle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
uint8_t Laser_Ctrl_Send_Ld_Handle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t laser_process_data_in(uint8_t *data,uint32_t length);
#endif
#endif
