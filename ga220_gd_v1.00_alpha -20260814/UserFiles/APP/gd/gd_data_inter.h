#ifndef __GD_DATA_RCV_H
#define __GD_DATA_RCV_H
#include "base/base_inc.h"
#include "config/config.h"

#if GD_INSIDE_INCLUDE

#define INTER_GD_HEADER_1 0xeb
#define INTER_GD_HEADER_2 0x90
#define INTER_GD_CMD_ID_TLY 0x24
#define INTER_GD_CMD_ID_POS_RECV 0x65

#pragma pack(1)
/*
描述：内惯导接收数据帧
长度：60字节
*/
typedef struct
{
	uint8_t Header1;
	uint8_t Header2;
	uint8_t cmd_flag;
	uint8_t calibrate_cmd;
	int16_t pod_ori;
	int16_t pod_pitch;
	int16_t pod_roll;
	int16_t gd_heading;
	int16_t gd_pitch;
	int16_t gd_roll;
	int32_t gnss_longi;
	int32_t gnss_lat;
	int32_t gnss_alt;
	int16_t gnss_spd_e;
	int16_t gnss_spd_n;
	int16_t gnss_spd_t;
	uint8_t gnss_num;
	uint8_t gnss_Sta;
	int16_t gnss_hdop;
	uint8_t pod_data_valid;
	uint8_t gd_data_valid;
	int8_t arm1_x;
	int8_t arm1_y;
	int8_t arm1_z;
	int8_t arm2_x;
	int8_t arm2_y;
	int8_t arm2_z;
	uint16_t utc_year;
	uint8_t utc_month;
	uint8_t utc_day;
	uint8_t utc_hour;
	uint8_t utc_min;
	uint8_t utc_sec;
	uint16_t utc_ms;
	uint8_t res[3];
	uint16_t add_check;
} INSIDE_GD_MASTER_DATA_SEND_T;
extern INSIDE_GD_MASTER_DATA_SEND_T InnerGDTx_Data;
#define INSIDE_GD_MASTER_SEND_DATA_LEN sizeof(INSIDE_GD_MASTER_DATA_SEND_T)
//#endif
#if JV_MINS_6G
/*
描述：内惯导发送数据帧
长度：41字节
*/
typedef struct
{
	uint8_t Header1;
	uint8_t Header2;
	uint8_t cmd_flag;
	uint8_t produc_logo;
	int16_t pitch;
	int16_t roll;
	uint16_t heading;
	int16_t in_gd_temp;
	int32_t in_gd_longitude;
	int32_t in_gd_latitude;
	int32_t in_gd_alt;
	uint32_t send_cnt;
	uint32_t product_num;
	uint8_t main_version;
	uint8_t sec_version;
	uint8_t sys_status;
	uint8_t calibrate_sta;
	uint8_t res[3];
	uint16_t add_check;
} GD_INTERNAL_RECV_DATA_T;
#endif
#pragma pack()
#define GD_INTERNAL_RECV_DATA_LEN sizeof(GD_INTERNAL_RECV_DATA_T)
#endif

/*内惯导对准状态机*/
typedef enum
{
	AIM_CALIBRATE_WAIT,
	AIM_CALIBRATE_STEP1,
	AIM_CALIBRATE_STEP2,
	AIM_CALIBRATE_STEP3,
	AIM_CALIBRATE_END,
} AIM_CALIBRATE_STEP_ENUM;

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:数据接收初始化
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
void Inside_GD_API_Serial_Data_Init(void);

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:周期性数据解析
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
RECV_DATA_ERR_STA Inside_GD_API_Period_Handle(void);

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:数据发送
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
uint8_t INSIDE_GD_Process_Data_Send(uint8_t ctrl_cmd);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t GD_Inner_Calibrate(void);
#endif

