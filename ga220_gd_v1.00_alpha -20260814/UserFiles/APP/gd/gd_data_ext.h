#ifndef __GD_DATA_EXT_H
#define __GD_DATA_EXT_H
#include "Common/base_inc.h"

#if GD_EXT_INCLUDE

#if ZH_PLANE_GD
#pragma pack(1)
typedef struct
{
	uint8_t Header1;
	uint8_t Header2;
	uint8_t class;
	uint8_t id;
	uint16_t data_len;
	uint16_t data_cnt;
	uint16_t week;
	double week_sec;
	int32_t heading;
	int32_t pitch;
	int32_t roll;
	int32_t speed_e;
	int32_t speed_n;
	int32_t speed_u;
	int32_t longtitude;
	int32_t latitude;
	int32_t alt;
	int32_t speed_x;
	int32_t speed_y;
	int32_t speed_z;
	int32_t accelera_x;
	int32_t accelera_y;
	int32_t accelera_z;
	uint8_t main_sat;
	uint8_t sec_sat;
	uint8_t work_sta; // 0x00 - 待机 0x10 - 粗对准 0x20 - 精对准 0x30 - 组合导航 0x31 - 纯惯性
	struct
	{
		uint16_t ref_sta : 3;
		uint16_t position_valid : 1;
		uint16_t ori_valid : 1;
		uint16_t gps_data : 1;
		uint16_t rev : 10;
	} gnss_sta;
	/*
Bit2-Bit0：
=0：无效
=1：单点定位
=2：伪距差分
=3：RTK差分定位
Bit3：位置速度数据有效性 0：无效
Bit4：GNSS双天线航向有效性 0：无效
Bit5：GPS时数据有效 0：无效
Bit6-Bit15：预留为0
*/
	struct
	{
		uint16_t imu_x_err : 1;
		uint16_t imu_y_err : 1;
		uint16_t imu_z_err : 1;
		uint16_t acc_x_err : 1;
		uint16_t acc_y_err : 1;
		uint16_t acc_z_err : 1;
		uint16_t gnss_hw : 1;
		uint16_t rev : 9;
	} err_sta;
	/*
Bit0：X轴陀螺故障字 0：正常
Bit1：Y轴陀螺故障字 0：正常
Bit2：Z轴陀螺故障字 0：正常
Bit3：X轴加速度故障字 0：正常
Bit4：Y轴加速度故障字 0：正常
Bit5：Z轴加速度故障字 0：正常
Bit6：GNSS板卡硬件故障字0：正常
Bit7-Bit15：预留为0
*/
	uint8_t res[8];
	uint16_t add_check;
} GD_DATA_IN_EB_T;
#pragma pack()
#define GD_IN_DATA_LEN sizeof(GD_DATA_IN_EB_T)

#endif

#if JV_MINS_3C
#pragma pack(1)
typedef struct
{
	// 收到的原始数据
	uint8_t Head1;			  // 0
	uint8_t Head2;			  // 1
	uint16_t lenth;			  // 2~3 帧长度
	uint16_t ID;			  // 4~5 消息代码
	uint8_t Datasta;		  // 6   数据有效性	/*0：组合导航无效 1：组合导航有效*/
	int32_t FJlongitude;	  // 7~10 FJ经度
	int32_t FJlatitude;		  // 11~14 FJ维度
	int32_t FJheight;		  // 15~18 FJ海拔高度
	int16_t FJheading_angle;  // 19~20 FJ航向角
	int16_t FJpitch_angle;	  // 21~22 FJ俯仰角
	int16_t FJHrolling_angle; // 23~24 FJ横滚角
	int16_t FJEast_spped;	  // 25~26 FJ东向速度
	int16_t FJNorth_speed;	  // 27~28 FJ北向速度
	int16_t FJSky_speed;	  // 29~30 FJ天向速度
	int16_t FJ_x_anspeed;	  // 31~32 机体X轴角速度
	int16_t FJ_y_anspeed;	  // 33~34 机体Y轴角速度
	int16_t FJ_z_anspeed;	  // 35~36 机体Z轴角速度
	int16_t FJ_x_ac;		  // 37~38 机体X轴加速度
	int16_t FJ_y_ac;		  // 39~40 机体Y轴加速度
	int16_t FJ_z_ac;		  // 41~42 机体Z轴加速度
	uint16_t year;			  // 43~44 年
	uint8_t month;			  // 45    月
	uint8_t day;			  // 46    日
	uint8_t hour;			  // 47    时
	uint8_t min;			  // 48    分
	uint8_t second;			  // 49    秒
	uint16_t msecond;		  // 50~51 毫秒
	int32_t GNSS_longitude;	  // 52~55 GNSS经度
	int32_t GNSS_latitude;	  // 56~59 GNSS纬度
	int32_t GNSS_height;	  // 60~63 GNSS高度
	int16_t GNSS_East_spped;  // 64~65 GNSS东向速度
	int16_t GNSS_North_speed; // 66~67 GNSS北向速度
	int16_t GNSS_Sky_speed;	  // 68~69 GNSS天向速度
	uint8_t GNSS_DataSta;	  // 70 GNSS数据有效状态	/*0：无效 1：有效*/
	uint8_t NumberofFJ[8];	  // 71~78 飞机编号
	uint8_t GPS_Num;
	uint8_t Rev[7];
	uint8_t Sum; // 86  校验码
	uint8_t End; // 87  帧尾
} GD_DATA_IN_EB_T;
#pragma pack()
extern GD_DATA_IN_EB_T RxGDData;
#define GD_IN_DATA_LEN sizeof(GD_DATA_IN_EB_T)

#endif

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:初始化数据接收的结构体
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
void EXT_GD_API_Serial_Data_Init(void);
/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:接收后的数据处理
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
uint8_t EXT_GD_API_Period_Handle(void);
#endif
#endif
