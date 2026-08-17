#ifndef __GEO_FILE_H
#define __GEO_FILE_H
#include "Common/base_inc.h"
//------------函数定义------------//
void GEO(void);
void Target_Orient_Fun(void);
void Target_Vel_Estimate(void);
void LMC_Vel_Cal(void);
void GEO_para_init(void);
float GEO_Target_Get_Dis(void);

//用户数据结构体
__packed typedef struct
{
#if (GEO_TRA_SOUR == SOURCE_PLANEGD)|| (TARGET_POS_SOUR == SOURCE_PLANEGD) || LMC_STA
	__packed struct
	{
#if (GEO_TRA_SOUR == SOURCE_PLANEGD)|| (TARGET_POS_SOUR == SOURCE_PLANEGD)
		float longtitude;	//经度 单位：°
		float latitude;	//纬度 单位：°
		float alt;	//海拔高度 单位：m
		float yaw;	//航向 单位：°
		float pitch;	//俯仰 单位：° 抬头为正，低头为负
		float roll;	//滚转 单位：° 右低为正，左低为负
#endif
		
#if LMC_STA
		float speed_east;	//东向速度 单位：°/s
		float speed_north;	//东向速度 单位：°/s
		float speed_sky;	//东向速度 单位：°/s
#endif
	}plane_gd;	//外惯导数据
#endif

#if (GEO_TRA_SOUR == SOURCE_INNERGD)|| (TARGET_POS_SOUR == SOURCE_INNERGD)
	__packed struct
	{
		float longtitude;	//经度 单位：°
		float latitude;	//纬度 单位：°
		float alt;	//海拔高度 单位：m
		float yaw;	//航向 单位：°
		float pitch;	//俯仰 单位：° 抬头为正，低头为负
		float roll;	//滚转 单位：° 右低为正，左低为负
	}inner_gd;	//内惯导数据
#endif
	
	__packed struct
	{
		float fw;	//方位角 单位：°
		float fy;	//俯仰角 单位：° 抬头为正，低头为负
		float hg;	//横滚角 单位：° 右低为正，左低为负
	}pt_angle;	//平台框架角数据

#if TARGET_POS
	__packed struct
	{
		float longtitude;	//经度 单位：°
		float latitude;	//纬度 单位：°
		float alt;	//海拔高度 单位：m
	}target_tra;	//地理跟踪目标数据
#endif

#if LMC_STA || TARGET_POS
	float ground_abo;	//对地高度 单位：m
#endif
	
#if TARGET_POS
	float laser_sta;	//	1-正在测距	0-待机/故障
	float laserdistance;	//激光测距值 单位：m
#endif
}GEO_USER_IN_DATA_T;

__packed typedef struct
{
#if GEO_TRA
	__packed struct
	{
		float fw;	//方位角 单位：°
		float fy;	//俯仰角 单位：° 抬头为正，低头为负
		float hg;	//横滚角 单位：° 右低为正，左低为负
	}pt_angle;
	/*使用内惯导时，伺服将其当做角度误差使用
		使用外惯导时，伺服将其作为框架目标角
	*/
#endif
	
#if LMC_STA
	__packed struct
	{
		float fw;	//方位角 单位：°/s
		float fy;	//俯仰角 单位：°/s 抬头为正，低头为负
		float hg;	//横滚角 单位：°/s 右低为正，左低为负
	}lmc_speed;
#endif

#if TARGET_POS
	__packed struct
	{
		float longtitude;	//经度 单位：°
		float latitude;	//纬度 单位：°
		float alt;	//海拔高度 单位：m
	}target_pos;	//单目标定位数据
#endif

	float eo_center_dis;	//视轴中心目标距离
	
	float tra_target_dis;	//地理跟踪目标距离
	
}GEO_USER_OUT_DATA_T;
#endif
