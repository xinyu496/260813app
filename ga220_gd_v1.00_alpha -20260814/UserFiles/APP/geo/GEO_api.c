#include "Common/base_inc.h"
#include "Bsp/bsp_timer.h"
#include "../geo/GEO_api.h"
#include "Common/config.h"
#include "GEO_File.h"
#include "Common/utl_math.h"
#include "GEO_Track_C.h"

extern USER_CNT_T User_Tick;
extern float LaserDistance;
//计算结果
GEO_TRACK_DATA_T aim_info = {0};
void USER_Ctrl_GEO_process(void)
{
	GEO_USER_IN_DATA_T geo_datain = {0};
	GEO_USER_OUT_DATA_T geo_dataout = {0};
	if ( User_Tick.GEO_tick >= 40) /*20ms间隔*/
	{
		User_Tick.GEO_tick = 0;
		/*********************参数获取Start******************/
		//geo_datain = CONFIG_Get_GEO_data_in();//获取最新的外惯导信息
		
		/*PT_Angle为吊舱框架角*/
		PT_Angle.Fw = geo_datain.pt_angle.fw;
		PT_Angle.Fy = geo_datain.pt_angle.fy;
		PT_Angle.Hg = geo_datain.pt_angle.hg;
		
		
#if (GEO_TRA_SOUR == SOURCE_PLANEGD)|| (TARGET_POS_SOUR == SOURCE_PLANEGD)
		/*INS为FJ惯导数据*/
		INS.Pitch = geo_datain.plane_gd.pitch;
		INS.Roll = geo_datain.plane_gd.roll;
		INS.Yaw = geo_datain.plane_gd.yaw;
		INS.Yaw = UTL_Normalize_Angle(geo_datain.plane_gd.yaw);
		INS.longitude = deg2rad(geo_datain.plane_gd.longtitude);
		INS.lattitude = deg2rad(geo_datain.plane_gd.latitude);
		INS.height = geo_datain.plane_gd.alt;
#endif

#if (GEO_TRA_SOUR == SOURCE_INNERGD)|| (TARGET_POS_SOUR == SOURCE_INNERGD)
		/*INS_Inner为内惯导数据*/
		INS_Inner.Pitch = geo_datain.inner_gd.pitch;
		INS_Inner.Roll = geo_datain.inner_gd.roll;
		INS_Inner.Yaw = UTL_Normalize_Angle(geo_datain.inner_gd.yaw);
		INS_Inner.longitude = deg2rad(geo_datain.inner_gd.longtitude);
		INS_Inner.lattitude = deg2rad(geo_datain.inner_gd.latitude);
		INS_Inner.height = geo_datain.inner_gd.alt;
#endif

#if LMC_STA
		INS.Vel_E = geo_datain.plane_gd.speed_east;
		INS.Vel_N = geo_datain.plane_gd.speed_north;
		INS.Vel_S = geo_datain.plane_gd.speed_sky;
#endif
	
#if GEO_TRA
		/*Target为地理跟踪目标经纬高：外部输入的目标点经纬高*/
		Target.longitude =  deg2rad((double)geo_datain.target_tra.longtitude);
		Target.lattitude = deg2rad((double)geo_datain.target_tra.latitude);
		Target.height = geo_datain.target_tra.alt;
#endif

#if TARGET_POS
static uint8_t Dis_effect = 0;
		if(geo_datain.laser_sta)
		{
			LaserDistance = geo_datain.laserdistance;
			Dis_effect = 1;
		}
		else
		{
#if TARGET_POS_SOUR == SOURCE_PLANEGD
			LaserDistance = geo_dataout.eo_center_dis;
			Dis_effect = 1;
#endif
		}
#endif
		/*********************参数获取End******************/
		
/***********************地理跟踪功能Start***********************/
#if GEO_TRA
		if (CONFIG_Get_SF_Mode() == SF_TRACK_GEO)
		{
			GEO();
#if GEO_TRA_SOUR == SOURCE_PLANEGD
			geo_dataout.pt_angle.fw = OrientLoad_PT.Orient_Load_FW;
			geo_dataout.pt_angle.fy = OrientLoad_PT.Orient_Load_FY;
			geo_dataout.pt_angle.hg = 0;
#endif
#if GEO_TRA_SOUR == SOURCE_INNERGD
			geo_dataout.pt_angle.fw = OrientLoad_Inner.Orient_Load_FW - INS_Inner.Yaw;
			geo_dataout.pt_angle.fy = OrientLoad_Inner.Orient_Load_FY - INS_Inner.Pitch;
			geo_dataout.pt_angle.hg = 0;
#endif
		}
#endif
/***********************地理跟踪功能End***********************/

/**************************LMC Start**************************/
#if GEO_TRA
		if (CONFIG_Get_SF_Mode() == SF_SPEED_FOLLOW)
		{
			if (CONFIG_Get_LMC_Sta() == 0x01) // LMC开
			{
				/* TODO 参数没传 */
				LMC_Vel_Cal();
				if (PT_Angle.Fy < -75.0)
				{
					LMC.Fw = 0;
				}
			}
			else
			{
				LMC.Fw = 0;
				LMC.Fy = 0;
			}
			geo_dataout.lmc_speed.fw = LMC.Fw;
			geo_dataout.lmc_speed.fy = LMC.Fy;
		}
#endif
		if ((CONFIG_Get_SF_Mode() != SF_TRACK_GEO) ||(CONFIG_Get_SF_Mode() != SF_SPEED_FOLLOW))
		{
			GPS_Orient_Result.height = 0;
			GPS_Orient_Result.lattitude = 0;
			GPS_Orient_Result.longitude = 0;
			Target_Motion.Vel = 0;
			Target_Motion.Yaw = 0;
			
			OrientLoad_PT.Orient_Load_FW = 0;
			OrientLoad_PT.Orient_Load_FY = 0;
			OrientLoad_Inner.Orient_Load_FW = 0;
			OrientLoad_Inner.Orient_Load_FY = 0;
			
			LMC.Fw = 0;
			LMC.Fy = 0;
		}
/**************************LMC End**************************/
		
/***********************单目标定位Start*********************/
#if TARGET_POS
	/*激光正常工作的时候进行计算*/
		if(Dis_effect == 1)
		{
#if TARGET_POS_SOUR == SOURCE_PLANEGD
			/***********基于外惯导**********/
			Target_Orient_Fun();
			GPS_Orient_Result = Target_GPS_Cal(INS, LaserDistance, Orient);
#endif
#if TARGET_POS_SOUR == SOURCE_INNERGD
			/***********基于内惯导**********/
			Orient.Orient_Yaw = INS_Inner.Yaw;
			Orient.Orient_Pitch = INS_Inner.Pitch;
			Orient.Orient_Roll = INS_Inner.Roll;
			GPS_Orient_Result = Target_GPS_Cal(INS_Inner, LaserDistance, Orient);
#endif

			geo_dataout.target_pos.longtitude = rad2deg(GPS_Orient_Result.longitude);
			geo_dataout.target_pos.latitude = rad2deg(GPS_Orient_Result.lattitude);
			geo_dataout.target_pos.alt = GPS_Orient_Result.height;
		}
#endif
/*******************单目标定位End***********************/

/*******************目标距离计算Start*******************/
		geo_dataout.tra_target_dis = TGT3();
		
#if SOURCE_INNERGD
		if(INS_Inner.Pitch < 0)
		{
			geo_dataout.eo_center_dis = geo_datain.ground_abo / fabs(sin(INS_Inner.Pitch *  0.0174532925199));
		}
#endif
/*******************目标距离计算Start*******************/
	}
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:两个地理位置之间的斜距计算
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#define a 6378137.000
#define dEcc1 0.006694380066764658
float TGT3(void)
{
	float TargeTDis = 0.0;

	float dX_G[2], dY_G[2], dZ_G[2];
	float dNr[2];

	float Sin_PT_long = 0.0;
	float Cos_PT_long = 1.0;
	float Sin_PT_lat = 0.0;
	float Cos_PT_lat = 1.0;

	float Sin_Target_long = 0.0;
	float Cos_Target_long = 1.0;
	float Sin_Target_lat = 0.0;
	float Cos_Target_lat = 1.0;

	Sin_Target_long = sin(Target.longitude);
	Cos_Target_long = cos(Target.longitude);
	Sin_Target_lat = sin(Target.lattitude);
	Cos_Target_lat = cos(Target.lattitude);

	Sin_PT_long = sin(INS.longitude);
	Cos_PT_long = cos(INS.longitude);
	Sin_PT_lat = sin(INS.lattitude);
	Cos_PT_lat = cos(INS.lattitude);

	dNr[0] = a / (sqrt(1 - dEcc1 * pow(Sin_PT_lat, 2)));
	dX_G[0] = (dNr[0] + INS.height) * Cos_PT_lat * Cos_PT_long;
	dY_G[0] = (dNr[0] + INS.height) * Cos_PT_lat * Sin_PT_long;
	dZ_G[0] = (dNr[0] * (1 - dEcc1) + INS.height) * Sin_PT_lat;

	dNr[1] = a / (sqrt(1 - dEcc1 * pow(Sin_Target_lat, 2)));
	dX_G[1] = (dNr[1] + Target.height) * Cos_Target_lat * Cos_Target_long;
	dY_G[1] = (dNr[1] + Target.height) * Cos_Target_lat * Sin_Target_long;
	dZ_G[1] = (dNr[1] * (1 - dEcc1) + Target.height) * Sin_Target_lat;

	TargeTDis = sqrt(pow((dX_G[0] - dX_G[1]), 2) + pow((dY_G[0] - dY_G[1]), 2) + pow((dZ_G[0] - dZ_G[1]), 2));
	return TargeTDis;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取地理定位的计算结果
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
GEO_TRACK_DATA_T GEO_Get_Aim_Info(void)
{
    return aim_info;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:输入目标定位的经纬高
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void GEO_Set_Aim_Info(float *input_target)
{
	aim_info.TargetLon = input_target[0];
	aim_info.TargetLat = input_target[1];
	aim_info.TargetAlt = input_target[2];
}

void CONFIG_Get_GEO_data_in()
{
	
}
