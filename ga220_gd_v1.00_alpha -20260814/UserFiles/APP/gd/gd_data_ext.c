#include "gd/gd_data_ext.h"
#include "Common/config.h"
#include "Driver/drv_uart.h"

#if GD_EXT_INCLUDE && (ZH_PLANE_GD || JV_MINS_3C)
#if JV_MINS_3C
#define GD_DATA_IN_HEADER_1 0xEB
#define GD_DATA_IN_HEADER_2 0x90
#else
#define GD_DATA_IN_HEADER_1 0xAA
#define GD_DATA_IN_HEADER_2 0x55
#endif
static COM_RECV_INFO_T ext_gd_rcv_buf;
static GD_DATA_IN_EB_T gd_data_in;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void EXT_GD_API_Serial_Data_Init(void)
{
	memset(&ext_gd_rcv_buf,0x0,sizeof(ext_gd_rcv_buf));

	ext_gd_rcv_buf.header1 = GD_DATA_IN_HEADER_1;
	ext_gd_rcv_buf.tail1 = 0;
	ext_gd_rcv_buf.data_recv_len = 60;
	COM_Rcv_SerialPort_Init(COM_GD_EXT, ext_gd_rcv_buf.header1,ext_gd_rcv_buf.tail1,ext_gd_rcv_buf.data_recv_len);

}
#if ZH_PLANE_GD
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t year_buf = 0;
uint32_t month_buf = 0;
uint32_t day_buf = 0;
uint32_t gd_data_in_timeout = 0;//超时判断
void gd_process_data_in(uint8_t *data,uint32_t length)
{
	EXT_GD_DATA_T rcv_position;
//	PLANE_DATA_VALIDITY_ENUM gd_data_sta;
	if((data[0] == GD_DATA_IN_HEADER_1)&&(data[1] == GD_DATA_IN_HEADER_2))
	{
		//长度校验
		//将俯仰角、方向角、方向角速度、俯仰角速度、经纬高解算到同用参数里面进行管理
		memcpy(&gd_data_in,data,GD_IN_DATA_LEN);
		//gps周，周内秒转年月日时分秒
#if 0
		gps_week_to_date(gd_data_in.week,gd_data_in.week_sec,
		&year_buf,
		&month_buf,
		&day_buf,
		&rcv_position.hour,
		&rcv_position.min,
		&rcv_position.second);
#endif
		rcv_position.week = gd_data_in.week;
		rcv_position.week_sec = gd_data_in.week_sec;
		rcv_position.heading = gd_data_in.heading/10000.0;
		rcv_position.pitch = gd_data_in.pitch/10000.0;
		rcv_position.roll = gd_data_in.roll/10000.0;
		rcv_position.speed_east = gd_data_in.speed_e/10000.0;
		rcv_position.speed_north = gd_data_in.speed_n/10000.0;
		rcv_position.speed_sky = gd_data_in.speed_u/10000.0;
		rcv_position.longtitude = gd_data_in.longtitude/10000000.0;
		rcv_position.latitude = gd_data_in.latitude/10000000.0;
		rcv_position.alt = gd_data_in.alt/10000.0;
		rcv_position.gyro_speed_x = gd_data_in.speed_x/1000000.0;
		rcv_position.gyro_speed_y = gd_data_in.speed_y/1000000.0;
		rcv_position.gyro_speed_z = gd_data_in.speed_z/1000000.0;
		rcv_position.gyro_acc_x = gd_data_in.accelera_x/1000000.0;
		rcv_position.gyro_acc_y = gd_data_in.accelera_y/1000000.0;
		rcv_position.gyro_acc_z = gd_data_in.accelera_z/1000000.0;
		rcv_position.main_sat = gd_data_in.main_sat;
		rcv_position.sec_sat = gd_data_in.sec_sat;
		rcv_position.work_sta = gd_data_in.work_sta;
		memcpy(&rcv_position.gnss_sta,&gd_data_in.gnss_sta,sizeof(gd_data_in.gnss_sta));
		memcpy(&rcv_position.sys_err,&gd_data_in.err_sta,sizeof(gd_data_in.err_sta));
		//rcv_position.work_sta = gd_data_in.sync_sta;

		//外惯导数据有效性判断
#if 0
		if ((rcv_position.work_sta == NAVI_STA_INTERGRATE_NAVI) || (rcv_position.work_sta == 0x01))
		{
			gd_data_in_timeout = HAL_GetTick();
			//rcv_position.fj_gd_valid = PLANE_DATA_VALID;
			gd_data_sta = PLANE_DATA_VALID;
		}
#endif
//		CONFIG_Set_Plane_Data_Sta(gd_data_sta);
		CONFIG_Set_Ext_position(rcv_position);
	}
}
#endif

#if JV_MINS_3C
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void gd_process_data_in(uint8_t *data,uint32_t length)
{
	EXT_GD_DATA_T rcv_position;
	
	//长度校验
	//将俯仰角、方向角、方向角速度、俯仰角速度、经纬高解算到同用参数里面进行管理
	memcpy(&gd_data_in,data,GD_IN_DATA_LEN);

	rcv_position.datasta = gd_data_in.Datasta;
	rcv_position.heading = gd_data_in.FJheading_angle*0.00549333171;
	rcv_position.pitch = gd_data_in.FJpitch_angle*0.00274666585;
	rcv_position.roll = gd_data_in.FJHrolling_angle*0.00549333171;
	rcv_position.speed_east = gd_data_in.FJEast_spped*0.01562547685171;
	rcv_position.speed_north = gd_data_in.FJNorth_speed*0.01562547685171;	
	rcv_position.speed_sky = gd_data_in.FJSky_speed*0.01562547685171;
	rcv_position.longtitude = gd_data_in.FJlongitude*0.0000000838190318;
	rcv_position.latitude = gd_data_in.FJlatitude*0.0000000419095159;
	rcv_position.alt = gd_data_in.FJheight*0.001;	
	rcv_position.gyro_speed_x = gd_data_in.FJ_x_anspeed*0.0152592547380;		 	
	rcv_position.gyro_speed_y = gd_data_in.FJ_y_anspeed*0.0152592547380;			
	rcv_position.gyro_speed_z = gd_data_in.FJ_z_anspeed*0.0152592547380;			
	rcv_position.gyro_acc_x = gd_data_in.FJ_x_ac*0.0152592547380;	     	
	rcv_position.gyro_acc_y = gd_data_in.FJ_y_ac*0.0152592547380;	     	
	rcv_position.gyro_acc_z = gd_data_in.FJ_z_ac*0.0152592547380;
	rcv_position.year = gd_data_in.year;  
	rcv_position.month = gd_data_in.month; 
	rcv_position.day = gd_data_in.day;   
	rcv_position.hour = gd_data_in.hour;  
	rcv_position.min = gd_data_in.min;   
	rcv_position.second = gd_data_in.second;
	rcv_position.msecond = gd_data_in.msecond;
	
	rcv_position.gnss_longitude    = gd_data_in.GNSS_longitude*0.0000000838190318;   
	rcv_position.gnss_latitude     = gd_data_in.GNSS_latitude*0.0000000419095159;    
	rcv_position.gnss_height       = gd_data_in.GNSS_height*0.001;      		
	rcv_position.gnss_spped_east   = gd_data_in.GNSS_East_spped*0.01562547685171;  
	rcv_position.gnss_speed_north  = gd_data_in.GNSS_North_speed*0.01562547685171; 
	rcv_position.gnss_speed_sky    = gd_data_in.GNSS_Sky_speed*0.01562547685171;
	rcv_position.gnss_datasta      = gd_data_in.GNSS_DataSta;
	if ((rcv_position.datasta)&&(rcv_position.gnss_datasta))
	{
		CONFIG_Set_Plane_Data_Sta(1);//导航状态有效且数据有效才算有效
	}
	CONFIG_Set_Ext_position(rcv_position);
}

#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t EXT_GD_API_Period_Handle(void)
{
	uint16_t  recv_len = 0;
	recv_len = COM_REC_DataAnalysis_nocheck(COM_GD_EXT,ext_gd_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		#if 0
		//超过2s没有收到消息就自动将飞机信息置位无效
		if (HAL_GetTick() >= gd_data_in_timeout + 2000)
		{
			data_recv_sta = PLANE_DATA_INVALID;
			CONFIG_Set_Plane_Data_Sta(data_recv_sta);
		} 
		#endif
		return RECV_DATA_NULL;
	}
	else
	{
		gd_process_data_in(ext_gd_rcv_buf.recv_buf,recv_len);		
	}
}

#endif
