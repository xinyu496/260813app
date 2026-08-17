#include "utl/utl_math.h"
#include "config/config.h"
#include "gd/gd_data_inter.h"
#include "Driver/drv_uart.h"

#if GD_INSIDE_INCLUDE
#define INSIDE_GD_DATA_HEADER_1 0xEB
#define INSIDE_GD_DATA_HEADER_2 0x90

#define INSIDE_GD_DATA_CMD_FLAG 0x65
static COM_RECV_INFO_T gd_inter_rcv_buf;
/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:初始化数据接收的结构体
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
void Inside_GD_API_Serial_Data_Init(void)
{
	gd_inter_rcv_buf.header1 = INSIDE_GD_DATA_HEADER_1;
	gd_inter_rcv_buf.tail1 = 0;
	gd_inter_rcv_buf.data_recv_len = 60;
	COM_Rcv_SerialPort_Init(COM_GD_IN, gd_inter_rcv_buf.header1,gd_inter_rcv_buf.tail1,gd_inter_rcv_buf.data_recv_len);
}

#if (FM503G_0A || JV_MINS_6G)
/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:内惯导对准流程
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
#if 0
uint32_t AimStart_Time = 0, AimForce_Time = 0;
uint8_t sf_lock = 0;
uint8_t GD_Inner_Calibrate(void)
{
	static AIM_CALIBRATE_STEP_ENUM AimInit_Step;

//	SF_SYS_MODE_ENUM sf_mode;
//	uint8_t track_mode = 0;
	SYS_SF_DATA_T sf_info;
	uint8_t inner_gd_type;
	INSIDE_GD_DATA_T inner_gd = {0};

	if (HAL_GetTick() <= 5000)
	{
		return inner_gd_type;
	}

	switch (AimInit_Step)
	{
	case AIM_CALIBRATE_WAIT:
		/*飞机发进来的导航数据有效，且收到对准指令，且设备已经商店了5000ms即5s以上。则执行对准*/
		if ((CONFIG_Get_Plane_Data_Sta()) && (AimInit_Step == AIM_CALIBRATE_WAIT))
		{
			AimInit_Step = AIM_CALIBRATE_STEP1;
		}
		break;
	case AIM_CALIBRATE_STEP1:
		/*设置伺服归零(手动自检)*/
//		sf_mode = P_Zero;
		/*跟踪器退出功能*/
//		track_mode = 0;
		AimInit_Step = AIM_CALIBRATE_STEP2;
		AimForce_Time = HAL_GetTick();
		AimStart_Time = HAL_GetTick(); 
		break;

	case AIM_CALIBRATE_STEP2:
		if (fabs(sf_info.FY_Combine) <= 0.2f) /*方位和俯仰误差小于 ±0.2，则进入对准指令*/
		{
			if ((fabs(sf_info.FW_Combine) <= 0.2f) || (sf_info.FW_Combine >= 359.8f))
			{
				inner_gd_type = 0xCA; /*对准指令*/
				AimStart_Time = HAL_GetTick();
				AimInit_Step = AIM_CALIBRATE_STEP3;
			}
		}
		else if (HAL_GetTick() >= AimForce_Time + 15000) /*如果前面大于0.2的误差，15s之后也强制对准*/
		{
			inner_gd_type = 0xCA; /*对准指令*/
			AimStart_Time = HAL_GetTick();
			AimInit_Step = AIM_CALIBRATE_STEP3;
		}
		break;

	case AIM_CALIBRATE_STEP3: /*对准指令发了之后，等待返回*/
		inner_gd_type = 0x00;
		if (inner_gd.Sys_sta.ref_sta == 1) /*正在导航*/
		{
			AimInit_Step = AIM_CALIBRATE_END;
		}
		else /* 初始对准，即使内有进入对准标志位，
				但是由于进入静态对准或者动态对准，系统时间超时了，依旧退出对准*/
		{
			if (inner_gd.Aim_Sta == 1) /*静态对准*/
			{
				if (HAL_GetTick() >= AimStart_Time + 40000) /*静态超时40s*/
					AimInit_Step = AIM_CALIBRATE_END;
			}
			else if (inner_gd.Aim_Sta == 2) /*动态对准*/
			{
				if (HAL_GetTick() >= AimStart_Time + 100000) /*动态超时100s*/
					AimInit_Step = AIM_CALIBRATE_END;
			}
		}
		break;

	default:

		break;
	}

	if ((HAL_GetTick() >= AimStart_Time + 105000) && (AimStart_Time != AIM_CALIBRATE_WAIT) && (((AimInit_Step == AIM_CALIBRATE_STEP2) || (AimInit_Step == AIM_CALIBRATE_STEP3)))) /*对准超时105s*/
	{
		AimInit_Step = AIM_CALIBRATE_END;
	}

	if (AimInit_Step == AIM_CALIBRATE_END)
	{
		sf_lock = 1;
	}
	/*对准结束之后，sf进入锁定模式*/
	if ((sf_lock) && (HAL_GetTick() >= 10000))
	{
		sf_lock = 0;
//		sf_mode = P_GEO;
	}
	return inner_gd_type;
}
#endif
/*==============================================================
 *FUNCTION NAME:内惯导数据帧组帧发送
 *DISCRIPTION:
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
uint8_t INSIDE_GD_Process_Data_Send(uint8_t ctrl_cmd)
{
	INSIDE_GD_MASTER_DATA_SEND_T send_to_imu_gd = {0};
	SYS_SF_DATA_T sf_info = {0};
	EXT_GD_DATA_T ext_gd = {0};//飞机惯导或者内置外惯导
	uint32_t *arm_info;

	sf_info = CONFIG_Get_Sf_Info();
	//memcpy(arm_info, CONFIG_Get_Arm_info(),12);
	arm_info = CONFIG_Get_Arm_info();

	send_to_imu_gd.Header1 = INSIDE_GD_DATA_HEADER_1;
	send_to_imu_gd.Header2 = INSIDE_GD_DATA_HEADER_2;
	send_to_imu_gd.cmd_flag = INSIDE_GD_DATA_CMD_FLAG;
	send_to_imu_gd.calibrate_cmd = ctrl_cmd;// 指令类型

	sf_info.FW_Combine  = UTL_Normalize_Angle(sf_info.FW_Combine);
	send_to_imu_gd.pod_ori = (int16_t)(sf_info.FW_Combine / 0.00549333f);
	send_to_imu_gd.pod_pitch = (int16_t)(sf_info.FY_Combine / 0.00549333f);
	send_to_imu_gd.pod_roll = 0;

	
	if (CONFIG_Get_Plane_Data_Sta())
	{
		send_to_imu_gd.gnss_hdop = 100;
		send_to_imu_gd.gnss_num = 10;
		send_to_imu_gd.gnss_Sta = 1;
	}
	else
	{
		send_to_imu_gd.gnss_hdop = 0;
		send_to_imu_gd.gnss_Sta = 0;
		send_to_imu_gd.gnss_num = 0;
	}
	send_to_imu_gd.gd_heading = ext_gd.heading * 100;
	send_to_imu_gd.gd_pitch = ext_gd.pitch * 100;
	send_to_imu_gd.gd_roll = ext_gd.roll * 100;
	send_to_imu_gd.gnss_longi = ext_gd.longtitude * 10000000;
	send_to_imu_gd.gnss_lat = ext_gd.latitude * 10000000;
	send_to_imu_gd.gnss_alt = ext_gd.alt * 1000;
	send_to_imu_gd.gnss_spd_e = ext_gd.speed_east * 100;
	send_to_imu_gd.gnss_spd_n = ext_gd.speed_north * 100;
	send_to_imu_gd.gnss_spd_t = ext_gd.speed_sky * 100;
	send_to_imu_gd.pod_data_valid = 1;
	send_to_imu_gd.gd_data_valid = CONFIG_Get_Plane_Data_Sta();
	/*杆臂值在初始化的时候，从内部读取*/
	send_to_imu_gd.arm1_x = arm_info[0];
	send_to_imu_gd.arm1_y = arm_info[1];
	send_to_imu_gd.arm1_z = arm_info[2];
	send_to_imu_gd.arm2_x = 0;
	send_to_imu_gd.arm2_y = 0;
	send_to_imu_gd.arm2_z = 0;
	/*上级系统下发的时间：优先取用外惯导时间*/
	send_to_imu_gd.utc_year = 0;
	send_to_imu_gd.utc_month = 0;
	send_to_imu_gd.utc_day = 0;
	send_to_imu_gd.utc_hour = 0;
	send_to_imu_gd.utc_min = 0;
	send_to_imu_gd.utc_sec = 0;
	send_to_imu_gd.utc_ms = 0;

	send_to_imu_gd.add_check = 0;

	COM_API_Send_Data(COM_GD_IN, (uint8_t *)&send_to_imu_gd, sizeof(send_to_imu_gd));

	return RECV_DATA_SUC;
}

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:姿态通信协议
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
GD_INTERNAL_RECV_DATA_T master_rcv_inside_info;
void Inside_gd_process_position_data(uint8_t *data, uint32_t length)
{
	/*解算内部惯导的俯仰角经纬高等数据*/
	INSIDE_GD_DATA_T inside_gd_pos;
	/*将俯仰角、方向角、方向角速度、俯仰角速度、经纬高解算到通用参数库里面进行管理*/
	memcpy(&master_rcv_inside_info, data, 41);
	inside_gd_pos.produc_logo = master_rcv_inside_info.produc_logo;
	inside_gd_pos.pitch = master_rcv_inside_info.pitch * 0.01;
	inside_gd_pos.roll = master_rcv_inside_info.roll * 0.01;
	inside_gd_pos.heading = master_rcv_inside_info.heading * 0.01;
	inside_gd_pos.Temperature = master_rcv_inside_info.in_gd_temp * 0.01;
	inside_gd_pos.longtitude = (master_rcv_inside_info.in_gd_longitude * (180 / 2147483648.0)); /*2^31 = 2147483648.0*/
	inside_gd_pos.latitude = (master_rcv_inside_info.in_gd_latitude * (180 / 2147483648.0));
	inside_gd_pos.alt = master_rcv_inside_info.in_gd_alt * 0.001;
	inside_gd_pos.send_cnt = master_rcv_inside_info.send_cnt;
	inside_gd_pos.main_version = master_rcv_inside_info.main_version;
	inside_gd_pos.sec_version = master_rcv_inside_info.sec_version;
	*(uint8_t *)&inside_gd_pos.Sys_sta = *(uint8_t *)&master_rcv_inside_info.sys_status;
	inside_gd_pos.Aim_Sta = master_rcv_inside_info.calibrate_sta;

	CONFIG_Set_Internal_position(inside_gd_pos);
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
RECV_DATA_ERR_STA Inside_GD_API_Period_Handle(void)
{
	/*内惯导数据解析*/
    uint8_t recv_len = 0;
	recv_len = COM_REC_DataAnalysis_nocheck(COM_GD_IN,gd_inter_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
	Inside_gd_process_position_data(gd_inter_rcv_buf.recv_buf, recv_len);
}
#endif
