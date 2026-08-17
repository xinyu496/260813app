#include "Driver/drv_uart.h"

#if ETH_INCLUDE
#include "Driver/drv_udp.h"
#endif

#if VIDEO_TRACK_INCLUDE
#include "../cam_track/Track.h"
#include "Common/utl_math.h"
#include "Common/utl_check.h"
static COM_RECV_INFO_T track_rcv_info;
#define CAM_DATA_INFO_HEADER_1  0xEB
#define CAM_DATA_INFO_HEADER_2  0x90
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:跟踪器状态上报
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Track_Cam_Send_data(uint8_t send_cmd,uint8_t *send_data,uint8_t lengh)
{
	CAM_VIDEO_STATUS_DATA_T track_send_info;
	MASTER_CONFIG_INFO_T main_info;
	ATTITUDE_PAPA_T sf_angle;
	SYS_LASER_STA_T laser_info;
	POSITION_INFO_T turn_data_dms;
//	uint8_t sum_check = 0;
	//发送给跟踪器的状态信息
	
#if 0
	aim_info = CONFIG_Get_GEO_Aim_Info();

	track_send_info.header1 = CAM_DATA_INFO_HEADER_2;
	track_send_info.header2 = CAM_DATA_INFO_HEADER_1;
	track_send_info.data_len = send_cmd;
	track_send_info.PositionCoor = 0;//User_Cmd.TraCtrl_Para1;
	track_send_info.PitchCoor = 0;//User_Cmd.TraCtrl_Para2;
	track_send_info.Para = send_data[0];
	track_send_info.Osd.SysMode = 0;//EO_state.sf_info.SFstate.SFsta;
	track_send_info.Osd.EO_FWAngle = sf_angle.azimuth_para * 100;//EO_state.sf_info.FW_Combine * 100;
	track_send_info.Osd.EO_FYAngle = sf_angle.pitch_para * 100;//.fy_info.FY_Combine * 100;
	track_send_info.Osd.dc_state.LMCState = 0;//sys_main_info.LMCSta;//线性运动补偿状态
	track_send_info.Osd.Compass = 0;												   // 13指北针指向 1~24	1档指北，顺时针旋转，15°递增
	track_send_info.Osd.Ctrl.MianView = 0;//(sys_main_info.main_video == MainVideo_VL) ? 1 : 2; // 14 当前主视频1：可见光 2：中波红外 3：长波红外
	track_send_info.Osd.MainViewRange = 0;//sys_main_info.MianView_H;
	track_send_info.Osd.LazSta.power = laser_info.LZ_Powersta;
	track_send_info.Osd.LazCode = 0;//EO_state.fy_info.LaerNum;
	track_send_info.Osd.LazDis = 0;//EO_state.fy_info.LaserDistance;

	track_send_info.Osd.LazTemp = 0;//EO_state.fy_info.LaserTemp;


	track_send_info.Osd.Time.Year = 0;	 // 25 UTC年
	track_send_info.Osd.Time.Month = 0;	 // 26 UTC月
	track_send_info.Osd.Time.Day = 0;	 // 27 UTC日
	track_send_info.Osd.Time.Hour = 0;	 // 28 UTC时
	track_send_info.Osd.Time.Minute = 0; // 29 UTC分
	track_send_info.Osd.Time.Second = 0; // 30 UTC秒
#if 0
	turn_data_dms = UTL_Position_D_To_Dms(main_info.fj_gd.longtitude);
	track_send_info.Osd.ZJ_Pos.Lon_d = turn_data_dms.position_d;
	track_send_info.Osd.ZJ_Pos.Lon_f = turn_data_dms.position_m;
	track_send_info.Osd.ZJ_Pos.Lon_m = turn_data_dms.positon_s;
	turn_data_dms = UTL_Position_D_To_Dms(main_info.fj_gd.latitude);
	track_send_info.Osd.ZJ_Pos.Lat_d = turn_data_dms.position_d;
	track_send_info.Osd.ZJ_Pos.Lat_f = turn_data_dms.position_m;
	track_send_info.Osd.ZJ_Pos.Lat_m = turn_data_dms.positon_s;
	track_send_info.Osd.ZJ_Pos.Alt = main_info.fj_gd.alt;

	turn_data_dms = UTL_Position_D_To_Dms(aim_info.CalcLon);
	track_send_info.Osd.TarPos.Lat_d = turn_data_dms.position_d;
	track_send_info.Osd.TarPos.Lat_f = turn_data_dms.position_m;
	track_send_info.Osd.TarPos.Lat_m = turn_data_dms.positon_s;

	turn_data_dms = UTL_Position_D_To_Dms(aim_info.CalcLat);
	track_send_info.Osd.TarPos.Lat_d = turn_data_dms.position_d;
	track_send_info.Osd.TarPos.Lat_f = turn_data_dms.position_m;
	track_send_info.Osd.TarPos.Lat_m = turn_data_dms.positon_s;
	track_send_info.Osd.TarPos.Alt = aim_info.CalcAlt;
#endif
	if (CONFIG_Get_Track_Stable_Sta())
	{
		track_send_info.Osd.Ctrl.sensor_chose = 1;
	}
/*			YCTxBuf_GZQ.Sensorworkmode = 0x03; // 传感器工作模式(地理跟踪)
	else
		YCTxBuf_GZQ.Sensorworkmode = 0x05; // 传感器工作模式(手动)
*/
	//track_send_info.Osd.SetPara = User_Cmd.TraCtrl_Para3;

//	*(uint8_t *)&TraCtrl_data.OSD_overtandcovert1 = 0x00;
//	*(uint8_t *)&TraCtrl_data.OSD_overtandcovert2 = 0x00;
//	*(uint8_t *)&TraCtrl_data.OSD_overtandcovert3 = 0x00;
//	*(uint8_t *)&TraCtrl_data.OSD_overtandcovert4 = 0x00;
//	COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&track_send_info,sizeof(track_send_info));
//	Master_Ctrl_Send_data((uint8_t *)&track_send_info,sizeof(track_send_info));
	return 0;
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:后级控制指令实现
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t TRACK_API_Ctrl_SendHandle(uint8_t send_type,uint8_t *send_data)
{
	//伺服状态信息组帧发送
    static uint8_t cnt = 0;
	SYS_SF_DATA_T *sf_recv_info = NULL; 
	TRACK_TO_SF_STATUS_DATA_T track_txinfo = {0};
//	SF_SELFCHECK_union fw_check;
//	
//	sf_recv_info = CONFIG_Get_Sf_Info();
//	fw_check.tdata = sf_recv_info->check_WFW;
//	
//    track_txinfo.Header1 = SOFTWARE_ARM_SFFY;//SOFTWARE_TRACK;
//    track_txinfo.Header2 = SOFTWARE_TRACK;
//    track_txinfo.data_len = sizeof(TRACK_TO_SF_STATUS_DATA_T) - 6;
//    track_txinfo.sf_sys_sta = RxMCData.SFstate;

//    track_txinfo.amuzith = recv_sf_fw_angle;
//    track_txinfo.pitch = (GDSensor.angle-gd_set_bmq_zero) * 100; //对外上报用俯仰角度，不可用于报方位
    track_txinfo.roll = 0;//psfinfo->NHG_Angle;
//    /*陀螺*/
//    track_txinfo.amuzith_imu_speed = Gyrox * 100;
//    track_txinfo.pitch_imu_speed = Gyroy * 100;
//    track_txinfo.roll_imu_speed = Gyroz * 100;
//	if ((Gyrox == 0)||(Gyroy == 0)||(Gyroz == 0))
//	{
//		track_txinfo.gyro_sel_check = 1;
//	}
//	else
//	{
//		track_txinfo.gyro_sel_check = 0;
//	}
    /*自检*/
	if (COM_Connect_Err_Sta(COM_MOTOR))//编码器通信状态
	{
		APP_SET_BIT(track_txinfo.fy_self_check,0);//置位
	}
	else
	{
		APP_CLEAR_BIT(track_txinfo.fy_self_check,0);//取消置位
	}
	if (COM_Connect_Err_Sta(COM_MOTOR))
	{
		APP_SET_BIT(track_txinfo.fy_self_check,3);
	}
	else
	{
		APP_CLEAR_BIT(track_txinfo.fy_self_check,3);//取消置位
	}
	
	if (COM_Connect_Err_Sta(COM_SF_IN))
	{
		APP_SET_BIT(track_txinfo.fw_self_check,0);
		APP_SET_BIT(track_txinfo.fw_self_check,4);
		APP_SET_BIT(track_txinfo.fw_self_check,7);
	}
	else
	{
//		track_txinfo.fw_self_check = fw_check.u8data;
	}
		
    /*版本号*/
//    get_run_app_version(&track_txinfo.res, &track_txinfo.sf_version);
    /*TODO*/
    /*寻零状态   寻零增益*/
    /*红外信息*/
//    ir_report_info_set(&track_txinfo.ir_info);
//    track_txinfo.ir_info.field_view = hy5050_ir_focus;//焦距
//    track_txinfo.ir_info.contrast = lgcs_contrast_level;
//	track_txinfo.ir_info.luminance = lgcs_ligth_level;
//	laser_report_info_set(&track_txinfo.laser_info);
//    track_txinfo.laser_info.distance = laser_distance;//CONFIG_Get_Laser_Distance();
//	track_txinfo.laser_info.device_sta = lrd_laser_sta;
	track_txinfo.res1[3] = cnt++;
    track_txinfo.res1[4] = 0xff;
    track_txinfo.add_check = UTL_ADD_CHECK(&track_txinfo.Header1,
                                           sizeof(TRACK_TO_SF_STATUS_DATA_T) - 2);

//	memcpy(track_txdata,(uint8_t *)&track_txinfo,sizeof(TRACK_TO_SF_STATUS_DATA_T));
//	memcpy(track_txdata+sizeof(TRACK_TO_SF_STATUS_DATA_T)-2,&cdfk_data,sizeof(CDFK_DATA_T));
//	COM_API_Send_Data(COM_TRACK, track_txdata,track_txlen);
	
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:收到跟踪器的控制帧的处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
CAM_VIDEO_RECV_DATA_T recv_cam_info;
void TRACK_Recv_Ctrl_Data_Process(uint8_t *data_in,uint8_t lengh)
{
#if 1
	//解析控制数据
	TRACK_CTRL_DATA_T recv_info = {0};
	memcpy(&recv_info,data_in,sizeof(recv_info));//数据赋值
	CONFIG_Set_Main_Video(recv_info.main_video);//当前主视频
	//解析控制参数和控制指令
	if (recv_info.ctrl_object == MASTER_CTRL_SF)
	{
		if (CONFIG_Get_Board_Type() == SOFTWARE_ARM_SFFY)
		{
			CONFIG_Set_Master_Ctrl_Cmd(MASTER_FYSF_PARA,recv_info.ctrl_cmd);//解析控制指令
			CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_SF,(uint8_t*)&recv_info.FW_para1);
		}
		else if (CONFIG_Get_Board_Type() == SOFTWARE_ARM_SFFW)
		{
			CONFIG_Set_Master_Ctrl_Cmd(MASTER_FWSF_PARA,recv_info.ctrl_cmd);//解析控制指令
			CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_SF,(uint8_t*)&recv_info.FW_para1);
		}
		else 
		{
			CONFIG_Set_Master_Ctrl_Cmd(MASTER_FWSF_PARA,recv_info.ctrl_cmd);
		}
	}
	if (recv_info.ctrl_object == MASTER_CTRL_IR)
	{
		CONFIG_Set_Master_Ctrl_Cmd(MASTER_IR_PARA,recv_info.ctrl_cmd);//解析控制指令
		CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_SF,(uint8_t*)&recv_info.zh_para1);
	}
	else if (recv_info.ctrl_object == MASTER_CTRL_LASER)
	{		
		CONFIG_Set_Master_Ctrl_Cmd(MASTER_LASER_PARA,recv_info.ctrl_cmd);//解析控制指令
		CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_LASER,(uint8_t*)&recv_info.zh_para1);
	}
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:收到跟踪器的状态帧处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
CAM_VIDEO_RECV_DATA_T recv_cam_info;
void TRACK_Recv_Status_Data_Process(uint8_t *data_in,uint8_t lengh)
{
	#if 0
	SYS_TRACK_DATA_T current_main_info;
	
	memcpy(&recv_cam_info,data_in,sizeof(recv_cam_info));
	memcpy(&current_main_info.GZQStatus,&recv_cam_info.GZQStatus,sizeof(recv_cam_info.GZQStatus));
	current_main_info.fw_miss = recv_cam_info.PositionTBL;
	current_main_info.fy_miss = recv_cam_info.HeightTBL;
	memcpy(&current_main_info.SelfChecking,&recv_cam_info.GZQStatus,sizeof(recv_cam_info.GZQStatus));
	current_main_info.SurplusStorageCapacity = recv_cam_info.SurplusStorageCapacity;
	current_main_info.MultiTrackSta = recv_cam_info.OtherState;
	current_main_info.Fw_cross = recv_cam_info.Fw_cross;
	current_main_info.Fy_cross = recv_cam_info.Fy_cross;
	current_main_info.VideoBiterate = recv_cam_info.VideoBiterate;
	current_main_info.VL_Frame = recv_cam_info.VL_Frame;
	current_main_info.IR_Frame = recv_cam_info.IR_Frame;
	current_main_info.Temp = recv_cam_info.Temp;
	current_main_info.GZBM_Width = recv_cam_info.GZBM_Width;
	current_main_info.GZBM_Height = recv_cam_info.GZBM_Height;
	current_main_info.track_type = recv_cam_info.track_type.track_type;
	memcpy(&current_main_info.dest_port,&recv_cam_info.LanState,sizeof(recv_cam_info.LanState));

	CONFIG_Set_Track_Info(current_main_info);
	#endif
}
 #if ETH_INCLUDE /*网络*/
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:跟踪器数据发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
extern UDP_PCB_T track_drv_udp;
uint8_t Track_Send_data(uint8_t *send_data,uint8_t lengh)
{
#if 0
	if(track_udppcb != NULL)
	{
		udp_demo_senddata(track_udppcb,send_data,lengh);
	}
#else 
	if(track_drv_udp.udp_server != NULL)
	{
		udp_demo_senddata(track_drv_udp.udp_server,send_data,lengh);
	}
#endif
	return RECV_DATA_SUC;
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口接收数据初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TRACK_API_Serial_Data_Init(void)
{
	track_rcv_info.header1 = SOFTWARE_ARM_MC;
	track_rcv_info.header2 = SOFTWARE_TRACK;
	track_rcv_info.tail1 = 0;
	track_rcv_info.data_recv_len = 100;
	COM_Rcv_SerialPort_Init(COM_TRACK, track_rcv_info.header1,track_rcv_info.tail1,track_rcv_info.data_recv_len);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t TRACK_API_Period_Handle(void)
{
    uint8_t recv_len = 0;
	recv_len = COM_REC_DataAnalysis(COM_TRACK,track_rcv_info.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
	TRACK_Recv_Ctrl_Data_Process(track_rcv_info.recv_buf,recv_len); 
	return RECV_DATA_SUC;
}

#endif

