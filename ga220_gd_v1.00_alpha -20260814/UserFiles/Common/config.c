#include "Common/base_inc.h"
#include "Common/config.h"
#include "Common/utl_math.h"
#if FLASH_INCLUDE
#include "Driver/drv_savecfg.h"
#endif
//网络配置
//本地ip初始化
static uint8_t local_ip[4] = {192,168,1,158};
static uint8_t local_mask[4] = {255,255,255,255};
static uint8_t local_gw[4] = {192,168,1,0};
//udp1初始化ip
static uint8_t fk_dest_ip[4] = {192,168,1,218};
static uint16_t fk_dest_port = 10048;
static uint16_t fk_sr_port = 10047;
//udp1初始化ip
static uint8_t master_dest_ip[4] = {192,168,1,184};
static uint16_t master_dest_port = 5000;
static uint16_t master_sr_port = 5000;
//udp2初始化ip 
static uint8_t fiber_dest_ip[4] = {192,168,1,133};
static uint16_t fiber_dest_port = 5001;
static uint16_t fiber_sr_port = 5001;

//各外设参数
static MASTER_CONFIG_INFO_T m_sys_info;

//顺序不可更改
COMPONENT_TYEP_MAP_T component_map[]=
{
    {IR_LGCS123,			COM_IR_LGCS},
    {IR_DYBMC_L640C500A,	COM_IR_DYBMC},
    {IR_S640A_C1,			COM_IR_S640A},
	{IR_NX30_150,     		COM_IR_NX30},
	{IR_TWIN612RG2,     	COM_IR_TWIN612RG2},
};

COMPONENT_TYEP_MAP_T component_vl_map[]=
{
    {VL_F15_300,			COM_KJG_IN},
    {VL_VS2030,				COM_KJG_VS2030},
    //{IR_S640A_C1,			COM_IR_S640A},
};


/*函数调用
红外初始化：void IR_API_Serial_Data_Init(void)
红外发送：  uint8_t Infrared_Ctrl_SendHandle(SYS_IR_CMD_CTRL send_type,uint8_t *data)
红外接收：uint8_t IR_API_Period_Handle(void)
*/
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络初始化- phy-8742
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Net_ipif_Init(void)
{
    //网口参数初始化
    memcpy(&m_sys_info.local_ip.ip,local_ip,sizeof(m_sys_info.local_ip.ip));
    memcpy(&m_sys_info.local_ip.mask,local_mask,sizeof(m_sys_info.local_ip.mask));
    memcpy(&m_sys_info.local_ip.gw,local_gw,sizeof(m_sys_info.local_ip.gw));

    memcpy(&m_sys_info.udp_config_fk.dest_ip,fk_dest_ip,sizeof(m_sys_info.udp_config_fk.dest_ip));
    m_sys_info.udp_config_fk.dest_port = fk_dest_port;
    m_sys_info.udp_config_fk.s_port = fk_sr_port;

    memcpy(&m_sys_info.udp_config_fiber.dest_ip,fiber_dest_ip,sizeof(m_sys_info.udp_config_fiber.dest_ip));
    m_sys_info.udp_config_fiber.dest_port = fiber_dest_port;
	m_sys_info.udp_config_fiber.s_port = fiber_sr_port;

	memcpy(&m_sys_info.udp_config_master.dest_ip,master_dest_ip,sizeof(m_sys_info.udp_config_master.dest_ip));
    m_sys_info.udp_config_master.dest_port = master_dest_port;
    m_sys_info.udp_config_master.s_port = master_sr_port;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Monitor_Init(void)
{
    memset(&m_sys_info,0x0,sizeof(m_sys_info));
    /*当前主视频*/
    m_sys_info.main_video = MainVideo_VL;
    /*网络参数初始化*/
    CONFIG_Net_ipif_Init();
    /*上电自检状态*/
    m_sys_info.sys_work_sta = EOMODE_IBIT;
	/*板卡类型初始化*/
	m_sys_info.board_type = SOFTWARE_ARM_SFFY;
#if 0
#if 0
    m_sys_info.arm_info.arm_outx = Query_Data(STM32FALSH_Device_ADDR, 2);
    m_sys_info.arm_info.arm_outy = Query_Data(STM32FALSH_Device_ADDR, 3);
    m_sys_info.arm_info.arm_outz = Query_Data(STM32FALSH_Device_ADDR, 4);
#else
	m_sys_info.flash_save_info.arm_outx = Query_Data(STM32FALSH_Servo_ADDR, 6);
    m_sys_info.flash_save_info.arm_outy = Query_Data(STM32FALSH_Servo_ADDR, 7);
    m_sys_info.flash_save_info.arm_outz = Query_Data(STM32FALSH_Servo_ADDR, 8);
#endif
    CHECK_RESET(m_sys_info.flash_save_info.arm_outx,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.arm_outy,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.arm_outz,int32_t);
    /*geo方位俯仰数据初始化*/
    m_sys_info.flash_save_info.geo_fw_zero = Query_Data(STM32FALSH_Servo_ADDR, 0);
    m_sys_info.flash_save_info.geo_fy_zero = Query_Data(STM32FALSH_Servo_ADDR, 1);
    m_sys_info.flash_save_info.mech_fw_adj = Query_Data(STM32FALSH_Servo_ADDR, 2);
    m_sys_info.flash_save_info.mech_fy_adj = Query_Data(STM32FALSH_Servo_ADDR, 3);
    m_sys_info.flash_save_info.bmq_fw_zero = Query_Data(STM32FALSH_Servo_ADDR, 4);
    m_sys_info.flash_save_info.bmq_fy_zero = Query_Data(STM32FALSH_Servo_ADDR, 5);
#endif
    CHECK_RESET(m_sys_info.flash_save_info.geo_fw_zero,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.geo_fy_zero,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.mech_fw_adj,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.mech_fy_adj,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.bmq_fw_zero,int32_t);
    CHECK_RESET(m_sys_info.flash_save_info.bmq_fy_zero,int32_t);

    /*大中小视场对应的焦距值初始化*/
//#if (VISIBLE_INCLUDE&VL_F15_300)
//    m_sys_info.vl_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(VLPixel,VLPNum_H,VLSetLarView);
//    m_sys_info.vl_view_of_focus[MID_VIEW_FOCUS] = UTL_Focus_Calc(VLPixel,VLPNum_H,VLSetMidView);
//    m_sys_info.vl_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(VLPixel,VLPNum_H,VLSetSmaView);
//    m_sys_info.vl_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(VLPixel,VLPNum_H,VLSetMinView);
//#endif

//#if ((IR_CTRL_INCLUDE&IR_S640A_C1)||(IR_CTRL_INCLUDE&IR_DYBMC_L640C500A))
//    m_sys_info.ir_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(IRPixel,IRPNum_H,IRSetLarView);
//    m_sys_info.ir_view_of_focus[MID_VIEW_FOCUS] = UTL_Focus_Calc(IRPixel,IRPNum_H,IRSetMidView);
//    m_sys_info.ir_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(IRPixel,IRPNum_H,IRSetSmaView);
//    m_sys_info.ir_view_of_focus[LARGE_VIEW_FOCUS] = UTL_Focus_Calc(IRPixel,IRPNum_H,IRSetMinView);
//#endif
}

/*===================================================================
 * FUNCTION NAME:
 *	config_dump_flashpage1
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	N/A
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015/4/17 by LmnyL, Create
 *=================================================================*/
void config_dump_sf_info(void)
{
	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
	printf("\r\n yaw: %f pitch: %f roll: %f",m_sys_info.sf_info.FW_Combine,m_sys_info.sf_info.FY_Combine,m_sys_info.sf_info.HG_Combine);
//	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
//	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
//	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
//	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
//	printf("\r\n sf_status : %d",m_sys_info.sf_info.sf_sta);
	

}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取当前设备ip
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_LOCAL_IP_T CONFIG_Get_Current_ip(void)
{
    return m_sys_info.local_ip;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取当前UDP目的ip
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
UDP_CONFIG_T CONFIG_Get_Current_Udpfk(void)
{
    return m_sys_info.udp_config_fk;
}

UDP_CONFIG_T CONFIG_Get_Current_Udpfiber(void)
{
    return m_sys_info.udp_config_fiber;
}

UDP_CONFIG_T CONFIG_Get_Current_Udpmaster(void)
{
    return m_sys_info.udp_config_master;
}

void CONFIG_Get_Cali_vlFocus(float *focus)
{
    /*获取可见光标定的大中小视场对应的焦距值*/
    memcpy(focus,m_sys_info.vl_view_of_focus,sizeof(m_sys_info.vl_view_of_focus));
}

void CONFIG_Get_Cali_irFocus(float *focus)
{
    /*获取红外标定的大中小视场对应的焦距值*/
    memcpy(focus,m_sys_info.ir_view_of_focus,sizeof(m_sys_info.ir_view_of_focus));
}

void CONFIG_Printf_Dump(void)
{
	printf("\r\n laser_dis[0] %f",m_sys_info.laser_info.distance[0]);
	printf("\r\n laser_dis[1] %f",m_sys_info.laser_info.distance[1]);
	printf("\r\n laser_dis[2] %f",m_sys_info.laser_info.distance[2]);

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
FLASH_SAVE_INFO_T CONFIG_Get_Geo_Zero(void)
{
    return m_sys_info.flash_save_info;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Geo_Zero(FLASH_SAVE_INFO_T* geo_info)
{
    memcpy(&m_sys_info.flash_save_info,geo_info,sizeof(FLASH_SAVE_INFO_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Main_Video(void)
{
    return m_sys_info.main_video;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Main_Video(uint8_t main_video)
{
    m_sys_info.main_video = main_video;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取板卡类型
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Board_Type(void)
{
    return m_sys_info.board_type;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Adjust_Sta(uint8_t value)
{
    m_sys_info.adj_sta = value;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Adjust_Sta(void)
{
    return m_sys_info.adj_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取全部的参数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_CONFIG_INFO_T CONFIG_GET_SYS_Main_Info(void)
{
    return m_sys_info;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Plane_Data_Sta(void)
{
    return m_sys_info.fj_gd_valid;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Plane_Data_Sta(DATA_VALIDITY_ENUM status)
{
    m_sys_info.fj_gd_valid = status;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t *CONFIG_Get_Arm_info(void)
{
	static uint32_t arm_info[3] = {0};
    memcpy(arm_info,&m_sys_info.flash_save_info.arm_outx,12);
	return arm_info;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Arm_info(uint32_t *arm_info)
{
    memcpy(&m_sys_info.flash_save_info.arm_outx,arm_info,12);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sf_Attitude(ATTITUDE_PAPA_T attitude)
{
    memcpy(&m_sys_info.sf_attitude,&attitude,sizeof(attitude));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
ATTITUDE_PAPA_T CONFIG_Get_Sf_Attitude(void)
{
    return m_sys_info.sf_attitude;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Dst_Position(POSITION_INFO_T position)
{
    memcpy(&m_sys_info.dst_position,&position,sizeof(position));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
POSITION_INFO_T CONFIG_Get_Dst_Position(void)
{
    return m_sys_info.dst_position;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Master_Ctrl_Cmd(MASTER_PARA_ENUM object)
{
    return m_sys_info.master_ctrl_cmd[object];
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Master_Ctrl_Cmd(MASTER_PARA_ENUM object,uint8_t cmd)
{
    m_sys_info.master_ctrl_cmd[object] = cmd;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_CTRL_TYPE_ENUM CONFIG_Get_Master_Ctrl_Sta(void)
{
    return m_sys_info.master_ctrl_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_TYPE_ENUM cmd_sta)
{
    m_sys_info.master_ctrl_sta = cmd_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:设置控制类型和控制参数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_TYPE_ENUM set_type,uint8_t *value)
{
//	MASTER_PARA_ENUM ctrl_type;
    //无参数设置，则直接返回
    if(set_type == MASTER_CTRL_NULL)
    {
        return;
    }
    //设置需要控制的模块
    m_sys_info.master_ctrl_sta = set_type;
    //根据不同的模块传参
    if (APP_IS_BIT_SET(set_type,MASTER_IR_PARA))
    {
        memcpy(&m_sys_info.master_ctrl_buf[MASTER_IR_PARA][0],value,8);
    }
    if (APP_IS_BIT_SET(set_type,MASTER_VL_PARA))
    {
        memcpy(&m_sys_info.master_ctrl_buf[MASTER_VL_PARA][0],value,8);
    }

    if (APP_IS_BIT_SET(set_type,MASTER_LASER_PARA))
    {
        memcpy(&m_sys_info.master_ctrl_buf[MASTER_LASER_PARA][0],value,8);
    }

    if (APP_IS_BIT_SET(set_type,MASTER_TRACK_PARA))
    {
        memcpy(&m_sys_info.master_ctrl_buf[MASTER_TRACK_PARA][0],value,8);
    }

    if (APP_IS_BIT_SET(set_type,MASTER_SF_PARA))
    {
        memcpy(&m_sys_info.master_ctrl_buf[MASTER_SF_PARA][0],value,8);
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION：获取控制参数
*PARAMETERS:ctrl_type：传入参数的类型；*data_para：取出来的数据
*N/A
*NOTES:传入时应该注意：ctrl_type是MASTER_PARA_ENUM类型
*HISTORY:
*==============================================================*/
void CONFIG_Get_Master_Ctrl_Para(uint8_t ctrl_type,uint8_t *data_para)
{
    memcpy(data_para,&m_sys_info.master_ctrl_buf[ctrl_type][0],8);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Ext_position(EXT_GD_DATA_T position)
{
    memcpy(&m_sys_info.fj_gd,&position,sizeof(EXT_GD_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
EXT_GD_DATA_T CONFIG_Get_Ext_position(void)
{
    return m_sys_info.fj_gd;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Internal_position(INSIDE_GD_DATA_T position)
{
    memcpy(&m_sys_info.inner_gd,&position,sizeof(INSIDE_GD_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
INSIDE_GD_DATA_T CONFIG_Get_Internal_position(void)
{
    return m_sys_info.inner_gd;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Laser_Sta(SYS_LASER_STA_T laser_curr_sta)
{
    memcpy(&m_sys_info.laser_info,&laser_curr_sta,sizeof(SYS_LASER_STA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Laser_Dist(const SYS_LASER_STA_T *laser_curr_dist)
{
    memcpy(&m_sys_info.laser_info.distance,laser_curr_dist->distance,sizeof(laser_curr_dist->distance));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Get_Laser_Dist(uint16_t laser_curr_dist)
{
    memcpy((uint16_t *)&laser_curr_dist,m_sys_info.laser_info.distance,sizeof(m_sys_info.laser_info.distance));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_LASER_STA_T *CONFIG_Get_Laser_Sta(void)
{
    return &m_sys_info.laser_info;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sf_Info(SYS_SF_DATA_T *sf_info)
{
    memcpy(&m_sys_info.sf_info,sf_info,sizeof(SYS_SF_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_SF_DATA_T *CONFIG_Get_Sf_Info(void)
{
    return &m_sys_info.sf_info;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:系统编译时间
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_TIME_T CONFIG_Get_Sys_Time(void)
{
    m_sys_info.code_time.year = YEAR;
    m_sys_info.code_time.month = MONTH;
    m_sys_info.code_time.day = DAY;
    m_sys_info.code_time.hour = HOUR;
    m_sys_info.code_time.minute = MINUTE;
    m_sys_info.code_time.second = SECOND;

    return m_sys_info.code_time;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_TDC_Sta(void)
{
    return m_sys_info.tdc_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_TDC_Sta(bool tdc_sta)
{
    m_sys_info.tdc_sta = tdc_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Axis_Sta(void)
{
    return m_sys_info.Axis_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Axis_Sta(bool value)
{
    m_sys_info.Axis_sta = value;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Decenter_Sta(void)
{
    return m_sys_info.Traoffcenter_flg;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Decenter_Sta(bool value)
{
    m_sys_info.Traoffcenter_flg = value;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_LMC_Sta(void)
{
    return m_sys_info.LMCSta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_LMC_Sta(bool value)
{
    m_sys_info.LMCSta = value;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_View_Match_Sta(void)
{
    return m_sys_info.view_match_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_View_Match_Sta(bool flag)
{
    m_sys_info.view_match_sta = flag;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_CTRL_MODE_E CONFIG_Get_Sys_Work_Mode(void)
{
    return m_sys_info.sys_work_mode;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sys_Work_Mode(SYS_CTRL_MODE_E work_mode)
{
    m_sys_info.sys_work_mode = work_mode;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SF_MODE_E CONFIG_Get_SF_Mode(void)
{
    return m_sys_info.sf_mode;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_SF_Mode(SF_MODE_E info)
{
    m_sys_info.sf_mode = info;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Visible_Info(SYS_VISIBLE_DATA_T visi_info)
{
    memcpy(&m_sys_info.vl_info,&visi_info,sizeof(SYS_VISIBLE_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_VISIBLE_DATA_T CONFIG_Get_Visible_Info(void)
{
    return m_sys_info.vl_info;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_IR_STA_T* CONFIG_Get_Ir_Info(void)
{
    return &m_sys_info.ir_info;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Ir_Info(SYS_IR_STA_T* ir_info)
{
    memcpy(&m_sys_info.ir_info,&ir_info,sizeof(SYS_IR_STA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_TRACK_DATA_T CONFIG_Get_Track_Info(void)
{
    return m_sys_info.track_info;
}
/*=============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Info(SYS_TRACK_DATA_T cam_info)
{
    memcpy(&m_sys_info.track_info,&cam_info,sizeof(SYS_TRACK_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Plane_Info(SYS_FCU_STATUS plane_Data)
{
    memcpy(&m_sys_info.plane_Data,&plane_Data,sizeof(SYS_FCU_STATUS));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_FCU_STATUS CONFIG_Get_Plane_Sta(void)
{
    return m_sys_info.plane_Data;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Fk_Gd_Info(EXT_GD_DATA_T fj_gd)
{
    memcpy(&m_sys_info.fj_gd,&fj_gd,sizeof(EXT_GD_DATA_T));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
EXT_GD_DATA_T CONFIG_Get_Fk_Gd_Info(void)
{
    return m_sys_info.fj_gd;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sf_Reach_Sta(uint8_t status)
{
   m_sys_info.sf_reach_sta = status;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Sf_Reach_Sta(void)
{
    return m_sys_info.sf_reach_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Stable_Sta(bool status)
{
   m_sys_info.stable_sta = status;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Track_Stable_Sta(void)
{
    return m_sys_info.stable_sta;
}
#if FLASH_INCLUDE
/*==============================================================
*FUNC:	
*DESC:	
*PARAM: 
*RETURN:	
*N/A
*NOTES:(uint8_t*)&m_sys_info.flash_save_info
*HISTORY:
*==============================================================*/
void save_sys_userdata(void)
{
    write_config_crc((uint8_t*)&m_sys_info.flash_save_info,sizeof(FLASH_SAVE_INFO_T));
}

void load_sys_userdata(void)
{
	FLASH_SAVE_INFO_T tmp_info;
	
    uint16_t ret = read_config_crc((uint8_t*)&tmp_info,sizeof(FLASH_SAVE_INFO_T));
	if(STATUS_SUCCESS == ret)
	{
		memcpy(&m_sys_info.flash_save_info,&tmp_info,sizeof(FLASH_SAVE_INFO_T));
	}
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
TRACK_MISS_T CONFIG_Get_Track_Miss(void)
{
    return m_sys_info.dev_miss;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Miss(TRACK_MISS_T miss)
{
    m_sys_info.dev_miss.fw_miss = miss.fw_miss;
	m_sys_info.dev_miss.fy_miss = miss.fy_miss;
}


