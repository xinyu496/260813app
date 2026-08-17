#include "Common/utl_math.h"
#include "Common/utl_check.h"
#include "APP/master_ctrl/master_ctrl.h"
#if ETH_INCLUDE
#include "Driver/drv_udp.h"
#endif

#if ETH_INCLUDE
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:上位机报文发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
extern UDP_PCB_T master_drv_udp;
uint8_t Master_Ctrl_Send_data(uint8_t *send_data,uint8_t lengh)
{
	if(master_drv_udp.udp_server != NULL)
	{
		udp_demo_senddata(master_drv_udp.udp_server,send_data,lengh);
	}
	return RECV_DATA_SUC;
}
#endif

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MASTER_API_Serial_Data_Init(void)
{
	
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:上报上位机数据
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_PERIOD_SEND_T m_rep_info = {0};
void MASTER_Normal_Rsp_Send(void)
{
#if 1
	m_rep_info.header1 = SOFTWARE_ARM_MC;//0xAA;
	m_rep_info.header2 = SOFTWARE_ARM_UPPER;//0x52;
	m_rep_info.data_len = 100;
	m_rep_info.ack_id = 3;
	m_rep_info.sys_mode = 4;
	m_rep_info.ctrl_object = 5;
	m_rep_info.cmd_result = 6;
	m_rep_info.fw_angle = 0;
	m_rep_info.fy_angle = 0;

	m_rep_info.checksum = UTL_ADD_CHECK((uint8_t *)&m_rep_info.header1,MASTER_SEND_PERIOD_LEN);
	Master_Ctrl_Send_data((uint8_t*)&m_rep_info,MASTER_SEND_PERIOD_LEN);
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:版本应答帧
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_VERSION_RSP_T dev_version_info = {0};
void MASTER_Version_Rsp_Send(void)
{
	dev_version_info.header_sender = APP_SOFTWARE_VERSION;
	dev_version_info.header_recever = SOFTWARE_ARM_MC;
	dev_version_info.data_len = 10;
	dev_version_info.mc_id = SOFTWARE_ARM_MC;
	dev_version_info.mc_ver.version.h_version = APP_HARDWARE_VERSION;
	dev_version_info.mc_ver.version.m_version = APP_REWORK_VERSION;
	dev_version_info.mc_ver.version.s_version = APP_SOFTWARE_VERSION;
	dev_version_info.mc_ver.code_time_ymd = DATE_INT;
	dev_version_info.mc_ver.code_time_hms = TIME_INT;
	dev_version_info.mc_board_id = 0;
	m_rep_info.checksum = UTL_ADD_CHECK((uint8_t *)&dev_version_info,MASTER_VERSION_RSP_LEN);
	Master_Ctrl_Send_data((uint8_t*)&dev_version_info,MASTER_SEND_PERIOD_LEN);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:版本应答帧
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_CALIBRATION_RSP_T dev_calibration_info = {0};
void MASTER_Calibration_Rsp_Send(void)
{
	dev_calibration_info.header_sender = APP_SOFTWARE_VERSION;
	dev_calibration_info.header_recever = SOFTWARE_ARM_MC;
	dev_calibration_info.data_len = 10;
	dev_calibration_info.ack_id = SOFTWARE_ARM_MC;
	dev_calibration_info.fw_encoder_zero = APP_HARDWARE_VERSION;
	dev_calibration_info.fy_encoder_zero = APP_REWORK_VERSION;
	dev_calibration_info.roll_encoder_zero = APP_SOFTWARE_VERSION;
	dev_calibration_info.adj_status = DATE_INT;
	dev_calibration_info.sf_gain = TIME_INT;
	dev_calibration_info.fw_calib_angle = 0;
	dev_calibration_info.sum = UTL_ADD_CHECK((uint8_t *)&dev_calibration_info,MASTER_CALIBRATION_RSP_LEN);
	Master_Ctrl_Send_data((uint8_t*)&dev_calibration_info,MASTER_SEND_PERIOD_LEN);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据在这里实现分流 ,实现指令分配
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void master_process_data_in_phase(uint8_t *data,uint32_t length)
{ 
	//判断头、校验
	MASTER_RCV_CTRL_CMD_T recv_ctrl_info = {0};
	memcpy((uint8_t *)&recv_ctrl_info,data,length);
	
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据在这里实现分流 ,实现指令分配
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t master_recv_buf[MASTER_RCV_CTRL_CMD_LEN] = {0};
void MASTER_API_Period_Handle(void)
{
	uint16_t data_in_len = 0;
	//取值
	if (data_in_len == 0)
	{
		return;
	}
	master_process_data_in_phase(master_recv_buf,data_in_len);
}
