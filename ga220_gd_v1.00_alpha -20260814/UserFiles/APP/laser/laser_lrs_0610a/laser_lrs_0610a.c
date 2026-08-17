#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"
/*==============================================================
波特率 460800；
*==============================================================*/
#if (LASER_LRS_0610A||LASER_LSP_0410)
#include "laser_lrs_0610a.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"

#define LASER_DATA_IN_HEADER_1 0x55
#define LASER_LRS_RECV_LEN 8
static COM_RECV_INFO_T laser_lrs_rcv_buf;

LASER_DATA_LRS_T laser_data_out_buf;

//激光测照时间对照表
typedef struct
{
	SYS_LASER_DETECT_MODE_E detect_type;
	uint16_t detect_time;
}DETECT_MAP_T;

DETECT_MAP_T detect_map[]= 
{
	{LZ_LIGHT_START,												0},
	{LZ_DETEC_STOP,													0},
	{LZ_SINGLE_DETECT,											0},
	{LZ_DETECT_FREQ_1,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_5,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_10,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_20,			LASER_DETECT_TIME},
	{LZ_DETECT_PRECISION,		 LASER_LIGHT_TIME},
	{LZ_DETECT_CHANGE_INTERVAL,LASER_LIGHT_TIME},
	{LZ_DETECT_PLUSE,			 	 LASER_LIGHT_TIME},
	{LZ_LIGHT_EXT,				 	 LASER_LIGHT_TIME},
};

#define DETECT_TIME_MAX_S  (300)
#define DETECT_TIME_MAX_S_2  (60)
#define DETECT_TIME_MAX_S_3  (60)
#define DETECT_DELAY_MAX_S  (3000)
#define LIGHT_RANGE_MAX_1 (3)
#define LIGHT_RANGE_MAX_2 (1023)
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Serial_Data_Init(void)
{
	laser_lrs_rcv_buf.header1 = LASER_DATA_IN_HEADER_1;
	//尾是异或校验
	laser_lrs_rcv_buf.tail1 = 0;
	laser_lrs_rcv_buf.data_recv_len = 6;
	COM_Rcv_SerialPort_Init(COM_LASER_IN, laser_lrs_rcv_buf.header1,
	laser_lrs_rcv_buf.tail1,laser_lrs_rcv_buf.data_recv_len);

}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光测照
*PARAMETERS:
		cmd；LASEER_DETECT_MOD
		cmd_para1:使能或者选通值等通用参数
		cmd_para2:仅用于放照射序列码
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2)
{
	memset(&laser_data_out_buf,0x0,LASER_LRS_DATA_LEN);
	
	laser_data_out_buf.Header = LASER_DATA_IN_HEADER_1;

	switch(cmd)
	{
		case LZ_DETEC_STOP:/*待机*/
			laser_data_out_buf.Ctrl_Cmd = LASER_DATA_STOP;
		break;

		case LZ_SINGLE_DETECT:
			laser_data_out_buf.Ctrl_Cmd = LASER_SINGLE_DETECT;
		break;
		case LZ_DETECT_FREQ_1://传周期
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_CONTINUE;
			laser_data_out_buf.data_para = 1000;//ms
		break;	
		
		case LZ_DETECT_FREQ_5:
#if LASER_LRS_0610A
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_CONTINUE;
			laser_data_out_buf.data_para = 200;//ms
#else
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_5HZ;
			laser_data_out_buf.data_para = 200;//ms
#endif
		break;
		
		case LZ_DETECT_FREQ_10:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_CONTINUE;
			laser_data_out_buf.data_para = 100;//ms
		break;
		
		case LZ_DETECT_SELFCHECK://自检
			laser_data_out_buf.Ctrl_Cmd = LASER_PERIOD_SELF_CHECK;
		break;
		
		case LZ_RANGE_GATE_VALUE:
			/*最近距离为100m*/
			laser_data_out_buf.Ctrl_Cmd = LASER_RANGE_CFG;
			laser_data_out_buf.data_para = cmd_para1;//ms
		break;
		
		case LZ_LIGHT_CNT:
			laser_data_out_buf.Ctrl_Cmd = LASER_LIGHT_CNT_REQ;
		break;
		
		case LZ_LIGHT_FAR_DISTANCE:
			/*最远距离为20000m*/
			laser_data_out_buf.Ctrl_Cmd = LASER_FAR_DISANCE;
			laser_data_out_buf.data_para = cmd_para1;
		break;
#if LASER_LSP_0410		
		case LZ_LIGHT_FAR_DISTANCE_REQ:
			laser_data_out_buf.Ctrl_Cmd = LASER_FAR_DISANCE_REQ;			
		break;
#endif		
		case LZ_TX_STA:/*apd电源开关*/
			if (cmd_para1 == CMD_ENABLE)
			{
				laser_data_out_buf.Ctrl_Cmd = LASER_APD_POWER_ENABLE;
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				laser_data_out_buf.Ctrl_Cmd = LASER_APD_POWER_DISENABLE;
			}
		break;
	
		case LZ_RESET:
			laser_data_out_buf.Ctrl_Cmd = LASER_LIGHT_CNT_RES;
		break;

		case LZ_LIGHT_TIMEOUT_VALUE:
			if (cmd_para1 > 20)
			{
				cmd_para1 = 20;//最大值为20min。
			}
			laser_data_out_buf.Ctrl_Cmd = LASER_TIMEOUT_VALUE;
			laser_data_out_buf.data_para = cmd_para1;
		break;	
		
		case LZ_GOAL:
#if LASER_LRS_0610A
			laser_data_out_buf.Ctrl_Cmd = LASER_WORK_MODE_CHOOSE;
			if (cmd_para1 == 1)
			{
				cmd_para1 = 0;
			}
			else if (cmd_para1 == 2)
			{
				cmd_para1 = 1;
			}
			else if (cmd_para1 == 3)
			{
				cmd_para1 = 0x0010;
			}
			laser_data_out_buf.data_para = cmd_para1;
#endif
#if LASER_LSP_0410
			laser_data_out_buf.Ctrl_Cmd = LASER_SET_AIM;
			if (cmd_para1 == 1)
			{
				cmd_para1 = 1;
			}
			else if (cmd_para1 == 2)
			{
				cmd_para1 = 3;
			}
			else
			{
				return CMD_ERR;
			}
			laser_data_out_buf.data_para = cmd_para1;
#endif
		break;

		case LZ_DETECT_ARM_ID://电路板编号查询
			laser_data_out_buf.Ctrl_Cmd = LASER_LOCALID_REQ;
		break;
			
		case LZ_DETECT_INFO_REQ:
			laser_data_out_buf.Ctrl_Cmd = LASER_DEECT_PARA_REQ;
		break;

		default:
			return CMD_ERR;
	}
	/*该字节使用大端方式传输，因此需要进行大小端转换*/
	laser_data_out_buf.data_para = UTL_Htons(laser_data_out_buf.data_para);
	/*校验计算*/
	laser_data_out_buf.Xor = UTL_XOR_CHECK(&laser_data_out_buf.Header, (LASER_LRS_DATA_LEN-1));
	COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&laser_data_out_buf,sizeof(laser_data_out_buf));
#if CLI_INCLUDE
	DEBUG_LASER_TX_PRINT("\r\n LASER_SEND:");
	DEBUG_LASER_TX_PRINT("%02x %02x %02x %02x %02x" ,
		laser_data_out_buf.Header,
		laser_data_out_buf.Ctrl_Cmd,
		laser_data_out_buf.data_len,
		laser_data_out_buf.data_para,
		laser_data_out_buf.Xor);
#endif
	return CMD_SUCESS;
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
void laser_process_data_in(uint8_t *data_in,uint32_t length)
{
	SYS_LASER_STA_T *local_laser_info;
	LASER_0410_RECV_T *lz_recv_sta;
	LASER_0610_RECV_T *lz_0610_sta;
	uint32_t distance_buf[3] = {0};
	uint8_t data_xor= 0;

	/*校验计算*/
	data_xor = UTL_XOR_CHECK(data_in,(length-1));
	if(data_in[length-1] != data_xor)
	{
		return ;
	}
	local_laser_info = CONFIG_Get_Laser_Sta();
	
	switch(data_in[2])
	{
		case LASER_DATA_STOP:
			local_laser_info->sys_sta = LASER_WORK_STAY;//测距停止
		break;
		case LASER_SINGLE_DETECT:
		case LASER_DETECT_CONTINUE:
#if LASER_LSP_0410
			/*状态解析*/
			lz_recv_sta = (LASER_0410_RECV_T *)&data_in[3];
			/*状态监控*/
			local_laser_info->laser_sta.dest_type = lz_recv_sta->first_dest;
			local_laser_info->laser_sta.apd_lock = lz_recv_sta->apd_sta;
			local_laser_info->laser_sta.last_dest = lz_recv_sta->last_dest;
			local_laser_info->laser_sta.echo_sta = lz_recv_sta->echo_lobe;
			local_laser_info->laser_sta.first_dest = lz_recv_sta->first_dest;
			local_laser_info->laser_sta.main_lobe = lz_recv_sta->main_lobe; 
			/*故障告警*/
			local_laser_info->laser_alarm.laser_system = lz_recv_sta->sys_sta;
			local_laser_info->laser_alarm.apd_err = lz_recv_sta->apd_sta;
			local_laser_info->laser_alarm.overhang = lz_recv_sta->overhang;
#endif
#if LASER_LRS_0610A
			/*状态解析*/
			lz_0610_sta = (LASER_0610_RECV_T *)&data_in[3];
			/*状态监控*/
			local_laser_info->laser_sta.dest_type = lz_0610_sta->first_dest;
			local_laser_info->laser_sta.apd_lock = lz_0610_sta->apd_sta;
			local_laser_info->laser_sta.last_dest = lz_0610_sta->last_dest;
			local_laser_info->laser_sta.echo_sta = lz_0610_sta->echo_lobe;
			local_laser_info->laser_sta.first_dest = lz_0610_sta->first_dest;
			local_laser_info->laser_sta.main_lobe = lz_0610_sta->main_lobe; 
			local_laser_info->laser_sta.light_timeout = lz_0610_sta->time_out; 
			/*故障告警*/
			local_laser_info->laser_alarm.laser_system = lz_0610_sta->sys_sta;
			local_laser_info->laser_alarm.apd_err = lz_0610_sta->apd_sta;
#endif
			/*距离解析*/
			if (local_laser_info->sys_sta == LASER_WORK_SINGLE)//单目标的解析
			{			
				memcpy(&distance_buf[0],&data_in[4],3);
				distance_buf[0] = UTL_Htonl(distance_buf[0]);
				local_laser_info->distance[0] = (distance_buf[0]>>8)*0.1f;
			}
			else
			{				
				//三目标的解析,若激光型号为lrs_0610a，则需要对local_laser_info->sys_sta 进行赋值处理
				memcpy(&distance_buf[0],&data_in[4],3);
				distance_buf[0] = UTL_Htonl(distance_buf[0]);
				local_laser_info->distance[0] = (distance_buf[0]>>8)*0.1f;
				memcpy(&distance_buf[1],&data_in[7],3);
				distance_buf[1] = UTL_Htonl(distance_buf[1]);
				local_laser_info->distance[1] = (distance_buf[1]>>8)*0.1f;
				memcpy(&distance_buf[2],&data_in[10],3);
				distance_buf[2] = UTL_Htonl(distance_buf[2]);
				local_laser_info->distance[2] = (distance_buf[2]>>8)*0.1f;
			}
		break;
			
		case LASER_PERIOD_SELF_CHECK:
#if LASER_LSP_0410
			local_laser_info->blind_area =  *(uint16_t *)(data_in + 7);
			local_laser_info->blind_area = UTL_Htons(local_laser_info->blind_area);
			local_laser_info->apd_u = data_in[10];
			local_laser_info->apd_temp = data_in[11];
#endif
#if LASER_LRS_0610A
			local_laser_info->blind_area =  *(uint16_t *)(data_in+6);
			local_laser_info->blind_area = UTL_Htons(local_laser_info->blind_area);
			local_laser_info->apd_u = data_in[7];
			local_laser_info->apd_temp = data_in[8];
#endif
		break;
		
		case LASER_RANGE_CFG:
		case LASER_FAR_DISANCE_REQ:
			local_laser_info->range_gate_value =  *(uint16_t *)(data_in+3);
			local_laser_info->range_gate_value = UTL_Htons(local_laser_info->range_gate_value);
		break;

		case LASER_LIGHT_CNT_REQ:
			local_laser_info->light_total_cnt =  *(uint32_t *)(data_in+3);
			local_laser_info->light_total_cnt = UTL_Htonl(local_laser_info->light_total_cnt);
		break;
		
		case LASER_RSP_WORK_OUT:
			local_laser_info->laser_sta.light_timeout = true;//激光处于激光工作保护，不能测距。
		break;
		
		case LASER_RSP_CHECK_ERR:
#if CLI_INCLUDE
		DEBUG_LASER_RX_PRINT("\r\n check_err!");
#endif
		break;
		
		case LASER_RSP_CONECT_OUT:
#if CLI_INCLUDE
		DEBUG_LASER_RX_PRINT("\r\n connect_err!");
#endif
		break;
	
		case LASER_APD_POWER_ENABLE:
			local_laser_info->laser_sta.apd_lock = 0;
		break;
		
		case LASER_APD_POWER_DISENABLE:
			local_laser_info->laser_sta.apd_lock = 1;
		break;
		
		case LASER_LOCALID_REQ:
			memcpy(local_laser_info->laser_ver,&data_in[11],2);
			memcpy(&local_laser_info->laser_ver[2],&data_in[9],2);
		break;
		
		default:
		break;
	}
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
RECV_DATA_ERR_STA LASER_API_Period_Handle(void)
{
	uint8_t recv_len = 0;
	recv_len = COM_REC_DataAnalysis_nocheck(COM_LASER_IN,laser_lrs_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
#if CLI_INCLUDE
	DEBUG_LASER_RX_PRINT("\r\n LASER_RCV:");
	for(uint8_t i = 0;i < recv_len;i++)
	{
		DEBUG_LASER_RX_PRINT("%02x " ,laser_lrs_rcv_buf.recv_buf[i]);
	}
#endif
	laser_process_data_in(laser_lrs_rcv_buf.recv_buf,recv_len);		

}
#endif
