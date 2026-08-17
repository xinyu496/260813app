#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"

#if LASER_LSP_LD_0820
#include "laser_lsp_0820.h"
#include "laser_lsp_code.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"

#define LASER_DATA_IN_HEADER_1 0x55
#define LASER_LRD_IN_HEADER_1 0x5A

static COM_RECV_INFO_T laser_serial_rcv_buf;
LASER_DATA_OUT_TO_LSP_T laser_data_out_buf;
typedef struct
{
	SYS_LASER_DETECT_MODE_E detect_type;
	uint16_t detect_time;
}DETECT_MAP_T;

DETECT_MAP_T detect_map[LZ_DETECT_END]= 
{
	{LZ_LIGHT_START,							0},
	{LZ_DETEC_STOP,								0},
	{LZ_SINGLE_DETECT,							0},
	{LZ_DETECT_FREQ_1,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_5,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_10,			LASER_DETECT_TIME},
	{LZ_DETECT_FREQ_20,			LASER_DETECT_TIME},
	{LZ_DETECT_PRECISION,		 LASER_LIGHT_TIME},
	{LZ_DETECT_CHANGE_INTERVAL,	 LASER_LIGHT_TIME},
	{LZ_DETECT_PLUSE,			 LASER_LIGHT_TIME},
	{LZ_LIGHT_EXT,				 LASER_LIGHT_TIME},
};

#define DETECT_TIME_MAX_S  	 (300)
#define DETECT_TIME_MAX_S_2  (60)
#define DETECT_TIME_MAX_S_3  (60)
#define DETECT_DELAY_MAX_S   (3000)
#define LIGHT_RANGE_MAX_1 	 (3)
#define LIGHT_RANGE_MAX_2 	 (1023)

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
	laser_serial_rcv_buf.header1 = LASER_DATA_IN_HEADER_1;
	//尾是异或校验
	laser_serial_rcv_buf.tail1 = 0;
	laser_serial_rcv_buf.data_recv_len = 14;//定长
	COM_Rcv_SerialPort_Init(COM_LASER_IN, laser_serial_rcv_buf.header1,laser_serial_rcv_buf.tail1,laser_serial_rcv_buf.data_recv_len);
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
	uint16_t range_max = LIGHT_RANGE_MAX_1; 
	uint16_t time_thr = DETECT_TIME_MAX_S_2;
	uint16_t time_value = 0;
	memset(&laser_data_out_buf,0x0,LASER_OUT_DATA_TO_LSP_LEN);
	
	laser_data_out_buf.Header = LASER_DATA_IN_HEADER_1;
	switch(cmd)
	{
		case LZ_DETEC_STOP:
			laser_data_out_buf.Ctrl_Cmd = LASER_DATA_STOP;
		break;

		case LZ_SINGLE_DETECT:
			laser_data_out_buf.Ctrl_Cmd = LASER_SINGLE_DETECT;
		break;
		case LZ_DETECT_FREQ_1:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_FREQ_1;
			time_value = detect_map[LZ_DETECT_FREQ_1].detect_time;
			time_thr = DETECT_TIME_MAX_S;
		break;	
		case LZ_DETECT_FREQ_5:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_FREQ_5;
			time_value = detect_map[LZ_DETECT_FREQ_5].detect_time;
			time_thr = DETECT_TIME_MAX_S_3;
		break;
		case LZ_DETECT_PRECISION:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_PRECISION;
			time_value = detect_map[LZ_DETECT_PRECISION].detect_time;
			range_max = LIGHT_RANGE_MAX_2;
			memcpy(&laser_data_out_buf.CtrlData[2],&cmd_para2,sizeof(cmd_para2));
		break;
		case LZ_DETECT_CHANGE_INTERVAL:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_CHANGE_INTERVAL;
			time_value = detect_map[LZ_DETECT_CHANGE_INTERVAL].detect_time;
			range_max = LIGHT_RANGE_MAX_1;
			memcpy(&laser_data_out_buf.CtrlData[2],&cmd_para2,sizeof(cmd_para2));
		break;
		case LZ_DETECT_PLUSE:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_PLUSE;
			time_value = detect_map[LZ_DETECT_PLUSE].detect_time;
			range_max = LIGHT_RANGE_MAX_1;
			memcpy(&laser_data_out_buf.CtrlData[2],&cmd_para2,sizeof(cmd_para2));
		break;
		
		case LZ_LIGHT_EXT:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_PLUSE;
			time_value = detect_map[LZ_LIGHT_EXT].detect_time;
			range_max = LIGHT_RANGE_MAX_1;
		break;
		case LZ_RX_STA:
			laser_data_out_buf.Ctrl_Cmd = LASER_APD_POWER_ENABLE;//0，电源开（默认）；1，电源关
			if (cmd_para1 == CMD_ENABLE)
			{
				cmd_para1 = 0;
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				cmd_para1 = 1;
			}
		break;

		case LZ_RANGE_GATE_VALUE:
			laser_data_out_buf.Ctrl_Cmd = LASER_RANGE_CFG;
			memcpy(&laser_data_out_buf.CtrlData[0],&cmd_para1,sizeof(cmd_para1));
		break;

		case LZ_CODE_STA:
			laser_data_out_buf.Ctrl_Cmd = LASER_CODE_ENABLE;//0，启动装订；1，关闭装订（默认）
			if (cmd_para1 == CMD_ENABLE)
			{
				cmd_para1 = 0;
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				cmd_para1 = 1;
			}
		break;	
		
		case LZ_GOAL:/*首末目标設置 0，首目标；1，末目标*/
			laser_data_out_buf.Ctrl_Cmd = LASER_DEST_MODE_CHOOSE;
			if (cmd_para1 == CMD_ENABLE)
			{
				cmd_para1 = 0;
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				cmd_para1 = 1;
			}
		break;
		
		case LZ_DYNAMI:/*动静态设置 0，动态；1，静态*/
			laser_data_out_buf.Ctrl_Cmd = LASER_STATIC_DYNAMIC_CFG;
			if (cmd_para1 == CMD_ENABLE)
			{
				cmd_para1 = 0;
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				cmd_para1 = 1;
			}
		break;
		
		case LZ_VERSION:
			laser_data_out_buf.Ctrl_Cmd = LASER_VERSION_CHECK;
		break;

		case LZ_LIGHT_CNT:
			laser_data_out_buf.Ctrl_Cmd = LASER_LIGHT_TIME_REQ;
		break;

		case LZ_POWER_CFG:/*能量设置
						D0	：0，指令控制；1，模拟控制
						D1	：额定能量的百分比
						*/
			laser_data_out_buf.Ctrl_Cmd = LASER_ENERGY_CFG;
		break;

		case LZ_RESET:
			laser_data_out_buf.Ctrl_Cmd = LASER_RESET;
		break;

		case LZ_SELF_DIS:
			laser_data_out_buf.Ctrl_Cmd = LASER_SELF_DESTROY;
			if (cmd_para1 == CMD_ENABLE)
			{
				cmd_para1 = 0;//自毁失败
			}
			else if (cmd_para1 == CMD_DISABLE)
			{
				cmd_para1 = 1;//自毁成功
			}
		break;
			
		case LZ_LIGHT_DELAY:
			laser_data_out_buf.Ctrl_Cmd = LASER_DETECT_DELAYSET;
			time_value = cmd_para1;
			time_thr = DETECT_DELAY_MAX_S;
		break;

		default:
			return CMD_ERR;
		break;
	}
	if (cmd < LZ_CTRL_CMD_END)
	{
		if ((cmd == LZ_RANGE_GATE_VALUE))
		{
			memcpy(&laser_data_out_buf.CtrlData[0],&cmd_para1,sizeof(cmd_para1));
		}
		else
		{
			if (cmd == LZ_POWER_CFG)
			{
				laser_data_out_buf.CtrlData[0] = cmd_para1;
				memcpy(&laser_data_out_buf.CtrlData[1],&cmd_para2,sizeof(cmd_para2));
			}
			else
			{
				/*时间有效性判断*/
				if (time_value > time_thr)
				{
					return PARA_ERR;
				}
				else
				{
					memcpy(laser_data_out_buf.CtrlData,&time_value,sizeof(time_value));
				}
			}
		}
	}
	else
	{
		if (cmd != LZ_RESET)
		{
			/*序列码有效性判断*/
			if (cmd_para1 > range_max)
			{
				return PARA_ERR;
			}
			else
			{
				memcpy(&laser_data_out_buf.CtrlData[0],&cmd_para1,sizeof(cmd_para1));
			}
		}
	}
	/*校验计算*/
	laser_data_out_buf.Xor = UTL_XOR_CHECK(&laser_data_out_buf.Header, (LASER_OUT_DATA_TO_LSP_LEN-1));
	
	COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&laser_data_out_buf,sizeof(laser_data_out_buf));
#if CLI_INCLUDE
	DEBUG_LASER_TX_PRINT("\r\n LASER_SEND:");
	DEBUG_LASER_TX_PRINT("%02x %02x %02x %02x %02x %02x %02x %02x" ,
		laser_data_out_buf.Header,
		laser_data_out_buf.Ctrl_Cmd,
		laser_data_out_buf.CtrlData[0],
		laser_data_out_buf.CtrlData[1],
		laser_data_out_buf.CtrlData[2],
		laser_data_out_buf.CtrlData[3],
		laser_data_out_buf.CtrlData[4],
		laser_data_out_buf.Xor);
#endif
	return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据解算，设置指令回报解算
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t laser_process_data_in(uint8_t *data,uint32_t length)
{
	LASER_DATA_IN_EB_T laser_data_in;
	LASER_SELF_CHECK_T self_check;
	SYS_LASER_STA_T *local_laser_sta;
	LASER_RECV_DETECT_T recv_detect;
	uint8_t data_xor;
	
	//长度计算
	if (length != LASER_IN_DATA_LEN)
	{
		return RECV_DATA_LEN_ERR;
	}
	data_xor = UTL_XOR_CHECK(data,(LASER_IN_DATA_LEN-1));
	//校验计算
	if(data[length-1] != data_xor)
	{
		return RECV_DATA_XOR_ERR;
	}
	memcpy(&laser_data_in,data,LASER_IN_DATA_LEN);
	local_laser_sta = CONFIG_Get_Laser_Sta();
	
	//一级指令输入信息
	switch(laser_data_in.Ctrl_Cmd)
	{
		case LASER_DATA_STOP:
			local_laser_sta->sys_sta = LASER_WORK_STAY;
		break;

		case LASER_PERIOD_SELF_CHECK:
		//解算自检的11个字节的数据
			memcpy(&self_check,&laser_data_in.CtrlData,sizeof(self_check));
				// 00：待机，01：测距，10：照射，11：休息
			if ((self_check.laser_status2.laser_status == 0x00)||(self_check.laser_status2.laser_status == 0x11))
			{
				local_laser_sta->sys_sta = LASER_WORK_STAY;
			}
			else if (self_check.laser_status2.laser_status == 0x10)
			{
				local_laser_sta->sys_sta = LASER_WORK_IRRA;
			}
			else 
			{
				local_laser_sta->sys_sta = LASER_STA_1HZ;
			}
			local_laser_sta->laser_sta.static_status = self_check.laser_status2.static_status;
			local_laser_sta->laser_sta.dest_type = self_check.laser_status1.dest_type;
			local_laser_sta->laser_sta.apd_lock = self_check.laser_status1.apd_lock;
			local_laser_sta->laser_sta.last_dest = self_check.laser_status1.ldest_status;
			local_laser_sta->laser_sta.echo_sta = self_check.laser_status1.echo_status;
			local_laser_sta->laser_sta.first_dest = self_check.laser_status1.fdest_status;
			local_laser_sta->laser_sta.main_lobe = self_check.laser_status1.main_lobe;
			
			local_laser_sta->laser_alarm.power_u5v = self_check.laser_status1.power_err;
			local_laser_sta->laser_alarm.apd_err = self_check.laser_status1.apd_err;
			local_laser_sta->laser_alarm.laser_system = self_check.laser_status2.system_status;
			local_laser_sta->laser_alarm.overheating = self_check.laser_status2.high_temp_alarm;
			local_laser_sta->laser_alarm.temperature_err = self_check.laser_status2.temp_err;
			local_laser_sta->laser_alarm.pd_err = self_check.laser_status2.pd_err;
			local_laser_sta->laser_alarm.light_enable = self_check.laser_status2.light_status;
			
			local_laser_sta->range_gate_value =  self_check.choose_range_value;
			local_laser_sta->temp =  self_check.environ_temp;
			local_laser_sta->ld_temp =  self_check.ld_temp;
			local_laser_sta->current_gear =  self_check.power_rate;
			local_laser_sta->apd_u =	self_check.apd_vol;
			local_laser_sta->light_delay = self_check.light_delay/10.0; 
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n=============================");
		char *utl_alarm_map[2] = {"故障","正常"};
		DEBUG_LASER_PRINT("\r\n%8s：%s\r\n%8s：%s\r\n%8s：%s\r\n%8s：%s \r\n%8s：%s\r\n%8s：%s\r\n%8s：%s",
				"动静态\0",local_laser_sta->laser_sta.static_status ? "静态\0":"动态\0",
				"首末目标\0",local_laser_sta->laser_sta.dest_type ? "末目标\0":"首目标\0",
				"APD闭锁\0\0",local_laser_sta->laser_sta.apd_lock ? "关\0":"开\0",
				"后目标\0",local_laser_sta->laser_sta.last_dest? "有\0":"无\0",
				"前目标\0\0",local_laser_sta->laser_sta.first_dest? "有\0":"无\0", 
				"主波\0",local_laser_sta->laser_sta.main_lobe? "有\0":"无\0",
				"回波\0",local_laser_sta->laser_sta.echo_sta ? "有\0":"无\0");
		DEBUG_LASER_PRINT("\r\n=============================");
		DEBUG_LASER_PRINT("\r\n%8s：%s\r\n%8s：%s \r\n%8s：%s \r\n%8s：%s \r\n%8s：%s \r\n%8s：%s \r\n%8s：%s",
				"5V\0",local_laser_sta->laser_alarm.power_u5v ? utl_alarm_map[0]: utl_alarm_map[1],
				"APD\0",local_laser_sta->laser_alarm.apd_err ?  utl_alarm_map[0]: utl_alarm_map[1],
				"系统\0",local_laser_sta->laser_alarm.laser_system ?  utl_alarm_map[0]: utl_alarm_map[1],
				"超温\0",local_laser_sta->laser_alarm.overheating?  utl_alarm_map[0]: utl_alarm_map[1],
				"温度\0",local_laser_sta->laser_alarm.temperature_err?  utl_alarm_map[0]: utl_alarm_map[1],
				"PD\0",local_laser_sta->laser_alarm.pd_err? utl_alarm_map[0]: utl_alarm_map[1],
				"出光\0",local_laser_sta->laser_alarm.light_enable ? "禁止\0": "允许\0");
		DEBUG_LASER_PRINT("\r\n=============================");
		DEBUG_LASER_PRINT("\r\n%10s：%d \r\n%10s：%d \r\n%10s：%d \r\n%10s：%d \r\n%10s：%d \r\n%10s：%fs",
			"选通值距离\0",local_laser_sta->range_gate_value,
			"环境温度\0",local_laser_sta->temp,
			"LD温度\0",local_laser_sta->ld_temp,
			"APD电压\0",local_laser_sta->apd_u,
			"当前挡位\0",local_laser_sta->current_gear,
			"照射延时\0",local_laser_sta->light_delay);
#endif
		break;

		case LASER_SINGLE_DETECT:
		case LASER_DETECT_FREQ_1:
		case LASER_DETECT_FREQ_5:
		case LASER_DETECT_PRECISION:
		case LASER_DETECT_CHANGE_INTERVAL:
		case LASER_LIGHT_EXT:
		case LASER_DETECT_PLUSE:
			if (laser_data_in.Ctrl_Cmd == LASER_DETECT_FREQ_1)
			{
				local_laser_sta->sys_sta = LASER_STA_1HZ;
			}
			else if (laser_data_in.Ctrl_Cmd == LASER_DETECT_FREQ_5)
			{
				local_laser_sta->sys_sta = LASER_WORK_5HZ;
			}
			memcpy(&recv_detect,&laser_data_in.CtrlData,sizeof(recv_detect));
			local_laser_sta->distance[1] = recv_detect.dest_distance;
			local_laser_sta->light_cnt = recv_detect.light_cnt;
			local_laser_sta->ld_pulse = recv_detect.ld_pulse;
			local_laser_sta->temp =  recv_detect.environ_temp;
			local_laser_sta->ld_temp =  recv_detect.ld_temp;
			local_laser_sta->laser_power = recv_detect.laser_power;
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 目标距离：%f \r\n 环境温度：%d \r\n LD温度：%d \r\n 出光次数：%d \r\n 激光能量：%fmJ \r\n LD驱动脉宽：%d",
			local_laser_sta->distance[1],
			local_laser_sta->temp,
			local_laser_sta->ld_temp,
			local_laser_sta->light_cnt,
			local_laser_sta->laser_power,
			local_laser_sta->ld_pulse);
#endif
		break;

		case LASER_DEST_MODE_CHOOSE:
			if (laser_data_in.CtrlData[0] == 1)
			{
				local_laser_sta->laser_sta.dest_type = 1;
			}
			else if (laser_data_in.CtrlData[0] == 0)
			{
				local_laser_sta->laser_sta.dest_type = 0;
			}
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n目标选择：%s ",local_laser_sta->laser_sta.dest_type ? "末目标\0":"首目标\0");
#endif
		break;

		case LASER_RANGE_CFG:
			memcpy(&local_laser_sta->range_gate_value,&laser_data_in.CtrlData[0],2);
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 选通值距离：%d ",local_laser_sta->range_gate_value);
#endif
		break;

		case LASER_LIGHT_TIME_REQ:
			memcpy(&local_laser_sta->light_total_cnt,&laser_data_in.CtrlData[0],sizeof(local_laser_sta->light_total_cnt));
#if CLI_INCLUDE
			DEBUG_LASER_PRINT("\r\n 总出光次数：%d ",local_laser_sta->light_total_cnt);
#endif
		break;

		case LASER_APD_POWER_ENABLE:
			local_laser_sta->laser_sta.apd_lock = laser_data_in.CtrlData[0];
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n APD：%s ",local_laser_sta->laser_sta.apd_lock ? "关\0":"开\0");
#endif
		break;

		case LASER_CODE_ENABLE:
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 编码使能：%s ",laser_data_in.CtrlData[0] ? "关\0":"开\0");
#endif
		break;
		
		case LASER_SELF_DESTROY:
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 编码自毁：%s ",laser_data_in.CtrlData[0] ? "成功\0":"失败\0");
#endif		
		break;
		
		case LASER_VERSION_CHECK:
			memcpy(&local_laser_sta->laser_ver[0],&laser_data_in.CtrlData[0],sizeof(local_laser_sta->laser_ver));
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n CPU: %d:%d %d  FPGA: %d:%d %d",local_laser_sta->laser_ver[0],
												local_laser_sta->laser_ver[1],
												local_laser_sta->laser_ver[2],
												local_laser_sta->laser_ver[3],
												local_laser_sta->laser_ver[4],
												local_laser_sta->laser_ver[5]);
#endif
		break;

		case LASER_ENERGY_CFG:
		local_laser_sta->laser_power = laser_data_in.CtrlData[1];
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 额定能量百分比：%f ",local_laser_sta->laser_power);
#endif
		break;
		
		case LASER_STATIC_DYNAMIC_CFG:
		/*动静态设置 0：动；1：静*/
		local_laser_sta->laser_sta.static_status = laser_data_in.CtrlData[0];
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 动静态：%s ",laser_data_in.CtrlData[0] ? "静态\0":"动态\0");
#endif
		break;
	
		case LASER_RESET:

		break;
		
		case LASER_DETECT_DELAYSET:
		memcpy(&local_laser_sta->light_delay,&laser_data_in.CtrlData[0],sizeof(local_laser_sta->light_delay));
		local_laser_sta->light_delay = (local_laser_sta->light_delay *0.1);
#if CLI_INCLUDE
		DEBUG_LASER_PRINT("\r\n 照射延时：%f ",local_laser_sta->light_delay);
#endif
		break;
	
		case LASER_UPGRADE:
		break;

		default:

		break;
	}
	
	return RECV_DATA_SUC;
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
uint8_t LASER_API_Period_Handle(void)
{
	uint8_t i = 0;
    uint8_t recv_len = 0;
	recv_len = COM_REC_DataAnalysis_nocheck(COM_LASER_IN,laser_serial_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
	laser_process_data_in(laser_serial_rcv_buf.recv_buf, recv_len);
	return RECV_DATA_SUC;
}
#endif
