#include "Driver/drv_uart.h"
#include "Common/utl_check.h"

#if LASER_LRD_25B07
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif

#include "laser_lrd_25b07.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"
/*DYB-01YC1协议一致 波特率 460800*/
LASER_RxFrame_t g_laser_rx;
static LASER_TxFrame_t g_laser_tx;

static COM_RECV_INFO_T laser_serial_rcv_buf;//接收缓冲区
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:报文解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint8_t laser_process_lrd_data_in(uint8_t *data, uint32_t length){
	if(length < LASER_RX_FRAME_LEN)
		return RECV_DATA_NULL;
	/*搜索帧头*/
	for(uint32_t i=0;i<=length-LASER_RX_FRAME_LEN;i++){
		if(data[i] == LASER_HEAD1 && data[i+1] == LASER_HEAD2){
			uint8_t *frame = &data[i];
			uint8_t sum = 0;
			for(uint8_t j = 2;j <= 22;j++)
			{
				sum += frame[j];
			}
			if(sum != frame[23])
				continue;
			/*解析到全局结构体*/
			LASER_RxFrame_t *rx = &g_laser_rx;
			rx->header1 = frame[0];
			rx->header2 = frame[1];
			rx->status1 = frame[2];			
			rx->status2 = frame[3];
			rx->first_dist = frame[4] | ((uint16_t)frame[5] << 8);
			rx->work_time[0] = frame[6];	
			rx->work_time[1] = frame[7];
			rx->work_time[2] = frame[8];	
			rx->temp = (int8_t)frame[9];
			rx->light_code = frame[10];
			rx->apd_voltage = frame[11];
			rx->status3 = frame[12];
			rx->sync_code = frame[15];
			rx->second_dist = frame[19] | ((uint16_t)frame[20] << 8);
			rx->third_dist = frame[21] | ((uint16_t)frame[22] << 8);
			rx->sum = frame[23];
			
			uint8_t target_valid = (rx->status1 >> 0) & 0x01;
			if(target_valid){
				uint8_t target_cnt = rx->status3 & 0x03; //目标数量
				SYS_LASER_STA_T laser_dist;
				memset(&laser_dist,0,sizeof(laser_dist));
				laser_dist.distance[0] = (float)rx->first_dist;
				if(target_cnt >= 2){
				laser_dist.distance[1] = (float)rx->second_dist;
				}
				if(target_cnt >= 3){
				laser_dist.distance[2] = (float)rx->third_dist;
				}
				CONFIG_Set_Laser_Dist(&laser_dist);
			}
			
			return RECV_DATA_SUC;
		}
	}
	return RECV_DATA_NULL;//没找到数据帧
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光模块初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Serial_Data_Init(void)
{
	memset(&g_laser_rx,0,sizeof(g_laser_rx));
	memset(&g_laser_tx,0,sizeof(g_laser_tx));
	memset(&laser_serial_rcv_buf,0,sizeof(laser_serial_rcv_buf));
	
	laser_serial_rcv_buf.header1 = LASER_HEAD1;
	laser_serial_rcv_buf.header2 = LASER_HEAD2;
	laser_serial_rcv_buf.data_recv_len = LASER_RX_FRAME_LEN;
	COM_Rcv_SerialPort_Init(COM_LASER_IN,(LASER_HEAD1 << 8)|(LASER_HEAD2),0,LASER_RX_FRAME_LEN);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:周期接受处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Period_Handle(void){
		uint8_t recv_len = 0;
		recv_len = COM_REC_Data_Direct(COM_LASER_IN,laser_serial_rcv_buf.recv_buf);
		if(recv_len == 0)
			return;
		laser_process_lrd_data_in(laser_serial_rcv_buf.recv_buf,recv_len);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:发送指令到激光模块
*PARAMETERS:
		cmd；控制字
		param1:参数1（编码序号L）
		param2:参数2（编码序号H）
		code_L:编码值L
		code_M:编码值M
		code_H:编码值H
		sync:时统码
		cnt:编码数量
		group:编码组号
*RETURN:
*N/A
*NOTES:具体执行指令在应用代码中编写，调用完Laser_SendCommand函数后请使用CONFIG_Set_Master_Ctrl_Sta和CONFIG_Set_Master_Ctrl_Para进行状态清空，参数清空
*HISTORY:
*==============================================================*/
uint8_t Laser_SendCommand(LASER_CMD_E cmd,uint8_t param1,uint8_t param2,uint8_t code_L,uint8_t code_M,uint8_t code_H,uint8_t sync,uint8_t cnt,uint8_t group)
{
		memset(&g_laser_tx,0,sizeof(g_laser_tx));
		g_laser_tx.header1 = LASER_HEAD1;
		g_laser_tx.header2 = LASER_HEAD2;
		g_laser_tx.cmd = cmd;
		g_laser_tx.param1 = param1;
		g_laser_tx.code_val_L = code_L;
		g_laser_tx.code_val_M = code_M;
		g_laser_tx.code_val_H = code_H;
		g_laser_tx.param2 = param2;
		g_laser_tx.sync_code = sync;
		g_laser_tx.param3 = cnt;
		g_laser_tx.param4 = group;
		g_laser_tx.sum = UTL_ADD_CHECK((uint8_t *)&g_laser_tx.cmd,sizeof(g_laser_tx)-3);
	
		COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&g_laser_tx,LASER_TX_FRAME_LEN);
		return 1;
}

/*各指令发送实现*/
/*空指令*/
static void Laser_Send_Null(void){
	Laser_SendCommand(LASER_CMD_NULL,0,0,0,0,0,0,0,0);
}
/*维护自检*/
static void Laser_Send_SelfCheck(void){
	Laser_SendCommand(LASER_CMD_SELF_CHECK,0,0,0,0,0,0,0,0);
}
/*停止测照*/
static void Laser_Send_Stop(void){
	Laser_SendCommand(LASER_CMD_STOP,0,0,0,0,0,0,0,0);
}
/*单次发射*/
static void Laser_Send_Single(void){
	Laser_SendCommand(LASER_CMD_SINGLE,0x64,0,0,0,0,0,0,0);
}
/*1HZ测距*/
static void Laser_Send_1Hz(void){
	Laser_SendCommand(LASER_CMD_DETECT_FREQ_1,0x64,0,0,0,0,0,0,0);
}
/*MARK测距*/
static void Laser_Send_Mark(void){
	Laser_SendCommand(LASER_CMD_DETECT_MARK,0x64,0,0,0,0,0,0,0);
}
/*5HZ测距*/
static void Laser_Send_5Hz(void){
	Laser_SendCommand(LASER_CMD_DETECT_FREQ_5,0x64,0,0,0,0,0,0,0);
}
/*20HZ测距*/
static void Laser_Send_20Hz(void){
	Laser_SendCommand(LASER_CMD_DETECT_FREQ_20,0x64,0,0,0,0,0,0,0);
}
/*10HZ测距*/
static void Laser_Send_10Hz(void){
	Laser_SendCommand(LASER_CMD_DETECT_FREQ_10,0x64,0,0,0,0,0,0,0);
}
/*激光照射1*/
static void Laser_Send_Light1(uint8_t code_index){
	Laser_SendCommand(LASER_CMD_LIGHT1,code_index,0,0,0,0,0,0,0);
}
/*激光照射2*/
static void Laser_Send_Light2(uint8_t code_index){
	Laser_SendCommand(LASER_CMD_LIGHT2,code_index,0,0,0,0,0,0,0);
}
/*编码装订*/
static void Laser_Send_CodeSet(uint8_t code_index,uint8_t code_L,uint8_t code_M,uint8_t code_H){
	Laser_SendCommand(LASER_CMD_CODE_SET,code_index,0,code_L,code_M,code_H,0,0,0);
}
/*打光次数查询*/
static void Laser_Send_LightQuery(void){
	Laser_SendCommand(LASER_CMD_LIGHT_QUERY,0x64,0,0,0,0,0,0,0);
}
/*激光使能*/
static void Laser_Send_Enable(void){
	Laser_SendCommand(LASER_CMD_ENABLE,0,0,0,0,0,0,0,0);
}
/*激光禁止*/
static void Laser_Send_Disable(void){
	Laser_SendCommand(LASER_CMD_DISABLE,0,0,0,0,0,0,0,0);
}
/*低功耗开启*/
static void Laser_Send_LowPwrOn(void){
	Laser_SendCommand(LASER_CMD_LOW_POWER_ON,0,0,0,0,0,0,0,0);
}
/*低功耗关闭*/
static void Laser_Send_LowPwrOff(void){
	Laser_SendCommand(LASER_CMD_LOW_POWER_OFF,0,0,0,0,0,0,0,0);
}
/*激光接收开*/
static void Laser_Send_RxOn(void){
	Laser_SendCommand(LASER_CMD_RX_ON,0,0,0,0,0,0,0,0);
}
/*激光接收关*/
static void Laser_Send_RxOff(void){
	Laser_SendCommand(LASER_CMD_RX_OFF,0,0,0,0,0,0,0,0);
}
/*切换到内时统*/
static void Laser_Send_InnerSync(void){
	Laser_SendCommand(LASER_CMD_INNER_SYNC,0,0,0,0,0,0,0,0);
}
/*切换到外时统*/
static void Laser_Send_OuterSync(void){
	Laser_SendCommand(LASER_CMD_OUTER_SYNC,0,0,0,0,0,0,0,0);
}
/*到正常能量*/
static void Laser_Send_NomalEn(void){
	Laser_SendCommand(LASER_CMD_NORMAL_EN,0,0,0,0,0,0,0,0);
}
/*到小能量*/
static void Laser_Send_LowEn(void){
	Laser_SendCommand(LASER_CMD_LOW_EN,0,0,0,0,0,0,0,0);
}
/*设置盲区*/
static void Laser_Send_BlindSet(uint16_t blind_value){
	uint8_t val_L = blind_value & 0xFF;
	uint8_t val_M = (blind_value >> 8) & 0xFF;
	Laser_SendCommand(LASER_CMD_BLIND_SET,0,0,val_L,val_M,0,0,0,0);
}
/*距离平均使能/禁用*/
static void Laser_Send_AvgEn(uint8_t enable){
	Laser_SendCommand(LASER_CMD_AVG_EN,enable ? 1:0,0,0,0,0,0,0,0);
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:根据上位机指令码发送激光指令
*PARAMETERS:
	master_cmd：上位机下发指令码
	para1：参数1
	para2：参数2
*RETURN: 1：成功 0：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E master_cmd,uint16_t para1,uint16_t para2){
	uint8_t result = 1;
	switch(master_cmd){
//		case LASER_CMD_NULL://空指令
//			Laser_Send_Null();
//			break;
		case LZ_STATUS_ASK://维护自检
			Laser_Send_SelfCheck();
			break;	
		case LZ_DETEC_STOP://停止测照
			Laser_Send_Stop();
			break;	
		case LZ_SINGLE_DETECT://单次发射
			Laser_Send_Single();
			break;	
		case LZ_DETECT_FREQ_1://1HZ测距
			Laser_Send_1Hz();
			break;	
		case LZ_DETECT_MARK://MARK测距
			Laser_Send_Mark();
			break;
		case LZ_DETECT_FREQ_5://5HZ测距
			Laser_Send_5Hz();
			break;	
		case LZ_DETECT_FREQ_20://20HZ测距
			Laser_Send_20Hz();
			break;
		case LZ_DETECT_FREQ_10://10HZ测距
			Laser_Send_10Hz();
			break;		
		case LZ_LIGHT://激光照射1
			if(para1 == 1){
				Laser_Send_Light1(0);
			}
			else if(para1 == 2){
				Laser_Send_Light2(0);
			}	
			break;

		case LZ_CODE_WRITE://编码装订 para1 = 编码序号 para2 = 编码值低16位
			Laser_Send_CodeSet((uint8_t) para1,(uint8_t) (para2 & 0xFF),(uint8_t) ((para2 >> 8) & 0xFF),(uint8_t) ((para2 >> 16) & 0xFF));
			break;
//		case 0xAA://打光次数查询
//			Laser_Send_LightQuery();
//			break;
		case LZ_TX_STA://激光使能
			if (para1 == 1)
			{
				Laser_Send_Enable();
			}
			else if (para1 == 2)
			{
				Laser_Send_Disable();
			}
			break;
//		case LASER_CMD_LOW_POWER_ON://低功耗开启
//			Laser_Send_LowPwrOn();
//			break;
//		case LASER_CMD_LOW_POWER_OFF://低功耗关闭
//			Laser_Send_LowPwrOff();
//			break;
		case LZ_RX_STA://激光接收
			if(para1 == 1){
				Laser_Send_RxOn();
			}
			else if(para1 == 2){
				Laser_Send_RxOff();
			}
			break;

//		case 0xBB://到正常能量
//			Laser_Send_NomalEn();
//			break;
//		case LASER_CMD_LOW_EN://到小能量
//			Laser_Send_LowEn();
//			break;
//		case 0xCC://设置盲区
//			Laser_Send_BlindSet(para1);
//			break;
//		case LASER_CMD_AVG_EN://距离平均使能/禁用
//			Laser_Send_AvgEn((uint8_t)para1);
//			break;
		default:
			result = 0;
			break;
	}
	return result;
}

#endif


