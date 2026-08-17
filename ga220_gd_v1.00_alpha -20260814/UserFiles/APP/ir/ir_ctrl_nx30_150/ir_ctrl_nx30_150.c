#include "stm32f4xx_hal.h"
#include "string.h"
//#include "utl_rcv_in.h"
//#include "Common/com_api.h"
#include "Common/config.h"
#include "Common/opt_module.h"
#include "../ir/ir_ctrl_nx30_150.h"

#if (IR_NX30_150&IR_CTRL_INCLUDE)/*昆明南旭*/
#define IR_NX30_HEADER_1 0xFF
#define IR_NX30_HEADER_2 0x01

static COM_BUF_INFO_T ir_nx30_rcv_buf;
IR_NX30_CTRL_T ir_nx30_send_buf;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:每一句数据处理完之后，要对com_serial_rcv_buf进行清理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_NX30_RECBUFF_Reset(void)
{
	ir_nx30_rcv_buf.buf_rcved = false;
	ir_nx30_rcv_buf.length = 0;
	memset(ir_nx30_rcv_buf.com_data_buf,0,sizeof(ir_nx30_rcv_buf.com_data_buf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据进来之后，准备新的BUF处理数据
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool IR_Nx30_Data_in_rcv_cb(uint8_t *data,uint16_t data_len)
{		
	if ((data[0] != IR_NX30_HEADER_1)&&((data[1] != IR_NX30_HEADER_2)))
	{
		return CMD_ERR;
	}
	memcpy(ir_nx30_rcv_buf.com_data_buf,data,data_len);
	ir_nx30_rcv_buf.buf_rcved = true;
	ir_nx30_rcv_buf.length = data_len;
	
	return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_NX30_API_Serial_Data_Init(void)
{
	memset(&ir_nx30_rcv_buf,0x0,sizeof(ir_nx30_rcv_buf));
	IR_NX30_RECBUFF_Reset();
	//注册串口接收数据
	COM_API_register_RcvCb(component_map[3].com_type_in,IR_Nx30_Data_in_rcv_cb);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Ir_Ctrl_Nx30_SendHandle(SYS_IR_CMD_CTRL send_type,uint8_t *data)
{
	memset(&ir_nx30_send_buf,0x0,IR_NX30_CTRL_LEN);

	ir_nx30_send_buf.Header1 = IR_NX30_HEADER_1;
	ir_nx30_send_buf.Header2 = IR_NX30_HEADER_2;
	switch(send_type)
	{
		case IR_ZOOM_ADD:
			if (data[0] == 1)//若传进来的data[0]为1，则设置连续变倍记忆加，否则就是普通的变倍加
			{
					ir_nx30_send_buf.ctrl_cmd2 = 0x8;//连续变倍记忆加
					ir_nx30_send_buf.ctrl_data = 0x5a00;
			}
			else
			{
					ir_nx30_send_buf.ctrl_cmd2 = 0x40;
			}
		break;

		case IR_ZOOM_MINUS:
			if  (data[0] == 1)//若传进来的data[0]为1，则设置连续变倍记忆加，否则就是普通的变倍加
			{
				ir_nx30_send_buf.ctrl_cmd2 = 0x8;
				ir_nx30_send_buf.ctrl_data = 0x5900;
			}
			else
			{
				ir_nx30_send_buf.ctrl_cmd2 = 0x20;
			}
		break;

		case IR_FOCUS_ADD:
			ir_nx30_send_buf.ctrl_cmd2 = 0x80;
		break;
		
		case IR_FOCUS_MINUS:
			ir_nx30_send_buf.ctrl_cmd1 = 0x1;
		break;

		case IR_ZOOM_STOP:/*按键按下微发送，松手发送*/
		case IR_FOCUS_STOP:
		break;

		case IR_FOCUS_AUTO:
			ir_nx30_send_buf.ctrl_cmd1 = 0x64;
			ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_FOCUS_AUTO;
		break;

		case IR_CH_MLARGE:
			ir_nx30_send_buf.ctrl_cmd1 = 0x64;
			ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_VIEW_FOCUS;
			ir_nx30_send_buf.ctrl_data = 0x6c08;//根据实际需求更改其值
		break;
		
		case IR_CH_LARGE:
			ir_nx30_send_buf.ctrl_cmd1 = 0x64;
			ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_VIEW_FOCUS;
			ir_nx30_send_buf.ctrl_data = 0xec00;//根据实际需求更改其值
		break;
		
		case IR_CH_MIN:
			ir_nx30_send_buf.ctrl_cmd1 = 0x64;
			ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_VIEW_FOCUS;
			ir_nx30_send_buf.ctrl_data = 0xa004;//根据实际需求更改其值
		break;

		case IR_CH_MMIN:
			ir_nx30_send_buf.ctrl_cmd1 = 0x64;
			ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_VIEW_FOCUS;
			ir_nx30_send_buf.ctrl_data = 0xac06;//根据实际需求更改其值
		break;

		case IR_SYS_STA_REQ:
			if (data[0] == 1)
			{
				ir_nx30_send_buf.ctrl_cmd1 = 0x64;
				ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_SELFCHECK_ON;
			}
			else
			{
				ir_nx30_send_buf.ctrl_cmd1 = 0x64;
				ir_nx30_send_buf.ctrl_cmd2 = IR_NX30_SELFCHECK_OFF;
			}
		break;

		case IR_TRACK_FOCUS:
			ir_nx30_send_buf.ctrl_cmd1 = 0x68;
		if (data[0] == 1)
		{
	
			ir_nx30_send_buf.ctrl_data = 0x200;//跟焦开
		}
		else
		{
			ir_nx30_send_buf.ctrl_data = 0x100;//跟焦关
		}
		break;

		case IR_SLIGHT_ZOOM_MINUS:
			ir_nx30_send_buf.ctrl_cmd2 = 0x7;
			ir_nx30_send_buf.ctrl_data = IR_NX30_SLIGHT_ZOOM_MINUS;
		break;
		
		case IR_SLIGHT_ZOOM_ADD:
			ir_nx30_send_buf.ctrl_cmd2 = 0x7;
			ir_nx30_send_buf.ctrl_data = IR_NX30_SLIGHT_ZOOM_ADD;
		break;
		
		case IR_SLIGHT_FOCUS_ADD:
			ir_nx30_send_buf.ctrl_cmd2 = 0x7;
			ir_nx30_send_buf.ctrl_data = IR_NX30_SLIGHT_FOCUS_MINUS;
		break;
		
		case IR_SLIGHT_FOCUS_MINUS:
			ir_nx30_send_buf.ctrl_cmd2 = 0x7;
			ir_nx30_send_buf.ctrl_data = IR_NX30_SLIGHT_FOCUS_ADD;
		break;

	}

	ir_nx30_send_buf.add = UTL_ADD_CHECK((uint8_t *)&ir_nx30_send_buf,IR_NX30_CTRL_LEN-1);
	return COM_API_Send_Data(COM_IR_NX30,(uint8_t *)&ir_nx30_send_buf,sizeof(ir_nx30_send_buf));
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
void IR_NX30_PROCESS_Get(uint8_t *data,uint32_t length)
{
	IR_NX30_CTRL_T rcv_ir_info = {0};
	SYS_IR_STA_T main_ir_info;
	memcpy(&rcv_ir_info,&data,length);
	if (data[3] == IR_NX30_CUR_ZOOM_RSP)
	{
		main_ir_info.zoom_position = rcv_ir_info.ctrl_data;
	}
	if (data[3] == IR_NX30_CUR_FOCUS_RSP)
	{
		main_ir_info.ir_focus_value = rcv_ir_info.ctrl_data;
	}

	CONFIG_Set_Ir_Info(main_ir_info);
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
uint8_t IR_API_Nx30_Period_Handle(void)
{
	if (ir_nx30_rcv_buf.buf_rcved)
	{
		IR_NX30_PROCESS_Get(ir_nx30_rcv_buf.com_data_buf,ir_nx30_rcv_buf.length);		
		IR_NX30_RECBUFF_Reset();

		return RECV_DATA_SUC;
	}
	else
	{
		return RECV_DATA_NULL;
	}
	return RECV_DATA_SUC;
}
#endif

