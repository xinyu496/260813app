#include "stm32f4xx_hal.h"
#include "string.h"
#include "Common/utl_math.h"
#include "laser_lsp_code.h"
#include "Common/config.h"
#include <stdio.h>
#if LASER_LSP_LD_0820
#define LASER_CODE_SEND_HEAD  		0x55
#define LASER_CODE_INTERVAL_CMD   0x30
#define LASER_CODE_INTERVAL_REQ  	0x31 
#define LASER_CODE_PULSE_CMD   		0x32
#define LASER_CODE_PULSE_REQ   		0x33
#define LASER_CODE_PRECISION_CMD  0x34
#define LASER_CODE_PRECISION_REQ  0x35
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:变间隔码照射
*PARAMETERS:code_class：组号；
						Datalen：数据个数，一个32位的变间隔码算一个数据；
						code：变间隔码数组
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Change_Interval_Send(uint8_t code_class,uint16_t Datalen,uint32_t *code)
{
	uint8_t send_page = 0;
	LASER_LSP_CODE_T code_send_buf = {0};
	uint8_t total_page = 0;
	bool send_sta = false;
	
	//该组字节最多1024
	if (Datalen > 1024)
	{
		Datalen = 1024;
	}

	//算总页数
	if (0 != (Datalen%11))
	{
		total_page = (Datalen/11) + 1;
	}
	else
	{
		total_page = (Datalen/11);
	}

	code_send_buf.head = LASER_CODE_SEND_HEAD;
	code_send_buf.cmd_type = LASER_CODE_INTERVAL_CMD;
	code_send_buf.code_class = code_class;
	code_send_buf.code_len = Datalen;

	//循环发送
	for (send_page = 0;send_page < total_page;send_page++)
	{
		//当发到最后一页时，需要状态置位
		if (send_page == (total_page - 1))
		{
			send_sta = true;//发最后一帧的时候，将该标志位置为true
			send_page = 0xFF;//分页结束序号
		}
		//如果刚好是整数页，每页的数据量都是11
		if (send_page != 0xFF)
		{
			code_send_buf.code_cur_len = 11;
			memcpy(code_send_buf.code_data,&code[send_page*11],sizeof(code_send_buf.code_data));
		}
		else
		{
			//最后一阵的数据需要特殊处理
			code_send_buf.code_cur_len = Datalen%11;
			//先将最后一帧清0
			memset(code_send_buf.code_data,0,sizeof(code_send_buf.code_data));
			//再只放该放的数据进去
			memcpy(&code_send_buf.code_data[0],&code[(total_page - 1)*11],code_send_buf.code_cur_len*4);
		}
		code_send_buf.code_cur_page = send_page;
		code_send_buf.xor = UTL_XOR_CHECK(&code_send_buf.head,sizeof(code_send_buf)-1);
		
		COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&code_send_buf,sizeof(code_send_buf));

		if (send_page == 0xFF)
		{
			//跳出循环，发完了。
			send_page = total_page;
		}
		//由于dma发送报文后需要更新状态，因此这里必须加时延，避免不发包的情况
		HAL_Delay(1);
	}
	return send_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:脉冲序列码写入
*PARAMETERS:code_class：组号；
						base_freq：基频
						Datalen：数据个数，一个8位的脉冲序列码算一个数据；
						code：脉冲序列码数组
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Pulse_Send(uint8_t code_class,uint8_t base_freq,uint16_t Datalen,uint8_t *code)
{
	uint8_t send_page = 0;
	LASER_LSP_CODE_T code_send_buf = {0};
	uint8_t total_page = 0;
	bool send_sta = false;
	
	//该组字节最多128
	if (Datalen > 128)
	{
		Datalen = 128;
	}
	
	//算总页数，总共128个字节    
	if (0 != (Datalen%44))
	{
		total_page = (Datalen/44) + 1;
	}
	else
	{
		total_page = (Datalen/44);
	}

	code_send_buf.head = LASER_CODE_SEND_HEAD;
	code_send_buf.cmd_type = LASER_CODE_PULSE_CMD;
	code_send_buf.code_class = code_class;
	//这里需要分成两个字节，低8位为该组基频，
	code_send_buf.code_len = base_freq;
	//高8位放该组字节数
	code_send_buf.code_len = (((Datalen & 0x00ff) << 8)|code_send_buf.code_len);
	for (send_page = 0;send_page < total_page;send_page++)
	{
		if (send_page == (total_page-1))
		{   
			send_sta = true;
			send_page = 0xFF;//分页结束序号
		}
		code_send_buf.code_cur_page = send_page;
		if (send_page != 0xFF)
		{
			code_send_buf.code_cur_len = 44;
			memcpy(code_send_buf.code_data,&code[send_page*44],44);
		}
		else
		{
			code_send_buf.code_cur_len = Datalen%44;
			memset(code_send_buf.code_data,0,sizeof(code_send_buf.code_data));
			memcpy(code_send_buf.code_data,&code[(total_page-1)*44],code_send_buf.code_cur_len);
		}
		code_send_buf.xor = UTL_XOR_CHECK(&code_send_buf.head,sizeof(code_send_buf)-1);
		COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&code_send_buf,sizeof(code_send_buf));

		if (send_page == 0xFF)
		{
			//发完跳出循环
			send_page = total_page;
		}
		//由于dma发送报文后需要更新状态，因此这里必须加时延，避免不发包的情况
		HAL_Delay(1);
	}
	return send_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:精频码写入
*PARAMETERS:Datalen：数据个数，一个32位的精频码算一个数据；
						code：精频码数组
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Precision_Send(uint16_t Datalen,uint32_t *code)
{
	uint8_t send_page = 0;
	LASER_LSP_CODE_T code_send_buf = {0};
	uint8_t total_page = 0;
	bool send_sta = false;
	//该组字节最多1024
	if (Datalen > 1024)
	{
		Datalen = 1024;
	}

	//算总页数     
	if (0 != (Datalen/11))
	{
		total_page = (Datalen/11) + 1;
	}
	else
	{
		total_page = (Datalen/11);
	}

	//发到最后一页就不发了
	if (send_page == total_page)
	{
		send_sta = true;
		send_page = 0xFF;//分页结束序号
	}
	
	code_send_buf.head = LASER_CODE_PRECISION_CMD;
	code_send_buf.cmd_type = LASER_CODE_PRECISION_CMD;
	code_send_buf.code_class = 0;
	code_send_buf.code_len = Datalen;
	//循环发送
	for (send_page = 0;send_page < total_page;send_page++)
	{
		//当发到最后一页时，需要状态置位
		if (send_page == (total_page - 1))
		{
			send_sta = true;//发最后一帧的时候，将该标志位置为true
			send_page = 0xFF;//分页结束序号
		}
		//如果刚好是整数页，每页的数据量都是11
		if (send_page != 0xFF)
		{
			code_send_buf.code_cur_len = 11;
			memcpy(code_send_buf.code_data,&code[send_page*11],sizeof(code_send_buf.code_data));
		}
		else
		{
			//最后一阵的数据需要特殊处理
			code_send_buf.code_cur_len = Datalen%11;
			//先将最后一帧清0
			memset(code_send_buf.code_data,0,sizeof(code_send_buf.code_data));
			//再只放该放的数据进去
			memcpy(&code_send_buf.code_data[0],&code[(total_page - 1)*11],code_send_buf.code_cur_len*4);
		}
		
		code_send_buf.code_cur_page = send_page;
		
		code_send_buf.xor = UTL_XOR_CHECK(&code_send_buf.head,sizeof(code_send_buf)-1);
		
		COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&code_send_buf,sizeof(code_send_buf));

		if (send_page == 0xFF)
		{
			//跳出循环，发完了。
			send_page = total_page;
		}
		//由于dma发送报文后需要更新状态，因此这里必须加时延，避免不发包的情况
		HAL_Delay(1);
	}
	return send_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:照射查询
*PARAMETERS:code_type:0-变间隔，1-脉冲序列，2-精频码、
						code_class：组号
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_CODE_Read_Req(uint8_t code_type,uint8_t code_class)
{		
	LASER_CODE_READ_REQ_T read_code_buf = {0};
	read_code_buf.read_head = LASER_CODE_SEND_HEAD;
	if (code_type == 0)
	{
		read_code_buf.read_cmd = LASER_CODE_INTERVAL_REQ;
	}
	else if (code_type == 1)
	{
		read_code_buf.read_cmd = LASER_CODE_PULSE_REQ;
	}
	else if (code_type == 2)
	{
		read_code_buf.read_cmd = LASER_CODE_PRECISION_REQ;
	}
	read_code_buf.read_class = code_class;
	read_code_buf.res1 = 0;
	read_code_buf.res2 = 0;   
	read_code_buf.xor = UTL_XOR_CHECK(&read_code_buf.read_head,sizeof(read_code_buf)-1);
	COM_API_Send_Data(COM_LASER_IN,(uint8_t *)&read_code_buf,sizeof(read_code_buf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据接收解算
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t LASER_CODE_Recv_Handle(uint16_t data_len,uint8_t *data_in)
{
	//对写入时的回报报文进行解析。
	LASER_CODE_RSP_T code_recv_sta_buf;
	LASER_LSP_CODE_T recv_code_buf;
	uint8_t rcv_xor;
	uint32_t interval_code[3][1024] = {0};
	//判断数据长度
	if ((data_len != sizeof(code_recv_sta_buf))&&(data_len != sizeof(recv_code_buf)))
	{
		return RECV_DATA_LEN_ERR;
	}
	//校验判断
	rcv_xor = UTL_XOR_CHECK(data_in,data_len-1);
	if (data_in[data_len] != rcv_xor)
	{
		return RECV_DATA_XOR_ERR;
	}
	//写入的编码数量
	if (data_len == sizeof(code_recv_sta_buf))
	{
		uint16_t w_code_num = 0;
		memcpy(&code_recv_sta_buf,data_in,sizeof(code_recv_sta_buf));
		if (code_recv_sta_buf.rcv_sta == 0)//写入成功
		{
			code_write_sta = true;
		}
	}
	else
	{
		//对查询的报文进行解析
		memcpy(&recv_code_buf,data_in,sizeof(recv_code_buf));
		if (recv_code_buf.cmd_type == LASER_CODE_INTERVAL_REQ)//变间隔码
		{
			//解析回报的变间隔码
			memcpy(&interval_code[recv_code_buf.code_class][recv_code_buf.code_cur_page*11],
					&recv_code_buf.code_data[0],recv_code_buf.code_cur_len);//将解析到的变间隔码，解析到固定数组中去
		}
		else if (recv_code_buf.cmd_type == LASER_CODE_PULSE_REQ)//脉冲序列码
		{
			//解析回报的脉冲序列码
			memcpy(&interval_code[recv_code_buf.code_class][recv_code_buf.code_cur_page*11],
			&recv_code_buf.code_data[0],recv_code_buf.code_cur_len);//
		}  
		else if (recv_code_buf.cmd_type == LASER_CODE_PULSE_REQ)//精频码
		{
			//解析回报的精频码
			memcpy(&interval_code[recv_code_buf.code_class][recv_code_buf.code_cur_page*11],
			&recv_code_buf.code_data[0],recv_code_buf.code_cur_len);
		}
	}
	return true;
}
#endif
