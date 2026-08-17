#include "Common/base_inc.h"
#include "Common/utl_check.h"
#include "Driver/drv_uart.h"
#if BMQ_UEF072
#include "../bmq/Bmq_uef072.h"
//extern uint32_t FWBMQ_Data ;          //给编码器函数使用变量
uint8_t BMQ_TRflag_FW = 1;

uint8_t FWBMQerr;
#define BMQ_DATA_LEN 7
uint8_t CRC8Check = 0;
uint8_t bmq_err = 0;
uint8_t recv_bmq_info[10] = {0};
//函数直接放在串口回调里面调用即可
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void BMQ_API_Serial_Data_Init(void)
{
    //初始化接收数据环
    COM_Rcv_SerialPort_Init(COM_BMQ_IN,0,0,BMQ_DATA_LEN);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:编码器数据周期性解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void BMQ_Period_Handle(void)//void RX_BMQ_Data(UART_HandleTypeDef *huart,uint8_t *Data,uint8_t Size)；该函数在主函数调用
{
	uint8_t recv_len = 0;
	uint32_t BMQ_tempData = 0;
//计算CRC校验码 规定生成多项式：x8+x7+x4+x2+x1+x0--0x97
	CRC8Check = 0x00;//默认值0x00
	uint8_t *Data = NULL;
	recv_len = COM_REC_Data_Direct(COM_BMQ_IN,recv_bmq_info);//编码器串口数据获取
	if (recv_len == 0)
	{
		return;//环空返回
	}
	Data = recv_bmq_info;
	for(uint8_t i = 2;i < 6;i ++ )
	{
		CRC8Check ^= Data[i];//逐字节异或
		for(uint8_t bit = 0;bit < 8;bit ++ )
		{
			if(CRC8Check & 0x80)//检验最高位是否为1
			{
			  CRC8Check = (CRC8Check << 1) ^ 0x97;
			}
			else
			{
				CRC8Check <<= 1;
			}
		}
	}
	if (recv_bmq_info[6] == CRC8Check)
	{
		FWBMQerr = 0x00;
//		BMQ_tempData = ((Data[2] << 16) | (Data[3]<<8) |(Data[4]&0xC0));
		BMQ_tempData = ((recv_bmq_info[2] << 16) | (recv_bmq_info[3]<<8) |(recv_bmq_info[4]&0xC1));
//		FWBMQ_Data = BMQ_tempData>>5;//方位编码器分辨率19位//编码器读取值，传给俯仰或者方位角
//		fwbmqdispose();
	}
	else
	{
		bmq_err++;
	}
}

#endif
