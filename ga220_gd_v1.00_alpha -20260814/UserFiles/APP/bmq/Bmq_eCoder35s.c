#include "Common/base_inc.h"
#include "Bsp/bsp_uart.h"
#include "Bsp/bsp_timer.h"
#include "Driver/drv_uart.h"
#if BMQ_ECODER32S
//编码器
//extern uint32_t FYBMQ_Data; //变量赋值给gdbmqdispose()使用;
uint8_t GetLen = 1;
uint8_t bmq_recv_err;
uint8_t BMQ_huart_txbuf[32];
uint8_t BMQ_huart_txdata = 0x02;
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
	COM_Rcv_SerialPort_Init(COM_BMQ_IN, 0,0,0);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:编码器初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void ECODER_Device_Init(void)
{
	BMQ_huart_txbuf[0] = 0x02;
//	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);//拉高发送使能管脚;按需调整
	BMQ_API_Serial_Data_Init();
	COM_API_Send_Data(COM_BMQ_IN,BMQ_huart_txbuf,1);
}
/*==============================================================
*FUNCTION NAME:编码器数据采集   
*DISCRIPTION:使用：1ms触发式发送：标志位：BMQ_TRflag_FY；485GPIO适配
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void BMQGetValue(void)
{
	GetLen = 0;
//	HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);//拉高发送使能管脚;按需调整
	COM_API_Send_Data(COM_BMQ_IN,BMQ_huart_txbuf,1);
}
/*==============================================================
*FUNCTION NAME:20位零差云控编码器; 
*DISCRIPTION:数据解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void ecoder_r20b_process_data_in(uint8_t *Data,uint8_t Size)
{
	 GetLen = 1;
	 uint32_t BMQ_tempData = 0;
    if( Data[1] != 0x00 )
	{
		bmq_recv_err = 0x01;//FY编码器故障
		//TxMCData.FYSelfChek.FYBMQ = 0;//自检状态传递
	}
	else if (Data[0] == 0x02)
	{
		bmq_recv_err = 0x00;
		BMQ_tempData = Data[2]|(Data[3] << 8)| (Data[4]<<16);
		//FYBMQ_Data = (BMQ_tempData & 0xfffff);//编码器读取值，传给俯仰或者方位角
		//TxMCData.FYSelfChek.FYBMQ = 1;//自检状态传递
//			gdbmqdispose();
	}
}
/*==============================================================
*FUNCTION NAME:编码器数据循环解析
*DISCRIPTION:数据解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t ecoder_rx[32] = {0};
RECV_DATA_ERR_STA ECODER_API_R20B_Period_Handle(void)
{
	uint16_t ecoder_recv_len = 0;
	ecoder_recv_len = COM_REC_Data_Direct(COM_BMQ_IN,ecoder_rx);
	if (ecoder_recv_len == 0)
    {
        return RECV_DATA_NULL;
    }
	ecoder_r20b_process_data_in(ecoder_rx, ecoder_recv_len);
	return RECV_DATA_SUC;
}	
#endif



