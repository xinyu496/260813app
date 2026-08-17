#ifndef __DRV_UART_H
#define __DRV_UART_H
#include <stdint.h>
#include "main.h"
#include "Bsp/bsp_uart.h"

#define DEBUG_UART_DRIVER 0 
#define DEBUG_UART_APP 0 

typedef struct
{
	uint8_t debug_data[20];
	uint8_t debug_length;
}DEBUG_DATA_T;




//函数调用
//直接接收数据
uint16_t COM_REC_Data_Direct(COM_TYPE_E com_id, uint8_t *arry);
//使用通用协议接收数据
uint16_t COM_REC_DataAnalysis(COM_TYPE_E com_id, uint8_t *arry);
//一头一尾接收数据
uint16_t COM_REC_DataAnalysis_nocheck(COM_TYPE_E com_id, uint8_t *arry);
//一头零尾异或校验（定长帧；如陀螺 13 字节，byte[1~11] 异或 = byte[12]）
uint16_t COM_REC_DataAnalysis_1head_xor(COM_TYPE_E com_id, uint8_t *arry);
//一头零尾累加校验（定长或变长；如图像/方位 0xEB 帧，累加和低 8 位）
uint16_t COM_REC_DataAnalysis_1head_accu(COM_TYPE_E com_id, uint8_t *arry);

//接收数据头尾长度初始化
void COM_Rcv_SerialPort_Init(COM_TYPE_E com_id, uint16_t head, uint16_t tail, uint16_t len);

//串口数据发送超时(用于ms中断回调，3ms未发送完成清除标志位)
void COM_API_Send_Overtime_Isr(void);


uint16_t COM_REC_DataAnalysis_DriverBoard(COM_TYPE_E com_id, uint8_t *arry);//无刷驱动板解析

DEBUG_DATA_T *COM_REC_Debug_Data(void);
#endif

