/**
  ******************************************************************************
  * @file           : hw_uart.c
  * @brief          : This file provides code for the configuration
  *          		  of the USART instances.
  ******************************************************************************
  * @attention
  *
  * @Copyright 		(c) Sichuan Zhongke Youcheng Technology Co.,Ltd.
  * @Author			: wangbao
  * @Version		: 1.0
  * @Date			: 2025.10.09
  * @History:
  *     +------------+---------------------------------------------------------+
  *		| 2025.10.09 |	创建文件,完成基本功能
  *     +------------+---------------------------------------------------------+
  ******************************************************************************
  */

#include "Driver/drv_uart.h"
#include <string.h>
#include <stdbool.h>
#include "Common/utl_check.h"
#include "Bsp/bsp_timer.h"
#include "Bsp/SEGGER_RTT.h"
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取接收缓冲环长度
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint16_t RINGBUFF_GetRecLen(COM_PORT_REC_QUEUE_T *STRUCT)
{
    if(STRUCT == NULL)
    {
        return 0;
    }

    return (STRUCT->write_index + STRUCT->buffer_size - STRUCT->read_index) % STRUCT->buffer_size;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取发送缓冲环长度
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t RINGBUFF_GetSendLen(COM_PORT_REC_QUEUE_T *STRUCT)
{
    if(STRUCT == NULL)
    {
        return 0;
    }

    return (STRUCT->twrite_index + STRUCT->buffer_size - STRUCT->tread_index) % STRUCT->buffer_size;
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取缓冲环下标数据
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint8_t RINGBUFF_GET_InDexData(COM_PORT_REC_QUEUE_T *STRUCT, unsigned short int index)
{    
	if(STRUCT == NULL)
    {
        return 0;
    }
    return STRUCT->buffer_ptr[index%STRUCT->buffer_size];//超过ringbuffer的部分，折回起始地址
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:环读指针移动一个距离
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint8_t RINGBUFF_READ_INDEX_Move(COM_PORT_REC_QUEUE_T *STRUCT, unsigned short int len)
{
	if(STRUCT == NULL)
    {
        return 0;
    }
    if (RINGBUFF_GetRecLen(STRUCT) < len)//判断环形缓冲区已有数据长度是否够偏移
    {
        STRUCT->read_index = (STRUCT->read_index + len) % STRUCT->buffer_size;
        return 0;
    }
    else
    {
        STRUCT->read_index = (STRUCT->read_index + len) % STRUCT->buffer_size;
        return 1;
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取帧数据区
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void RINGBUFF_READ_All(COM_PORT_REC_QUEUE_T *STRUCT, uint8_t *pData)
{
    uint16_t size = 12;
    int i = 0;

    if(STRUCT == NULL)
    {
        return;
    }
    for(i = 0; i < size; i++)
    {
        *(pData+i) = STRUCT->buffer_ptr[STRUCT->read_index];
        STRUCT->read_index = (STRUCT->read_index+1)%STRUCT->buffer_size;
    }

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口帧处理（适用于公司内部协议模板）
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t COM_REC_DataAnalysis(COM_TYPE_E com_id, uint8_t *arry)
{
    //获取接收句柄
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
    STRUCT = COM_Rec_Get_Com(com_id);
    uint16_t total_len = 0;
	
	if (com_id >= COM_USART_END)
	{
		return 0;
	}
    while(1)
    {
        if (RINGBUFF_GetRecLen(STRUCT) < 7) //环空返回
        {
            return 0;
        }

        if ((RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index) != STRUCT->head_h)			//包头校验 找不到包头移动读指针
                ||(RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index + 1) != STRUCT->head_l))
        {
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }
        //获取数据长度，该指令长度为数据内容长度；不包含头，校验，长度；
        uint16_t cmd_length	= (uint16_t)((RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + 2) << 8) | 
											RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + 3));
        if(RINGBUFF_GetRecLen(STRUCT) < cmd_length)
        {
            return 0;
        }

        // 如果校验和不正确 则跳过 重新开始寻找
        uint16_t s_check_sum = 0;
        total_len = RINGBUFF_GetRecLen(STRUCT);
        for (uint16_t i = 0; i < total_len; i++)
        {
            s_check_sum += RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + i);//从头开始校验
        }
        //校验判断;校验不对则重新在找头
        if (s_check_sum != RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + total_len - 2))
        {
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }
        RINGBUFF_READ_All(STRUCT,arry);

        return (uint16_t)((RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + 2) << 8) | RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + 3));
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取帧数据区
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void RINGBUFF_READ_All_nocheck(COM_PORT_REC_QUEUE_T *STRUCT, uint8_t *pData, uint16_t len)
{
    uint16_t size = len;
    int i = 0;
	
    if(STRUCT == NULL)
    {
        return;
    }
    for(i = 0; i < size; i++)
    {
        *(pData+i) = STRUCT->buffer_ptr[STRUCT->read_index];
        STRUCT->read_index = (STRUCT->read_index+1)%STRUCT->buffer_size;//移动读指针到写指针
    }
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
static void RINGBUFF_READ_All_DMA(COM_PORT_REC_QUEUE_T *STRUCT, uint8_t *pData )
{
    uint16_t size;
    int i = 0;

    if(STRUCT == NULL)
    {
        return;
    }
    size = RINGBUFF_GetRecLen(STRUCT);//STRUCT->write_index - STRUCT->read_index;

    for(i = 0; i < size; i++)
    {
        *(pData+i) = STRUCT->buffer_ptr[STRUCT->read_index];
        STRUCT->read_index = (STRUCT->read_index+1)%STRUCT->buffer_size;//移动读指针到写指针
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:增加透传处理机制，在从环中取数据调用
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t combypass = true;
uint8_t compypass_src_com = 2;
uint8_t compypass_dst_com = 1;
void COM_Bypass_Handle(COM_TYPE_E com_id, uint8_t *arry,uint16_t size)
{
	//透传控制,无差别透传
	if(combypass)//打开透传
	{
		//主循环中调用，当需要透传的数据从源串口数据输入时，才执行透传。
		if (com_id == compypass_src_com)
		{
			//数据来了之后，直接向透传的串口输出数据。
			COM_API_Send_Data(COM_USART_1,arry,size);
		}
		else if (com_id == compypass_dst_com)//如果输入的数据是透传的串口
		{
			//直接转发给源串口
			COM_API_Send_Data(COM_USART_2,arry,size);
		}
	}
	//区别透传，根据技术协议，识别透传指令
	if ((arry[4] == 0x77)&&(compypass_src_com == com_id))//说明该帧是透传指令
	{
		COM_API_Send_Data(COM_USART_1,&arry[5],size-7);
	}
	else if (compypass_dst_com == com_id)		//透传口返回的指令
	{
		COM_API_Send_Data(COM_USART_1,arry,size);
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:无需任何处理，直接获取缓冲区中的数据;长度是DMA接收长度
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
DEBUG_DATA_T debug_cache = {0};
uint16_t COM_REC_Data_Direct(COM_TYPE_E com_id, uint8_t *arry)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	if (com_id >= COM_USART_END)
	{
		return 0;
	}
    STRUCT = COM_Rec_Get_Com(com_id);
	
	if(STRUCT == NULL)
    {
        return 0;
    }
	uint16_t size = RINGBUFF_GetRecLen(STRUCT);

	if (size < 3) //环空返回
	{
		return 0;
	}

	RINGBUFF_READ_All_DMA(STRUCT,arry);
#if CLI_INCLUDE
	if (com_id == COM_DEBUG_ON)
	{
		memset(debug_cache.debug_data,0,20);
		memcpy(debug_cache.debug_data,arry,size);
		debug_cache.debug_length = size;
	}
#endif
	return size;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取调试口数据
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
DEBUG_DATA_T *COM_REC_Debug_Data(void)
{
	return &debug_cache;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口帧处理（适用于1包头 1包尾 ）;数据定长（最小长度即可）
*PARAMETERS:STRUCT：STRUCT：串口句柄   arry：传出数据区指针  len:传入数据长度
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t COM_REC_DataAnalysis_nocheck(COM_TYPE_E com_id, uint8_t *arry)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	
	if (com_id >= COM_USART_END)
	{
		return 0;
	}
    STRUCT = COM_Rec_Get_Com(com_id);

    while(1)
    {
        if (RINGBUFF_GetRecLen(STRUCT) < 3) //环空返回
        {
            return 0;
        }

        if (RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index) != STRUCT->head_l)		//包头校验 仅用低8位
        {
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }
        uint16_t cmd_length	= STRUCT->total_len;//获取指令长度
        uint16_t rcv_length = RINGBUFF_GetRecLen(STRUCT);//获取接收的数据总长度

        if(rcv_length < cmd_length) //总长度小于初始化数据长度
        {
            return 0;
        }

        uint8_t tail;
        tail = RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index + STRUCT->total_len);//包尾
        if (tail != STRUCT->tail_l)
        {   //包尾校验 仅用低8位
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }

        RINGBUFF_READ_All_nocheck(STRUCT,arry,rcv_length);
				
        return STRUCT->total_len;
		
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:一头零尾累加校验（定长）有数据长度
*PARAMETERS:STRUCT：com_id：串口号  arry：传出数据区指针  
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t check_errror_num;
uint16_t COM_REC_DataAnalysis_1head_accu(COM_TYPE_E com_id, uint8_t *arry)
{
	COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	
	if (com_id >= COM_USART_END)
	{
		return 0;
	}
  STRUCT = COM_Rec_Get_Com(com_id);
	while(1)
  {
		if (RINGBUFF_GetRecLen(STRUCT) < 3) //环空返回
		{
				return 0;
		}
				
		if (RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index) != STRUCT->head_l)		//包头校验 仅用低8位
    {
        RINGBUFF_READ_INDEX_Move(STRUCT, 1);
        continue;
    }
		uint16_t cmd_length = STRUCT->buffer_ptr[(STRUCT->read_index + 1 ) % 128];
		uint16_t rcv_length = RINGBUFF_GetRecLen(STRUCT);//获取接收的数据总长度
		if((rcv_length < cmd_length) || (rcv_length > 120))//总长度小于初始化数据长度
    {
        return 0;
    }
		
		uint8_t s_check_add = 0;
		uint8_t buff[128];
		for(char ii = 0 ; ii < cmd_length - 1 ; ii++)
		{
			buff[ii] = STRUCT->buffer_ptr[(STRUCT->read_index + ii) % 128];
		}
		s_check_add = UTL_ADD_CHECK(buff , cmd_length - 1);
		//校验判断;校验不对则重新在找头
    if (s_check_add != RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + cmd_length - 1))
    {
			check_errror_num++;
#if DEBUG_UART_DRIVER		
		SEGGER_RTT_SetTerminal(1);
		SEGGER_RTT_printf(0 , "error : uart receive check fault\n");
		SEGGER_RTT_SetTerminal(0);
#endif			
        RINGBUFF_READ_INDEX_Move(STRUCT, 1);
        continue;
    }
		
		RINGBUFF_READ_All_nocheck(STRUCT,arry,cmd_length);
#if DEBUG_UART_DRIVER		
		SEGGER_RTT_printf(0 , "tim is %d " , User_Tick.EthSend);
		for(char c = 0 ; c < cmd_length ; c++)
		{
			SEGGER_RTT_printf(0 , "0x%x " , arry[c]);
		}
		SEGGER_RTT_printf(0 , "\n");
#endif		
		return 1;
	}
	
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:一头零尾累加校验（定长）（无数据长度）
*PARAMETERS:STRUCT：com_id：串口号  arry：传出数据区指针  
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t COM_REC_DataAnalysis_DriverBoard(COM_TYPE_E com_id, uint8_t *arry)
{
	COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	
	if (com_id >= COM_USART_END)
	{
		return 0;
	}
  STRUCT = COM_Rec_Get_Com(com_id);
	while(1)
  {
		if (RINGBUFF_GetRecLen(STRUCT) < 3) //环空返回
		{
				return 0;
		}
				
		if (RINGBUFF_GET_InDexData(STRUCT,STRUCT->read_index) != STRUCT->head_l)		//包头校验 仅用低8位
    {
        RINGBUFF_READ_INDEX_Move(STRUCT, 1);
        continue;
    }
		uint16_t cmd_length = 8;
		uint16_t rcv_length = RINGBUFF_GetRecLen(STRUCT);//获取接收的数据总长度
		if((rcv_length < cmd_length) || (rcv_length > 120))//总长度小于初始化数据长度
    {
        return 0;
    }
		
		uint8_t s_check_add = 0;
		uint8_t buff[128];
		for(char ii = 0 ; ii < cmd_length - 1 ; ii++)
		{
			buff[ii] = STRUCT->buffer_ptr[(STRUCT->read_index + ii + 1) % 128];
		}
		s_check_add = UTL_ADD_CHECK(buff , cmd_length - 2);
		//校验判断;校验不对则重新在找头
    if (s_check_add != RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + cmd_length - 1))
    {
			check_errror_num++;
#if DEBUG_UART_DRIVER		
		SEGGER_RTT_SetTerminal(1);
		SEGGER_RTT_printf(0 , "error : uart receive check fault\n");
		SEGGER_RTT_SetTerminal(0);
#endif			
        RINGBUFF_READ_INDEX_Move(STRUCT, 1);
        continue;
    }
		
		RINGBUFF_READ_All_nocheck(STRUCT,arry,cmd_length);
#if DEBUG_UART_DRIVER		
		SEGGER_RTT_printf(0 , "tim is %d " , User_Tick.EthSend);
		for(char c = 0 ; c < cmd_length ; c++)
		{
			SEGGER_RTT_printf(0 , "0x%x " , arry[c]);
		}
		SEGGER_RTT_printf(0 , "\n");
#endif		
		return 1;
	}
	
}




/*==============================================================
 * FUNCTION NAME : COM_REC_DataAnalysis_1head_xor
 * DISCRIPTION   : 单帧头、定长、异或校验的串口帧解析
 *
 * 适用协议：仅帧头 + 定长数据 + 末字节异或校验（无固定包尾）
 * 典型用途：陀螺上报帧（13 字节，帧头 0xA2，COM_GD_GYRO / USART6）
 *
 * 使用前须调用 COM_Rcv_SerialPort_Init(com_id, head, 0, frame_len)：
 *   - head      : 帧头低 8 位（如 0xA2）
 *   - tail      : 传 0（最后一字节是异或校验，不是固定包尾）
 *   - frame_len : 整帧字节数（如 13）
 *
 * 校验规则：byte[frame_len-1] = byte[1] ^ byte[2] ^ … ^ byte[frame_len-2]
 *           即帧头 byte[0] 不参与异或，末字节不参与自身异或
 *
 * PARAMETERS    : com_id  串口编号
 *                 arry    输出缓冲区，长度 >= total_len
 * RETURN        : 0=未取到完整有效帧；非 0=成功，返回值为帧长度
 * NOTES         : 校验失败时读指针前移 1 字节重新找帧头
 *==============================================================*/
uint16_t COM_REC_DataAnalysis_1head_xor(COM_TYPE_E com_id, uint8_t *arry)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
    uint16_t cmd_length;
    uint16_t ii;
    uint8_t xor_check;

    if (com_id >= COM_USART_END || arry == NULL) {
        return 0;
    }

    STRUCT = COM_Rec_Get_Com(com_id);
    if (STRUCT == NULL || STRUCT->total_len < 3U) {
        return 0;
    }

    cmd_length = STRUCT->total_len;

    while (1) {
        if (RINGBUFF_GetRecLen(STRUCT) < cmd_length) {
            return 0;
        }

        if (RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index) != STRUCT->head_l) {
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }

        /* 异或校验：byte[1] ~ byte[cmd_length-2]，结果应等于 byte[cmd_length-1] */
        xor_check = 0U;
        for (ii = 1U; ii < (cmd_length - 1U); ii++) {
            xor_check ^= RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + ii);
        }

        if (xor_check != RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + cmd_length - 1U)) {
            RINGBUFF_READ_INDEX_Move(STRUCT, 1);
            continue;
        }

        /* tail=0 时跳过包尾检查；tail!=0 时末字节须等于 tail_l（与异或校验互斥，一般不用） */
        if ((STRUCT->tail_l != 0U) || (STRUCT->tail_h != 0U)) {
            if (RINGBUFF_GET_InDexData(STRUCT, STRUCT->read_index + cmd_length - 1U) != STRUCT->tail_l) {
                RINGBUFF_READ_INDEX_Move(STRUCT, 1);
                continue;
            }
        }

        RINGBUFF_READ_All_nocheck(STRUCT, arry, cmd_length);
        return cmd_length;
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据发送超时(用于ms中断回调，3ms未发送完成清除标志位)
*PARAMETERS:com_id:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_API_Send_Overtime_op(COM_PORT_REC_QUEUE_T * pstruct)
{
	if(pstruct->tflag == 1)
    {
        pstruct->overtime_cnt++;
    }
    if(pstruct->overtime_cnt >= 3)
    {
        pstruct->overtime_cnt = 0;
        pstruct->tflag = 0;
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据发送超时(用于ms中断回调，3ms未发送完成清除标志位)
*PARAMETERS:com_id:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_API_Send_Overtime_Isr(void)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;

#if USART1_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_1);
    COM_API_Send_Overtime_op(STRUCT);
#endif

#if USART2_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_2);
    COM_API_Send_Overtime_op(STRUCT);
#endif

#if USART3_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_3);
    COM_API_Send_Overtime_op(STRUCT);
#endif

#if USART4_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_4);
    COM_API_Send_Overtime_op(STRUCT);
#endif

#if USART5_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_5);
    COM_API_Send_Overtime_op(STRUCT);
#endif

#if USART6_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_6);
    COM_API_Send_Overtime_op(STRUCT);
#endif
	
#if USART8_ENABLE
    STRUCT = COM_Rec_Get_Com(COM_USART_8);
    COM_API_Send_Overtime_op(STRUCT);
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口接收环初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_Rcv_SerialPort_Init(COM_TYPE_E com_id, uint16_t head, uint16_t tail, uint16_t len)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	
		if (com_id >= COM_USART_END)
		{
			return;
		}
    STRUCT = COM_Rec_Get_Com(com_id);
    if(STRUCT == NULL)
    {
        return;
    }

    STRUCT->head_h = (uint8_t)(head>>8);
    STRUCT->head_l = (uint8_t)head;
    STRUCT->tail_h = (uint8_t)(tail>>8);
    STRUCT->tail_l = (uint8_t)tail;
    STRUCT->total_len = len;
    STRUCT->tflag = 0;
}



