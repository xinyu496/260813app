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
#include "Bsp/bsp_uart.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#if defined (STM32H743xx)||(STM32H7A3xx)
#include "usart.h"
#endif
/*串口发送超时*/
#define 	MAX_SERIAL_TIMEOUT  	(100)
#define COM_UART_RX_DMA_ABORT_TIMEOUT_MS  (10U)

volatile uint32_t com_rx_soft_recover_cnt;
volatile uint32_t com_rx_hard_recover_cnt;
volatile uint32_t com_rx_hard_recover_fail_cnt;
volatile uint32_t com_rx_dma_abort_fail_cnt;
volatile uint32_t com_tx_skip_cnt;
volatile uint32_t com_tx_start_fail_cnt;
volatile uint32_t com_tx_recover_cnt;

static void COM_Uart_EnableIdleInterrupt(UART_HandleTypeDef *huart);
static void COM_Uart_ClearHwErrorFlags(UART_HandleTypeDef *huart);
static bool COM_Uart_IsRxDmaRunning(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef COM_Uart_RxDmaHardRecover(UART_HandleTypeDef *huart, uint8_t *rcv_arry,
                                                   uint16_t rcv_len);
static void COM_Uart_TxHardRecover(UART_HandleTypeDef *huart);

/*************************定义接收内存************************************/
#if 1
    COM_PORT_REC_QUEUE_T	usart1_port_rec_handle = {0};
    uint8_t uart1_rx_array[USART1_RING_BUFF_SIZE];
    uint8_t uart1_tx_array[USART1_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart2_port_rec_handle = {0};
    uint8_t uart2_rx_array[USART2_RING_BUFF_SIZE];
    uint8_t uart2_tx_array[USART2_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart3_port_rec_handle = {0};
    uint8_t uart3_rx_array[USART3_RING_BUFF_SIZE];
    uint8_t uart3_tx_array[USART3_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart4_port_rec_handle = {0};
    uint8_t uart4_rx_array[USART4_RING_BUFF_SIZE];
    uint8_t uart4_tx_array[USART4_RING_BUFF_SIZE];
#endif
#if 1
    COM_PORT_REC_QUEUE_T	usart5_port_rec_handle = {0};
    uint8_t uart5_rx_array[USART5_RING_BUFF_SIZE];
    uint8_t uart5_tx_array[USART5_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart6_port_rec_handle = {0};
    uint8_t uart6_rx_array[USART6_RING_BUFF_SIZE];
    uint8_t uart6_tx_array[USART6_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart7_port_rec_handle;
    uint8_t uart7_rx_array[USART7_RING_BUFF_SIZE];
    uint8_t uart7_tx_array[USART7_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	usart8_port_rec_handle = {0};
    uint8_t uart8_rx_array[USART8_RING_BUFF_SIZE];
    uint8_t uart8_tx_array[USART8_RING_BUFF_SIZE];
#endif

#if 1
    COM_PORT_REC_QUEUE_T	lpusart1_port_rec_handle = {0};
    uint8_t lpusart1_rx_array[LPUART1_RING_BUFF_SIZE];
    uint8_t lpusart1_tx_array[LPUART1_RING_BUFF_SIZE];
#endif

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口初始化，启动DMA
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_DRV_SerialPort_Init(void)
{
#if USART1_ENABLE
//	HAL_NVIC_EnableIRQ(USART1_IRQn);
//	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, uart1_rx_array, sizeof(uart1_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart1);
    //串口接受句柄初始化
    usart1_port_rec_handle.buffer_size = USART1_RING_BUFF_SIZE;
    usart1_port_rec_handle.buffer_ptr = uart1_rx_array;
    usart1_port_rec_handle.tbuffer_ptr = uart1_tx_array;
#endif
#if USART2_ENABLE
    HAL_UART_Receive_DMA(&huart2, uart2_rx_array, sizeof(uart2_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart2);

    usart2_port_rec_handle.buffer_size = USART2_RING_BUFF_SIZE;
    usart2_port_rec_handle.buffer_ptr = uart2_rx_array;
    usart2_port_rec_handle.tbuffer_ptr = uart2_tx_array;
#endif
#if USART3_ENABLE
    HAL_UART_Receive_DMA(&huart3, uart3_rx_array, sizeof(uart3_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart3);

    usart3_port_rec_handle.buffer_size = USART3_RING_BUFF_SIZE;
    usart3_port_rec_handle.buffer_ptr = uart3_rx_array;
    usart3_port_rec_handle.tbuffer_ptr = uart3_tx_array;
#endif
#if USART4_ENABLE
    HAL_UART_Receive_DMA(&huart4, uart4_rx_array, sizeof(uart4_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart4);

    usart4_port_rec_handle.buffer_size = USART4_RING_BUFF_SIZE;
    usart4_port_rec_handle.buffer_ptr = uart4_rx_array;
    usart4_port_rec_handle.tbuffer_ptr = uart4_tx_array;
#endif
#if USART5_ENABLE
    HAL_UART_Receive_DMA(&huart5, uart5_rx_array, sizeof(uart5_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart5);

    usart5_port_rec_handle.buffer_size = USART5_RING_BUFF_SIZE;
    usart5_port_rec_handle.buffer_ptr = uart5_rx_array;
    usart5_port_rec_handle.tbuffer_ptr = uart5_tx_array;
#endif
#if USART6_ENABLE
    HAL_UART_Receive_DMA(&huart6, uart6_rx_array, sizeof(uart6_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart6);

    usart6_port_rec_handle.buffer_size = USART6_RING_BUFF_SIZE;
    usart6_port_rec_handle.buffer_ptr = uart6_rx_array;
    usart6_port_rec_handle.tbuffer_ptr = uart6_tx_array;
#endif
#if USART7_ENABLE
    HAL_UART_Receive_DMA(&huart7, uart7_rx_array, sizeof(uart7_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart7);

    usart7_port_rec_handle.buffer_size = USART7_RING_BUFF_SIZE;
    usart7_port_rec_handle.buffer_ptr = uart7_rx_array;
    usart7port_rec_handle.tbuffer_ptr = uart7_tx_array;
#endif
#if USART8_ENABLE
    HAL_UART_Receive_DMA(&huart8, uart8_rx_array, sizeof(uart8_rx_array));
    COM_Uart_EnableIdleInterrupt(&huart8);
	usart8_port_rec_handle.buffer_size = USART8_RING_BUFF_SIZE;
	usart8_port_rec_handle.buffer_ptr = uart8_rx_array;
	usart8_port_rec_handle.tbuffer_ptr = uart8_tx_array;
#endif

#if LPUART1_ENABLE
    HAL_UART_Receive_DMA(&hlpuart1, lpusart1_rx_array, sizeof(lpusart1_rx_array));
    COM_Uart_EnableIdleInterrupt(&hlpuart1);
	lpusart1_port_rec_handle.buffer_size = USART8_RING_BUFF_SIZE;
	lpusart1_port_rec_handle.buffer_ptr = lpusart1_rx_array;
	lpusart1_port_rec_handle.tbuffer_ptr = lpusart1_tx_array;
#endif

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口句柄对应
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
COM_PORT_REC_QUEUE_T *COM_Rec_Get_Com(COM_TYPE_E com)
{
    COM_TYPE_E port = com;
    COM_PORT_REC_QUEUE_T *recv_handle = NULL;

    switch (port)
    {
#if USART1_ENABLE
        case COM_USART_1:
            recv_handle = &usart1_port_rec_handle;
            break;
#endif
#if USART2_ENABLE

        case COM_USART_2:
            recv_handle = &usart2_port_rec_handle;
            break;
#endif
#if USART3_ENABLE

        case COM_USART_3:
            recv_handle = &usart3_port_rec_handle;
            break;
#endif
#if USART4_ENABLE

        case COM_USART_4:
            recv_handle = &usart4_port_rec_handle;
            break;
#endif
#if USART5_ENABLE

        case COM_USART_5:
            recv_handle = &usart5_port_rec_handle;
            break;
#endif
#if USART6_ENABLE
        case COM_USART_6:
            recv_handle = &usart6_port_rec_handle;
            break;
#endif
		
#if USART7_ENABLE
        case COM_USART_7:
            recv_handle = &usart7_port_rec_handle;
            break;
#endif
		
#if USART8_ENABLE
        case COM_USART_8:
            recv_handle = &usart8_port_rec_handle;
            break;
#endif
		default:
			recv_handle = NULL;
			break;
    }

    return recv_handle;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:中断服务函数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Recv_IT_Handle(UART_HandleTypeDef *huart, COM_PORT_REC_QUEUE_T *STRUCT)
{
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
	{
		
		STRUCT->write_index = STRUCT->buffer_size-__HAL_DMA_GET_COUNTER(huart->hdmarx);//指针赋值
		STRUCT->dbg_intr_cnt++;//中断计数
		STRUCT->communi_err_cnt = 0;
		__HAL_UART_CLEAR_IDLEFLAG(huart);
	}
}

void COM_Serial_Recv_IT(COM_TYPE_E com_id)
{
#if USART1_ENABLE
    if (com_id == COM_USART_1)
    { 
		COM_Recv_IT_Handle(&huart1,&usart1_port_rec_handle);
    }
#endif
	
#if USART2_ENABLE
    if (com_id == COM_USART_2)
    {
		COM_Recv_IT_Handle(&huart2,&usart2_port_rec_handle);
    }

#endif
	
#if USART3_ENABLE
    if (com_id == COM_USART_3)
    {
		COM_Recv_IT_Handle(&huart3,&usart3_port_rec_handle);
    }
#endif

#if USART4_ENABLE
    if (com_id == COM_USART_4)
    {
		COM_Recv_IT_Handle(&huart4,&usart4_port_rec_handle);
    }
#endif

#if USART5_ENABLE
    if (com_id == COM_USART_5)
    {
		COM_Recv_IT_Handle(&huart5,&usart5_port_rec_handle);
    }
#endif

#if USART6_ENABLE
    if (com_id == COM_USART_6)
    {
		COM_Recv_IT_Handle(&huart6,&usart6_port_rec_handle);
    }
#endif

#if USART7_ENABLE
    if (com_id == COM_USART_7)
    {
		COM_Recv_IT_Handle(&huart7,&usart7_port_rec_handle);
    }
#endif

#if USART8_ENABLE
    if (com_id == COM_USART_8)
    {
		COM_Recv_IT_Handle(&huart8,&usart8_port_rec_handle);
    }
#endif

#if LPUART1_ENABLE
    if (com_id == LPUART1_ENABLE)
    {
		COM_Recv_IT_Handle(&hlpuart1,&lpusart1_port_rec_handle);
    }
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:使能 IDLE 中断
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Uart_EnableIdleInterrupt(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:清除 UART 硬件错误标志
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Uart_ClearHwErrorFlags(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

#if defined(STM32F4)
    volatile uint32_t temp = huart->Instance->SR;
    temp = huart->Instance->DR;
    (void)temp;
#elif defined(STM32H7)
    volatile uint32_t temp = huart->Instance->ISR;
    temp = huart->Instance->RDR;
    (void)temp;
#endif

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:判断 Circular RX DMA 是否仍在运行
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static bool COM_Uart_IsRxDmaRunning(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance == NULL || huart->hdmarx == NULL)
    {
        return false;
    }

    if (huart->RxState != HAL_UART_STATE_BUSY_RX)
    {
        return false;
    }

    if (READ_BIT(huart->Instance->CR3, USART_CR3_DMAR) == 0U)
    {
        return false;
    }

    if (READ_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_EN) == 0U)
    {
        return false;
    }

    if (huart->hdmarx->State != HAL_DMA_STATE_BUSY)
    {
        return false;
    }

    return true;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:RX DMA 硬恢复：Abort 到 READY 后再启动 Circular 接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static HAL_StatusTypeDef COM_Uart_RxDmaHardRecover(UART_HandleTypeDef *huart, uint8_t *rcv_arry,
                                                   uint16_t rcv_len)
{
    HAL_StatusTypeDef status;
    uint32_t tickstart;

    if (huart == NULL || huart->Instance == NULL || rcv_arry == NULL || rcv_len == 0U)
    {
        return HAL_ERROR;
    }

    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    if (huart->hdmarx != NULL)
    {
        if (huart->hdmarx->State == HAL_DMA_STATE_BUSY)
        {
            status = HAL_DMA_Abort(huart->hdmarx);
            if (status != HAL_OK)
            {
                com_rx_dma_abort_fail_cnt++;
            }
        }
        else if (READ_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_EN) != 0U)
        {
            __HAL_DMA_DISABLE(huart->hdmarx);
        }

        tickstart = HAL_GetTick();
        while (READ_BIT(huart->hdmarx->Instance->CR, DMA_SxCR_EN) != 0U)
        {
            if ((HAL_GetTick() - tickstart) > COM_UART_RX_DMA_ABORT_TIMEOUT_MS)
            {
                com_rx_dma_abort_fail_cnt++;
                break;
            }
        }

        if (huart->hdmarx->Lock == HAL_LOCKED)
        {
            __HAL_UNLOCK(huart->hdmarx);
        }

        if (huart->hdmarx->State != HAL_DMA_STATE_READY)
        {
            huart->hdmarx->State = HAL_DMA_STATE_READY;
        }
    }

    huart->RxState = HAL_UART_STATE_READY;

    COM_Uart_ClearHwErrorFlags(huart);

    status = HAL_UART_Receive_DMA(huart, rcv_arry, rcv_len);
    if (status != HAL_OK)
    {
        com_rx_hard_recover_fail_cnt++;
        return status;
    }

    COM_Uart_EnableIdleInterrupt(huart);

    if (READ_BIT(huart->Instance->CR3, USART_CR3_DMAR) == 0U)
    {
        com_rx_hard_recover_fail_cnt++;
        return HAL_ERROR;
    }

    com_rx_hard_recover_cnt++;
    return HAL_OK;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:中断错误回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Flag_Clear(UART_HandleTypeDef *huart,uint8_t *rcv_arry,uint16_t rcv_len)
{
    if (huart == NULL || rcv_arry == NULL || rcv_len == 0U)
    {
        return;
    }

    COM_Uart_ClearHwErrorFlags(huart);

    if (COM_Uart_IsRxDmaRunning(huart))
    {
        COM_Uart_EnableIdleInterrupt(huart);
        com_rx_soft_recover_cnt++;
        return;
    }

    COM_Uart_RxDmaHardRecover(huart, rcv_arry, rcv_len);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:TX DMA 硬恢复
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Uart_TxHardRecover(UART_HandleTypeDef *huart)
{
    uint32_t tickstart;

    if (huart == NULL || huart->Instance == NULL)
    {
        return;
    }

    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAT);

    if (huart->hdmatx != NULL)
    {
        if (huart->hdmatx->State == HAL_DMA_STATE_BUSY)
        {
            (void)HAL_DMA_Abort(huart->hdmatx);
        }
        else if (READ_BIT(huart->hdmatx->Instance->CR, DMA_SxCR_EN) != 0U)
        {
            __HAL_DMA_DISABLE(huart->hdmatx);
        }

        tickstart = HAL_GetTick();
        while (READ_BIT(huart->hdmatx->Instance->CR, DMA_SxCR_EN) != 0U)
        {
            if ((HAL_GetTick() - tickstart) > COM_UART_RX_DMA_ABORT_TIMEOUT_MS)
            {
                break;
            }
        }

        if (huart->hdmatx->Lock == HAL_LOCKED)
        {
            __HAL_UNLOCK(huart->hdmatx);
        }

        if (huart->hdmatx->State != HAL_DMA_STATE_READY)
        {
            huart->hdmatx->State = HAL_DMA_STATE_READY;
        }
    }

    __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);
    huart->gState = HAL_UART_STATE_READY;
    com_tx_recover_cnt++;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:中断错误回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t err_recv_in = 0;
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	err_recv_in++;
#if USART1_ENABLE
    if(huart->Instance == USART1)
    {
		usart1_port_rec_handle.recv_callback_cnt++;
		COM_Flag_Clear(&huart1, uart1_rx_array, sizeof(uart1_rx_array));
    }
#endif

#if USART2_ENABLE
    if(huart->Instance == USART2)
    {
		usart2_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart2, uart2_rx_array, sizeof(uart2_rx_array));
    }
#endif

#if USART3_ENABLE
    if(huart->Instance == USART3)
    {
		usart3_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart3, uart3_rx_array, sizeof(uart3_rx_array));
    }
#endif

#if USART4_ENABLE
    if(huart->Instance == UART4)
    {
		usart4_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart4, uart4_rx_array, sizeof(uart4_rx_array));
    }
#endif

#if USART5_ENABLE
    if(huart->Instance == UART5)
    {
		usart5_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart5, uart5_rx_array, sizeof(uart5_rx_array));
    }
#endif

#if USART6_ENABLE
    if(huart->Instance == USART6)
    {
		usart6_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart6, uart6_rx_array, sizeof(uart6_rx_array));
    }
#endif

#if USART7_ENABLE
    if(huart->Instance == USART7)
    {
		usart7_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart7, uart7_rx_array, sizeof(uart7_rx_array));
    }
#endif

#if USART8_ENABLE
    if(huart->Instance == UART8)
    {
		usart8_port_rec_handle.recv_callback_cnt++;
        COM_Flag_Clear(&huart8, uart8_rx_array, sizeof(uart8_rx_array));
    }
#endif
}

uint16_t Err_Callback_recv(void)
{
	return err_recv_in;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据发送
*PARAMETERS:com_id:串口号  size：指令总长
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Send_Data_Handle(UART_HandleTypeDef *huart, COM_PORT_REC_QUEUE_T *STRUCT, uint16_t size)
{
    if (huart == NULL || STRUCT == NULL || size == 0U)
    {
        return;
    }

    if (huart->gState != HAL_UART_STATE_READY)
    {
        com_tx_skip_cnt++;
        return;
    }

    STRUCT->tflag = 1;
    STRUCT->overtime_cnt = 0;

    if (HAL_UART_Transmit_DMA(huart, STRUCT->tbuffer_ptr, size) != HAL_OK)
    {
        com_tx_start_fail_cnt++;
        STRUCT->tflag = 0;
        COM_Uart_TxHardRecover(huart);
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据发送
*PARAMETERS:com_id:串口号  size：指令总长
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_API_Send_Data(COM_TYPE_E com_id, uint8_t *tx_ptr, uint16_t size)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
    STRUCT = COM_Rec_Get_Com(com_id);
    STRUCT->tbuffer_ptr = tx_ptr;
	
	if (com_id >= COM_USART_END)
	{
		return;
	}
	
	if (size == 0)
	{
		return;
	}

    switch(com_id)
    {
        case COM_USART_1:
#if USART1_ENABLE
			COM_Send_Data_Handle(&huart1,STRUCT,size);
#endif
        break;

        case COM_USART_2:
#if USART2_ENABLE
			COM_Send_Data_Handle(&huart2,STRUCT,size);
#endif
            break;

        case COM_USART_3:
#if USART3_ENABLE
			COM_Send_Data_Handle(&huart3,STRUCT,size);
#endif
            break;

        case COM_USART_4:
#if USART4_ENABLE
			COM_Send_Data_Handle(&huart4,STRUCT,size);
#endif
            break;

        case COM_USART_5:
#if USART5_ENABLE
			COM_Send_Data_Handle(&huart5,STRUCT,size);
#endif
            break;

        case COM_USART_6:
#if USART6_ENABLE
			COM_Send_Data_Handle(&huart6,STRUCT,size);
#endif
            break;
		case COM_USART_7:
#if USART7_ENABLE
			COM_Send_Data_Handle(&huart7,STRUCT,size);
#endif
			break;
		case COM_USART_8:
#if USART8_ENABLE
			COM_Send_Data_Handle(&huart8,STRUCT,size);
#endif
			break;
		
		case COM_LPUSART_8:
#if LPUART1_ENABLE
			COM_Send_Data_Handle(&hlpuart1,STRUCT,size);
#endif
			break;
        default:
            break;
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口数据发送完成
*PARAMETERS:(用于发送完成回调) com_id:COM_TYPE_E
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_API_Send_Success_Isr(uint8_t com_id)
{
    COM_PORT_REC_QUEUE_T *STRUCT = NULL;
	
	if (com_id >= COM_USART_END)
	{
		return;
	}
    switch(com_id)
    {
    case 1:
#if USART1_ENABLE
        STRUCT = &usart1_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    case 2:
#if USART2_ENABLE
        STRUCT = &usart2_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    case 3:
#if USART3_ENABLE
        STRUCT = &usart3_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    case 4:
#if USART4_ENABLE
        STRUCT = &usart4_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    case 5:
#if USART5_ENABLE
        STRUCT = &usart5_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    case 6:
#if USART6_ENABLE
        STRUCT = &usart6_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
	    case 7:
#if USART7_ENABLE
        STRUCT = &usart7_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
		
	    case 8:
#if USART8_ENABLE
        STRUCT = &usart8_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
		
		case 9:
#if LPUART1_ENABLE
        STRUCT = &lpusart_port_rec_handle;
        STRUCT->tflag = 0;
        STRUCT->overtime_cnt = 0;
#endif
        break;
    default:
        break;
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口发送回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if USART1_ENABLE
    if (huart->Instance == USART1)
    {
        COM_API_Send_Success_Isr(COM_USART_1);
    }
#endif
#if USART2_ENABLE
    if (huart->Instance == USART2)
    {
        COM_API_Send_Success_Isr(COM_USART_2);
    }
#endif
#if USART3_ENABLE
    if (huart->Instance == USART3)
    {
        COM_API_Send_Success_Isr(COM_USART_3);
    }
#endif
#if USART4_ENABLE
    if (huart->Instance == UART4)
    {
        COM_API_Send_Success_Isr(COM_USART_4);
    }
#endif
#if USART5_ENABLE
    if (huart->Instance == UART5)
    {
        COM_API_Send_Success_Isr(COM_USART_5);
    }
#endif
#if USART6_ENABLE
    if (huart->Instance == USART6)
    {
        COM_API_Send_Success_Isr(COM_USART_6);
    }
#endif
#if USART7_ENABLE
    if (huart->Instance == UART7)
    {
        COM_API_Send_Success_Isr(COM_USART_7);
    }
#endif
#if USART8_ENABLE
    if (huart->Instance == UART8)
    {
        COM_API_Send_Success_Isr(COM_USART_8);
    }
#endif
#if LPUART1_ENABLE
    if (huart->Instance == LPUART1)
    {
        COM_API_Send_Success_Isr(COM_LPUSART_8);
    }
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:TX 超时检测（1ms 调用）
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void COM_Uart_TxTimeout_op(UART_HandleTypeDef *huart, COM_PORT_REC_QUEUE_T *pstruct)
{
    if (huart == NULL || pstruct == NULL)
    {
        return;
    }

    if (pstruct->tflag == 1U)
    {
        pstruct->overtime_cnt++;
    }

    if (pstruct->overtime_cnt >= 3U)
    {
        pstruct->overtime_cnt = 0;
        pstruct->tflag = 0;
        if (huart->gState != HAL_UART_STATE_READY)
        {
            COM_Uart_TxHardRecover(huart);
        }
    }
}

void COM_Uart_TxTimeout_Handler(void)
{
#if USART1_ENABLE
    COM_Uart_TxTimeout_op(&huart1, &usart1_port_rec_handle);
#endif
#if USART2_ENABLE
    COM_Uart_TxTimeout_op(&huart2, &usart2_port_rec_handle);
#endif
#if USART3_ENABLE
    COM_Uart_TxTimeout_op(&huart3, &usart3_port_rec_handle);
#endif
#if USART4_ENABLE
    COM_Uart_TxTimeout_op(&huart4, &usart4_port_rec_handle);
#endif
#if USART5_ENABLE
    COM_Uart_TxTimeout_op(&huart5, &usart5_port_rec_handle);
#endif
#if USART6_ENABLE
    COM_Uart_TxTimeout_op(&huart6, &usart6_port_rec_handle);
#endif
#if USART7_ENABLE
    COM_Uart_TxTimeout_op(&huart7, &usart7_port_rec_handle);
#endif
#if USART8_ENABLE
    COM_Uart_TxTimeout_op(&huart8, &usart8_port_rec_handle);
#endif
#if LPUART1_ENABLE
    COM_Uart_TxTimeout_op(&hlpuart1, &lpusart1_port_rec_handle);
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信超时判断
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_API_Communicate_Judg_Timer(void)
{
#if USART1_ENABLE
    if(++usart1_port_rec_handle.communi_err_cnt > USART1_CONNECT_TIME_OUT)
    {
		usart1_port_rec_handle.communi_err_flg_cnt++;
        usart1_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart1_port_rec_handle.communi_err_flg = 0;
    }
#endif

#if USART2_ENABLE
    if(++usart2_port_rec_handle.communi_err_cnt > USART2_CONNECT_TIME_OUT)
    {
		usart2_port_rec_handle.communi_err_flg_cnt++;
        usart2_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart2_port_rec_handle.communi_err_flg = 0;
    }
#endif

#if USART3_ENABLE
    if(++usart3_port_rec_handle.communi_err_cnt > USART3_CONNECT_TIME_OUT)
    {
		usart3_port_rec_handle.communi_err_flg_cnt++;
        usart3_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart3_port_rec_handle.communi_err_flg = 0;
    }
#endif

#if USART4_ENABLE
    if(++usart4_port_rec_handle.communi_err_cnt > USART4_CONNECT_TIME_OUT)
    {
		usart4_port_rec_handle.communi_err_flg_cnt++;
        usart4_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart4_port_rec_handle.communi_err_flg = 0;
    }
#endif
#if USART5_ENABLE
    if(++usart5_port_rec_handle.communi_err_cnt > USART5_CONNECT_TIME_OUT)
    {
		usart5_port_rec_handle.communi_err_flg_cnt++;
        usart5_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart5_port_rec_handle.communi_err_flg = 0;
    }
#endif
#if USART6_ENABLE
    if(++usart6_port_rec_handle.communi_err_cnt > USART6_CONNECT_TIME_OUT)
    {
		usart6_port_rec_handle.communi_err_flg_cnt++;
        usart6_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart6_port_rec_handle.communi_err_flg = 0;
    }
#endif
#if USART7_ENABLE
    if(++usart7_port_rec_handle.communi_err_cnt > USART7_CONNECT_TIME_OUT)
    {
		usart7_port_rec_handle.communi_err_flg_cnt++;
        usart7_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart7_port_rec_handle.communi_err_flg = 0;
    }
#endif

#if USART8_ENABLE
    if(++usart8_port_rec_handle.communi_err_cnt > USART8_CONNECT_TIME_OUT)
    {
		usart8_port_rec_handle.communi_err_flg_cnt++;
        usart8_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        usart8_port_rec_handle.communi_err_flg = 0;
    }
#endif

#if LPUART1_ENABLE
    if(++usart8_port_rec_handle.communi_err_cnt > LPUART1_CONNECT_TIME_OUT)
    {
        lpusart1_port_rec_handle.communi_err_flg = 1;
    }
    else
    {
        lpusart1_port_rec_handle.communi_err_flg = 0;
    }
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信错误状态位
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t COM_Connect_Err_Sta(COM_TYPE_E com)
{
    COM_TYPE_E port = com;
	uint8_t com_err_sta = 0;
	if (com >= COM_USART_END)
	{
		return 0;
	}
    switch (port)
    {
#if USART1_ENABLE
    case COM_USART_1:
        com_err_sta = usart1_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART2_ENABLE
    case COM_USART_2:
        com_err_sta = usart2_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART3_ENABLE
    case COM_USART_3:
        com_err_sta = usart3_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART4_ENABLE
    case COM_USART_4:
       com_err_sta = usart4_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART5_ENABLE
    case COM_USART_5:
        com_err_sta = usart5_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART6_ENABLE
    case COM_USART_6:
        com_err_sta = usart6_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART7_ENABLE
    case COM_USART_7:
        com_err_sta = usart7_port_rec_handle.communi_err_flg;
        break;
#endif
#if USART8_ENABLE
    case COM_USART_8:
        com_err_sta = usart8_port_rec_handle.communi_err_flg;
        break;
#endif
    default:
		//com_err_sta = 0;
        break;
    }

    return com_err_sta;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收错误计数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t COM_Connect_Err_Cnt(COM_TYPE_E com)
{
    COM_TYPE_E port = com;
	uint8_t com_err_sta = 0;
	if (com >= COM_USART_END)
	{
		return 0;
	}
    switch (port)
    {
#if USART1_ENABLE
    case COM_USART_1:
        com_err_sta = usart1_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART2_ENABLE
    case COM_USART_2:
        com_err_sta = usart2_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART3_ENABLE
    case COM_USART_3:
        com_err_sta = usart3_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART4_ENABLE
    case COM_USART_4:
       com_err_sta = usart4_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART5_ENABLE
    case COM_USART_5:
        com_err_sta = usart5_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART6_ENABLE
    case COM_USART_6:
        com_err_sta = usart6_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART7_ENABLE
    case COM_USART_7:
        com_err_sta = usart7_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
#if USART8_ENABLE
    case COM_USART_8:
        com_err_sta = usart8_port_rec_handle.communi_err_flg_cnt;
        break;
#endif
    default:
		//com_err_sta = 0;
        break;
    }
    return com_err_sta;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信异常回调计数值获取
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t COM_Recv_Err_Cnt(COM_TYPE_E com)
{
    COM_TYPE_E port = com;
	uint8_t com_err_sta = 0;
	if (com >= COM_USART_END)
	{
		return 0;
	}
    switch (port)
    {
#if USART1_ENABLE
    case COM_USART_1:
        com_err_sta = usart1_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART2_ENABLE
    case COM_USART_2:
        com_err_sta = usart2_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART3_ENABLE
    case COM_USART_3:
        com_err_sta = usart3_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART4_ENABLE
    case COM_USART_4:
       com_err_sta = usart4_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART5_ENABLE
    case COM_USART_5:
        com_err_sta = usart5_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART6_ENABLE
    case COM_USART_6:
        com_err_sta = usart6_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART7_ENABLE
    case COM_USART_7:
        com_err_sta = usart7_port_rec_handle.recv_callback_cnt;
        break;
#endif
#if USART8_ENABLE
    case COM_USART_8:
        com_err_sta = usart8_port_rec_handle.recv_callback_cnt;
        break;
#endif
    default:
		//com_err_sta = 0;
        break;
    }
    return com_err_sta;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口printf()重定义
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
int fputc(int ch, FILE *f)
{
#if 1
	switch(COM_DEBUG_ON)
	{
		case 1:
#if USART1_ENABLE
			 HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 1);
#endif
		break; 
		
		case 2:
#if USART2_ENABLE
			 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 1);
#endif
		break;
		
		case 3:
#if USART3_ENABLE
			 HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1);
#endif
		break;
		
		case 4:
#if USART4_ENABLE
			 HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 1);
#endif
		break;
		
		case 5:
#if USART5_ENABLE
			 HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 1);
#endif
		break;
		
		case 6:
#if USART6_ENABLE
			 HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 1);
#endif
		break;
		
		case 7:
#if USART7_ENABLE
			 HAL_UART_Transmit(&huart7, (uint8_t *)&ch, 1, 1);
#endif
		break;
		case 8:
#if USART8_ENABLE
			 HAL_UART_Transmit(&huart8, (uint8_t *)&ch, 1, 1);
#endif
		break;
		case COM_LPUSART_8:
#if LPUART1_ENABLE
			 HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 1);
#endif
		break;
		default:
			
		break;
	}
	return 0;
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION://串口调试信息打印
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_Printf_Dump(uint8_t prt_type)
{
	//串口调试信息打印
	if (prt_type == 1)
	{
		printf("\r\n debug_communi :\r\n com1:%d \r\n com2:%d \r\n com3:%d \r\n com4:%d \r\n com5:%d \r\n com6:%d \r\n com7:%d \r\n com8:%d",
		usart1_port_rec_handle.communi_err_flg_cnt,
		usart2_port_rec_handle.communi_err_flg_cnt,
		usart3_port_rec_handle.communi_err_flg_cnt,
		usart4_port_rec_handle.communi_err_flg_cnt,
		usart5_port_rec_handle.communi_err_flg_cnt,
		usart6_port_rec_handle.communi_err_flg_cnt,
		usart7_port_rec_handle.communi_err_flg_cnt,
		usart8_port_rec_handle.communi_err_flg_cnt);
	}
	
	if (prt_type == 2)
	{
		printf("\r\n debug_intr :\r\n com1:%d \r\n com2:%d \r\n com3:%d \r\n com4:%d \r\n com5:%d \r\n com6:%d \r\n com7:%d \r\n com8:%d",
		usart1_port_rec_handle.dbg_intr_cnt,
		usart2_port_rec_handle.dbg_intr_cnt,
		usart3_port_rec_handle.dbg_intr_cnt,
		usart4_port_rec_handle.dbg_intr_cnt,
		usart5_port_rec_handle.dbg_intr_cnt,
		usart6_port_rec_handle.dbg_intr_cnt,
		usart7_port_rec_handle.dbg_intr_cnt,
		usart8_port_rec_handle.dbg_intr_cnt);
	}
	
	if (prt_type == 3)
	{
		printf("\r\n debug_callback :\r\n com1:%d \r\n com2:%d \r\n com3:%d \r\n com4:%d \r\n com5:%d \r\n com6:%d \r\n com7:%d \r\n com8:%d",
		usart1_port_rec_handle.recv_callback_cnt,
		usart2_port_rec_handle.recv_callback_cnt,
		usart3_port_rec_handle.recv_callback_cnt,
		usart4_port_rec_handle.recv_callback_cnt,
		usart5_port_rec_handle.recv_callback_cnt,
		usart6_port_rec_handle.recv_callback_cnt,
		usart7_port_rec_handle.recv_callback_cnt,
		usart8_port_rec_handle.recv_callback_cnt);
	}




}
