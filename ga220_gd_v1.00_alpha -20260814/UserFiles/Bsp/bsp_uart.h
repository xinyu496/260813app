#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "Common/config.h"

#define debug_lxy_0  1
//串口号映射枚举
typedef enum
{
	COM_USART_0,
	COM_USART_1,
	COM_USART_2,
	COM_USART_3,
	COM_USART_4,
	COM_USART_5,
	COM_USART_6,
	COM_USART_7,
	COM_USART_8,
	COM_LPUSART_8,
	COM_USART_END,
}COM_TYPE_E;
/**********************串口使能开关*********************/
#define 	USART1_ENABLE 0
#define 	USART2_ENABLE 1
#define 	USART3_ENABLE 0
#define 	USART4_ENABLE 1	
#define 	USART5_ENABLE 1
#define 	USART6_ENABLE 1
#define 	USART7_ENABLE 0	
#define 	USART8_ENABLE 0	
#define 	LPUART1_ENABLE 0
/**********************串口接收环长度*********************/
#define		USART1_RING_BUFF_SIZE		32
#define		USART2_RING_BUFF_SIZE		128
#define		USART3_RING_BUFF_SIZE		32
#define		USART4_RING_BUFF_SIZE		128
#define		USART5_RING_BUFF_SIZE		128
#define		USART6_RING_BUFF_SIZE		128
#define		USART7_RING_BUFF_SIZE		32
#define		USART8_RING_BUFF_SIZE		32
#define 	LPUART1_RING_BUFF_SIZE		32
/**********************串口接收超时时间*********************/
#define 	USART1_CONNECT_TIME_OUT	20
#define 	USART2_CONNECT_TIME_OUT	20
#define 	USART3_CONNECT_TIME_OUT	20
#define 	USART4_CONNECT_TIME_OUT	20
#define 	USART5_CONNECT_TIME_OUT	20
#define 	USART6_CONNECT_TIME_OUT	20
#define 	USART7_CONNECT_TIME_OUT	20
#define 	USART8_CONNECT_TIME_OUT	20
#define 	LPUART1_CONNECT_TIME_OUT	20
/*串口映射*/
#define COM_GD_EXT		COM_USART_0
#define COM_GD_IN		COM_USART_6

#define COM_LASER_IN 	COM_USART_3

#define COM_IR_DYBMC	COM_USART_0
#define COM_IR_LGCS		COM_USART_4
#define COM_IR_S640A  	COM_USART_0
#define COM_IR_NX30		COM_USART_5
#define COM_IR_TWIN612RG2 COM_USART_3

#define COM_KJG_LD		COM_USART_0
#define COM_KJG_VS2030	COM_USART_1
#define COM_KJG_IN		COM_USART_0
#define COM_SF_IN		COM_USART_0
#define COM_STORAGE_OUT COM_USART_0
#define COM_TRACK   	COM_USART_0 //跟踪器串口

#define COM_MOTOR  		COM_USART_4 //电机驱动板串口
#define COM_BMQ_IN      COM_USART_0//编码器解析串口

/*串口映射*/
#define COM_DEBUG_ON  	COM_USART_3
#define COM_BOOT 		COM_USART_1

#if USART1_ENABLE
extern UART_HandleTypeDef huart1;
#endif

#if USART2_ENABLE
extern UART_HandleTypeDef huart2;
#endif

#if USART3_ENABLE
extern UART_HandleTypeDef huart3;
#endif

#if USART4_ENABLE
extern UART_HandleTypeDef huart4;
#endif

#if USART5_ENABLE
extern UART_HandleTypeDef huart5;
#endif

#if USART6_ENABLE
extern UART_HandleTypeDef huart6;
#endif

#if USART8_ENABLE
extern UART_HandleTypeDef huart8;
#endif 

#if LPUART1_ENABLE
extern UART_HandleTypeDef hlpuart1;
#endif
/**********************重定义串口号*********************/
//串口数据接收结构体
//typedef bool (*COM_API_RCV_CB)(uint8_t *data,uint16_t length);
typedef struct
{
    unsigned char         			*buffer_ptr;  	//缓冲区指针
    unsigned char         			*tbuffer_ptr;  	//发送缓冲区指针
    volatile unsigned int		 	write_index;    //写指针
	volatile unsigned int		 	read_index;    	//读指针
    volatile unsigned int		 	tread_index;    //发送读指针
    volatile unsigned int		 	twrite_index;   //发送写指针
    volatile unsigned char		 	rflag;    		//接收标志
    unsigned short int				buffer_size;	//缓冲环长度


    volatile unsigned int			dbg_intr_cnt;	//数据滞留超时时间
    unsigned char         			head_h;  		//报文头高8位
    unsigned char         			head_l;  		//报文头低8位
    unsigned char         			tail_h;  		//报文头高8位
    unsigned char         			tail_l;  		//报文头低8位
    unsigned short int				total_len;
    volatile unsigned short int		overtime_cnt;   //发送超时计数
    volatile uint16_t				communi_err_cnt;
    volatile uint8_t 				communi_err_flg; //通信异常标志
	volatile uint16_t 				communi_err_flg_cnt; //通信异常计数
	volatile uint16_t 				recv_callback_cnt; //通信异常回调计数
    unsigned char         			tflag;  		//发送空闲标志
} COM_PORT_REC_QUEUE_T;


//用户数据接收结构体
typedef struct
{
    uint8_t header1;
    uint8_t header2;
    uint16_t data_recv_len;
    uint8_t tail1;
    uint8_t tail2;
    uint8_t recv_buf[256];//优化成一个指针，变量长度变为可配置的
} COM_RECV_INFO_T;

//串口初始化
void COM_DRV_SerialPort_Init(void);
//获取串口句柄
COM_PORT_REC_QUEUE_T *COM_Rec_Get_Com(COM_TYPE_E com);
//获取串口接收数组
uint8_t *COM_Rec_Get_Rec_Arry(COM_TYPE_E com);
//获取串口发送数组
uint8_t *COM_Rec_Get_Send_Arry(COM_TYPE_E com);
//中断函数
void COM_Serial_Recv_IT(COM_TYPE_E com_id);
//串口数据发送
void COM_API_Send_Data(COM_TYPE_E com_id,uint8_t *tx_ptr, uint16_t size);

void COM_API_Communicate_Judg_Timer(void);

void COM_UART_Reset(COM_TYPE_E com_id);

uint8_t COM_Connect_Err_Cnt(COM_TYPE_E com);

uint16_t Err_Callback_recv(void);

uint8_t COM_Recv_Err_Cnt(COM_TYPE_E com);

void COM_Printf_Dump(uint8_t prt_type);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信错误状态位
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t COM_Connect_Err_Sta(COM_TYPE_E com);
extern volatile uint32_t com_rx_soft_recover_cnt;
extern volatile uint32_t com_rx_hard_recover_cnt;
extern volatile uint32_t com_rx_hard_recover_fail_cnt;
extern volatile uint32_t com_rx_dma_abort_fail_cnt;
extern volatile uint32_t com_tx_skip_cnt;
extern volatile uint32_t com_tx_start_fail_cnt;
extern volatile uint32_t com_tx_recover_cnt;

void COM_Uart_TxTimeout_Handler(void);
#endif
