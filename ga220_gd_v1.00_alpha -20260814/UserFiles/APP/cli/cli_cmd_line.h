/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CLI_CMD_LINE_H
#define __CLI_CMD_LINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "Common/base_inc.h"

/* USER CODE BEGIN Includes */
#if CLI_INCLUDE
#define UART_RX_BUF_SIZE  128
#define SHELL_CMD_LEN 64
typedef struct
{
    uint16_t    idx;	// 接收命令最后一个字符的下标
    uint8_t 	rx_buff;  // 中断中用于接收命令的数组
    char        cmd[SHELL_CMD_LEN + 1];	// 待处理的命令
    uint16_t    len;		// 接收完成后的命令长度
    char* 		pSecondPos;	// 命令二级参数的首地址
    char* 		pThirdPos;	// 命令三级参数的首地址
    char*		pFourthPos;	// 命令四级参数的首地址
    uint8_t 	hasCmd;		// 收到一条命令的标志
    uint8_t* (*isr_getaddr) (void);			// 中断中获取setbuffer的地址
	uint8_t password_sta;
} ShellCmd_t;

typedef enum
{
    DEBUG_GD_INFO,
    DEBUG_EXT_GD,
    DEBUG_LASER_INFO,
    DEBUG_LASER_RX,
    DEBUG_LASER_TX,
    DEBUG_IR_INFO,
    DEBUG_VL_INFO,
    DEBUG_VL_RX,
    DEBUG_VL_TX,
	DEBUG_NXIR_INFO,
	DEBUG_UDP_INFO,
	DEBUG_UDP_MASTERRX,
	DEBUG_UDP_MASTERTX,
	DEBUG_UDP_TRACKRX,
	DEBUG_UDP_TRACKTX,
    DEBUG_INFO_END,
} DEBUG_TYPE_INFO;

typedef struct
{
    char *debug_file;//调试指令
    uint8_t file;//对应的枚举类型
    bool debug_sta;//打印状态
} DEBUG_FILE_MAP_T;


extern DEBUG_FILE_MAP_T debug_map[DEBUG_INFO_END];

#define DEBUG_INFO_PRINT(modulde,fmt,...)\
{\
if(debug_map[modulde].debug_sta)\
printf(fmt,##__VA_ARGS__);\
}
void cli_rx_handle(uint8_t *rx_buff,uint16_t rx_len);

#define DEBUG_GD_PRINT(fmt,...)    		DEBUG_INFO_PRINT(DEBUG_GD_INFO,fmt, ##__VA_ARGS__)
#define DEBUG_LASER_PRINT(fmt,...) 		DEBUG_INFO_PRINT(DEBUG_LASER_INFO,fmt, ##__VA_ARGS__)
#define DEBUG_IR_PRINT(fmt,...)    		DEBUG_INFO_PRINT(DEBUG_IR_INFO,fmt, ##__VA_ARGS__)
#define DEBUG_IR_TX_PRINT(fmt,...)    	DEBUG_INFO_PRINT(DEBUG_IR_TX,fmt, ##__VA_ARGS__)
#define DEBUG_IR_RX_PRINT(fmt,...)    	DEBUG_INFO_PRINT(DEBUG_IR_RX,fmt, ##__VA_ARGS__)
#define DEBUG_LASER_TX_PRINT(fmt,...)   DEBUG_INFO_PRINT(DEBUG_LASER_TX,fmt, ##__VA_ARGS__)
#define DEBUG_LASER_RX_PRINT(fmt,...)   DEBUG_INFO_PRINT(DEBUG_LASER_RX,fmt, ##__VA_ARGS__)
#define DEBUG_VL_RX_PRINT(fmt,...)    	DEBUG_INFO_PRINT(DEBUG_VL_RX,fmt, ##__VA_ARGS__)
#define DEBUG_VL_TX_PRINT(fmt,...)    	DEBUG_INFO_PRINT(DEBUG_VL_TX,fmt, ##__VA_ARGS__)
#define DEBUG_VL_INFO_PRINT(fmt,...)    DEBUG_INFO_PRINT(DEBUG_VL_INFO,fmt, ##__VA_ARGS__)
#define DEBUG_NX_IR_PRINT(fmt,...)    	DEBUG_INFO_PRINT(DEBUG_NXIR_INFO,fmt, ##__VA_ARGS__)
#define DEBUG_MASTER_RX_PRINT(fmt,...)  DEBUG_INFO_PRINT(DEBUG_UDP_MASTERRX,fmt, ##__VA_ARGS__)
#define DEBUG_MASTER_TX_PRINT(fmt,...)  DEBUG_INFO_PRINT(DEBUG_UDP_MASTERTX,fmt, ##__VA_ARGS__)
#define DEBUG_TRACK_RX_PRINT(fmt,...)   DEBUG_INFO_PRINT(DEBUG_UDP_TRACKRX,fmt, ##__VA_ARGS__)
#define DEBUG_TRACK_TX_PRINT(fmt,...)   DEBUG_INFO_PRINT(DEBUG_UDP_TRACKTX,fmt, ##__VA_ARGS__)
#define DEBUG_UDP_PRINT(fmt,...)   		DEBUG_INFO_PRINT(DEBUG_UDP_INFO,fmt, ##__VA_ARGS__)
void CLI_Rx_Handle(uint8_t *rx_buff,uint16_t rx_len);
void CLI_Cmd_Period_Handle(void);
void CLI_Cmd_Init(void);

#endif 
#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
