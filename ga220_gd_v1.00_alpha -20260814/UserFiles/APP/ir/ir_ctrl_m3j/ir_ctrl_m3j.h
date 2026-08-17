#ifndef __IR_CTRL_M3J_H
#define __IR_CTRL_M3J_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "Common/opt_module.h"
#include "Common/config.h"

#if (IR_CTRL_INCLUDE&IR_M3J)
#pragma pack(1)
typedef struct
{
    uint8_t Header;
    uint8_t data_len;
    uint8_t dev_id;
    uint8_t ctrl_cmd[3];
	uint8_t sum_check;
	uint8_t tail1;
	uint8_t tail2;
} IR_M3J_HEADER_T;
#pragma pack()
#define IR_M3J_HEADER_LEN  sizeof(IR_M3J_HEADER_T)

#pragma pack(1)
typedef struct
{
    uint8_t Header;
    uint8_t data_len;
    uint8_t ctrl_cmd[3];
    uint8_t sum_check;
    uint8_t tail1;
	uint8_t tail2;
} IR_M3J_RCV_DATA_T;
#pragma pack()
#define IR_M3J_RCV_DATA_LEN  sizeof(IR_M3J_RCV_DATA_T)

typedef enum
{
    IR_M3J_CONTRAST = 0x21,			/*对比度设置*/
    IR_M3J_LIGHT = 0x22,				/*亮度设置*/
    IR_M3J_B_W_CH = 0x42,				/*极性设置*/
} IR_M3J_CORE_SEND_TYPE;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_M3J_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_M3J_SendHandle(uint8_t send_type,uint8_t *data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
RECV_DATA_ERR_STA IR_API_M3J_Period_Handle(void);
#endif
#endif
