#ifndef __IR_CTRL_LGCS123_H
#define __IR_CTRL_LGCS123_H
#include "Common/base_inc.h"
#include "Common/config.h"

#if (IR_CTRL_INCLUDE&IR_LGCS123)
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t ctrl_cmd;
    uint8_t ctrl_data1;
    uint8_t ctrl_data2;
    uint8_t ctrl_data3;
    uint8_t ctrl_data4;
    uint8_t tail_1;
    uint8_t tail_2;
} IR_LGCS_DATA_T;
#pragma pack()
#define IR_LGCS_DATA_LEN  sizeof(IR_LGCS_DATA_T)

#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t ctrl_cmd;
    uint8_t ctrl_data1;
    uint8_t ctrl_data2;
    uint8_t ctrl_data3;
    uint8_t tail_1;
    uint8_t tail_2;
} IR_LGCS_RCV_DATA_T;
#pragma pack()
#define IR_LGCS_RCV_DATA_LEN  sizeof(IR_LGCS_RCV_DATA_T)

typedef enum
{
    IR_SHUTTER_SET = 0x2,
    IR_LGCS_SHUTTER_OFF = 0x11,
    IR_DDE_SET = 0x19,
    IR_LGCS_CONTRAST_SET = 0x21,
    IR_LIGHT_SET = 0x23,
    IR_DETIAL_GIAN = 0x1e,
    IR_POLARITY_SET=0x42,
    IR_RECV_END,
} IR_CORE_RECV_TYPE;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_LGCS_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:dde挡位：0-8
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Ir_Ctrl_Lgcs_Data_SendHandle(uint8_t send_type,uint8_t *data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:dde挡位：0-8
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
RECV_DATA_ERR_STA IR_API_Lgcs_Period_Handle(void);
#endif
#endif
