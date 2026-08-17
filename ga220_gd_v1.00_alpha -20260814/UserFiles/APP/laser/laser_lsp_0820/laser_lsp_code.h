#ifndef __LASER_LSP_CODE_H
#define __LASER_LSP_CODE_H
#include "Common/base_inc.h"
#include "Common/config.h"
#if LASER_LSP_LD_0820
#pragma pack(1)
typedef struct
{
	uint8_t head;
	uint8_t cmd_type;
	uint8_t code_class;
	uint16_t code_len;
	uint8_t code_cur_page;//传输分页号
	uint8_t code_cur_len;//当前分页数据量
	uint32_t code_data[11];
	uint8_t xor;
}LASER_LSP_CODE_T;
#pragma pack()
//写序列码返回的结构体
typedef struct
{
	uint8_t rcv_head;
	uint8_t rcv_cmd;
	uint8_t rcv_sta;
	uint8_t rcv_class;
	uint16_t code_len;
	uint8_t xor;
}LASER_CODE_RSP_T;
static bool code_write_sta = false;

typedef struct
{
	uint8_t read_head;
	uint8_t read_cmd;
	uint8_t read_class;
	uint8_t res1;
	uint8_t res2;
	uint8_t xor;
}LASER_CODE_READ_REQ_T;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:精频码写入
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Precision_Send(uint16_t Datalen,uint32_t *code);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:脉冲序列码写入
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Pulse_Send(uint8_t code_class,uint8_t base_freq,uint16_t Datalen,uint8_t *code);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:变间隔码写入
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t  LASER_CODE_Change_Interval_Send(uint8_t code_class,uint16_t Datalen,uint32_t *code);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收解算
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t LASER_CODE_Recv_Handle(uint16_t data_len,uint8_t *data_in);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:读取指令
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_CODE_Read_Req(uint8_t code_type,uint8_t code_class);
#endif
#endif

