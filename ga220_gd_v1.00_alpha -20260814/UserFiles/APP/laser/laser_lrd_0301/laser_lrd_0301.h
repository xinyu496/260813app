#ifndef __LASER_LRD_0301_H
#define __LASER_LRD_0301_H
#include "Common/base_inc.h"
#include "Common/config.h"

/*模块开关*/
#if LASER_LRD_0301

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t LASER_API_Period_Handle(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
uint8_t Laser_Ctrl_Send_Lrd_Handle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
uint8_t Laser_Ctrl_Send_Ld_Handle(SYS_LASER_DETECT_MODE_E cmd,uint16_t cmd_para1,uint16_t cmd_para2);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t laser_process_data_in(uint8_t *data,uint32_t length);
#endif
#endif
