#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H
#include "Common/base_inc.h"

typedef enum
{
	MOTOR_OFF,
	MOTOR_ON,
	MOTOR_FIND_ZERO = 0xF0,
	MOTOR_STANBY,
	MOTOR_L_SET,
	MOTOR_L_READ,
	MOTOR_R_SET,
	MOTOR_R_READ,
	MOTOR_POLE_SET,
	MOTOR_POLE_READ,
	MOTOR_ZERO_READ,
	MOTOR_SATURATE_SET,
	MOTOR_SATURATE_READ,
	MOTOR_U_SET,
	MOTOR_U_READ,
	MOTOR_READ_FLASH,
	MOTOR_CFG_END,
}MOTOR_CTRL_INFO_E;//电机驱动控制指令集

typedef struct  
{
	uint8_t 	Head;//数据头
	int16_t 	Current;//电流
	uint8_t 	DrvSta;//驱动状态
	uint8_t 	ParIde;//参数标识
	uint16_t 	ParVal;//参数值
	uint8_t 	Sum;//校验和
}userRxMotorDrvParTypeDef;//接收电机驱动器数据
// userRxMotorDrvParTypeDef userRxMotorDrvPar;
typedef struct  
{
	uint8_t  Cfg_flg;//电机驱动配置标识
	uint8_t  Cfg_Cmd;//电机驱动配置指令
	uint16_t Cfg_Par;//电机驱动配置指令
}userTxMotorCfgParTypeDef;//电机驱动配置参数
// userTxMotorCfgParTypeDef userTxMotorCfgPar;

typedef struct  
{
	uint8_t  Use_flg;//电机驱动使用标识
	int16_t  Current_Par;//电机驱动电流
	uint8_t  Enable;//电机驱动使能
}userTxMotorDrvParTypeDef;//电机使用驱动参数
// userTxMotorDrvParTypeDef userTxMotorDrvPar;

typedef struct  
{
	uint16_t  CfgCmdBac_flg;//配置指令反馈标识
	uint16_t  CfgCmdBac;//反馈的配置指令
	uint16_t  Proportion;//比例参数
	uint16_t  Integral;//积分参数
	uint16_t  PoleNum;//电机极对数参数
	uint16_t  AlignVoltage;//对齐电压
	uint16_t  SaturationOut;//饱和输出
	uint16_t  SeekSequent;//寻零结果
}userRxMotorCfgParTypeDef;//电机驱动配置参数
// userRxMotorCfgParTypeDef userRxMotorCfgPar;
//发送函数
void Motor_Send_Handler(uint8_t send_flag,uint8_t ctrl_type,uint8_t *send_para);
//接收解析函数
uint8_t MOTOR_API_Period_Handle(void);
int16_t Motor_Get_Current(void);
void Motor_Set_Current(userTxMotorDrvParTypeDef drv_info);
#endif
