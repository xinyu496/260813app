#include "../ir/ir_ctrl_api.h"
#include "Common/config.h"
#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
#include "../ir/ir_ctrl_dybmc_l640.h"
#endif
#if (IR_CTRL_INCLUDE&IR_LGCS123)
#include "../ir/ir_ctrl_lgcs123.h"
#endif

#if (IR_CTRL_INCLUDE&IR_S640A_C1)
#include "../ir/ir_ctrl_s640a_c1.h"
#endif
#if (IR_CTRL_INCLUDE&IR_NX30_150)
#include "../ir/ir_ctrl_nx30_150.h"
#endif
#if (IR_CTRL_INCLUDE&IR_LGCS6122)
#include "../ir/ir_ctrl_lgcs6122.h"
#endif


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void Infrared_API_Serial_Data_Init(void)
{
//根据opt_module里面的宏控制相关器件
#if (IR_CTRL_INCLUDE&IR_LGCS123)
	IR_LGCS_API_Serial_Data_Init();
#endif

#if (IR_CTRL_INCLUDE&IR_NX30_150)
	IR_NX30_API_Serial_Data_Init();
#endif

#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
	IR_Dybmc_Serial_Data_Init();
#endif

#if (IR_CTRL_INCLUDE&IR_S640A_C1)
	IR_S640A_API_Serial_Data_Init();
#endif

#if (IR_CTRL_INCLUDE&IR_LGCS6122)
	IR_LGCS_API_Serial_Data_Init();
#endif
	

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Infrared_API_Ctrl_SendHandle(uint8_t ir_type,uint8_t send_type,uint8_t *data)
{
	if ((ir_type&IR_CTRL_INCLUDE) == IR_LGCS123)
	{
#if (IR_CTRL_INCLUDE&IR_LGCS123)
		Ir_Ctrl_Lgcs_Data_SendHandle (send_type,data);
#endif
	}
	else if ((ir_type&IR_CTRL_INCLUDE) == IR_NX30_150)
	{
#if (IR_CTRL_INCLUDE&IR_NX30_150)
		Ir_Ctrl_Nx30_SendHandle(send_type,data);
#endif
	}
	else if ((ir_type&IR_CTRL_INCLUDE) == IR_DYBMC_L640C500A)
	{
#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
		IR_Ctrl_Dybmc_SendHandle(send_type,data);
#endif
	}
	else if ((ir_type&IR_CTRL_INCLUDE) == IR_S640A_C1)
	{
#if (IR_CTRL_INCLUDE&IR_S640A_C1)
		IR_S640A_SendHandle(send_type,data);
#endif
	}
	if ((ir_type&IR_CTRL_INCLUDE) == IR_LGCS6122)
	{
#if (IR_CTRL_INCLUDE&IR_LGCS6122)
		Ir_Ctrl_Lgcs_Data_SendHandle (send_type,data);
#endif
	}
	else
	{
		return CMD_ERR;
	}
    
	return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Infrared_API_Period_Handle(void)
{
#if (IR_CTRL_INCLUDE&IR_LGCS123)
		IR_API_Lgcs_Period_Handle();
#endif
	
#if (IR_CTRL_INCLUDE&IR_NX30_150)
		IR_API_Nx30_Period_Handle();
#endif
	
#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
		IR_Dybmc_Period_Handle();
#endif

#if (IR_CTRL_INCLUDE&IR_S640A_C1)
		IR_API_A640s_Period_Handle();
#endif
	
#if (IR_CTRL_INCLUDE&IR_LGCS6122)
		IR_API_Lgcs_Period_Handle();
#endif
	return CMD_SUCESS;
}
