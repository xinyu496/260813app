#include "Common/base_inc.h"
#include "Driver/drv_uart.h"
#include "Common/utl_math.h"
#include "../visible/visible_vs2030.h"
#include "../visible/visible_light_f15.h"
#include "../visible/visible_ctrl.h"
#include "Common/config.h"
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_API_Serial_Data_Init(void)
{
//根据opt_module里面的宏控制相关器件
#if (VISIBLE_INCLUDE&VL_VS2030)
    VISIBLE_VS2030_Serial_Data_Init();
#endif

#if (VISIBLE_INCLUDE&VL_F15_300)
    VISIBLE_F15_Serial_Data_Init();
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:控制指令；
			ctrl_type:指令类型；
			ctrl_data:数据内容；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Visible_API_Ctrl_SendHandle(uint8_t vl_type,uint8_t send_type,uint8_t *ctrl_data)
{

    if ((vl_type&VISIBLE_INCLUDE) == VL_VS2030)
    {
#if (VISIBLE_INCLUDE&VL_VS2030)
        Visible_VS2030_Ctrl_SendHandle(send_type,ctrl_data);
#endif
    }
    else if ((vl_type&VISIBLE_INCLUDE) == VL_F15_300)
    {
#if (VISIBLE_INCLUDE&VL_F15_300)
        Visible_F15_Ctrl_SendHandle(send_type,ctrl_data);
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
uint8_t VISIABLE_API_Period_Handle(void)
{
#if (VISIBLE_INCLUDE&VL_VS2030)
    VISIABLE_VS2030_Period_Handle();
#endif

#if (VISIBLE_INCLUDE&VL_F15_300)
    VISIABLE_F15_Period_Handle();
#endif
    return CMD_SUCESS;
}
