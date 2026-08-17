#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"
/*==============================================================
波特率 115200；
*==============================================================*/
#if LASER_15H
#include "laser_15h.h"
#include "Common/config.h"

static uint8_t single_cmd[8]={0x55,0xAA,0x88,0xFF,0xFF,0xFF,0xFF,0x83};
static uint8_t stop_cmd[8]={0x55,0xAA,0x8E,0xFF,0xFF,0xFF,0xFF,0x89};
static uint8_t continuous_cmd[8]={0x55,0xAA,0x89,0xFF,0xFF,0xFF,0xFF,0x8A};

uint8_t laser_serial_rcv_buf[20] = {0};
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:报文发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t lrd_laser_sta;
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd, uint16_t cmd_para1,
                              uint16_t cmd_para2)
{
	SYS_LASER_STA_T* local_laser_sta;
	local_laser_sta = CONFIG_Get_Laser_Sta();

    switch (cmd)
    {
    case LZ_DETEC_STOP://待机
		COM_API_Send_Data(COM_LASER_IN, (uint8_t *)&stop_cmd, sizeof(stop_cmd));
        break;

    case LZ_DETECT_FREQ_CONTINUOUS:/*连续测距*/
		COM_API_Send_Data(COM_LASER_IN, (uint8_t *)&continuous_cmd, sizeof(continuous_cmd));
        break;
	case LZ_SINGLE_DETECT:/*单次测距*/
		COM_API_Send_Data(COM_LASER_IN, (uint8_t *)&single_cmd, sizeof(single_cmd));
		break;
    default:
        break;
    }

//发送完了状态清空，参数清空
//    CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
//    CONFIG_Set_Master_Ctrl_Para(MASTER_LASER_PARA, 0);
    return CMD_SUCESS;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:报文解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

uint8_t laser_process_lrd_data_in(uint8_t *data, uint32_t length)
{
    
    uint8_t data_add;
    uint16_t detect_distance = 0;
	float temp = 0;;
    uint16_t detect_code = 0;
    data_add = UTL_ADD_CHECK_REV((uint8_t *)&data[0], (length - 1));

    //校验计算
    if(data[length - 1] != data_add)
    {
        return RECV_DATA_XOR_ERR;
    }
	SYS_LASER_STA_T* local_laser_sta;
    local_laser_sta = CONFIG_Get_Laser_Sta();

    //输出测量数据的报文
    if (data[3] == 1)
    {
		if (data[2] == 0x88)
		{
			detect_distance =((uint16_t)data[5]) << 8 | data[6];//激光测距值
			temp = detect_distance*0.1;
			local_laser_sta->distance[0] = temp;
			local_laser_sta->sys_sta = LASER_WORK_SINGLE;//激光测距状态
		}
		else if(data[2] == 0x89)
		{
			detect_distance =((uint16_t)data[5]) << 8 | data[6];//激光测距值
			temp = detect_distance*0.1;
			local_laser_sta->distance[0] = temp;
			local_laser_sta->sys_sta = LASER_WORK_CONTINUOUS;//激光测距状态
		}
		else if(data[2] == 0x8E)
		{
			local_laser_sta->sys_sta = LASER_WORK_STAY;//激光测距状态
		}
    }
	else if(data[2] == 0x8E || data[3] == 0)			//停止失败，异常
	{
		local_laser_sta->laser_alarm.laser_system = 1;
	}
	else
	{
		local_laser_sta->distance[0] = 0;
		local_laser_sta->laser_alarm.overhang = 0;           //超距
	}
    return RECV_DATA_SUC;
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
uint8_t LASER_API_Period_Handle(void)
{
    uint8_t recv_len = 0;
    recv_len = COM_REC_Data_Direct(COM_LASER_IN,laser_serial_rcv_buf);

    if (recv_len == 0)
    {
        return RECV_DATA_NULL;
    }

#if CLI_INCLUDE
    DEBUG_LASER_RX_PRINT("\r\n LASER_RCV:");

    for(uint16_t i = 0; i < recv_len; i++)
    {
        DEBUG_LASER_RX_PRINT("%02x ", laser_serial_rcv_buf[i]);
    }
#endif
    laser_process_lrd_data_in(laser_serial_rcv_buf, recv_len);
    return RECV_DATA_SUC;
}
#endif


