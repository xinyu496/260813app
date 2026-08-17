#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"
/*==============================================================
波特率 500000；
LSP-LRD-1500和LSP-LRD-2000协议一致
*==============================================================*/
#if LASER_LRD_1200_G
#include "laser_lrd_1200_g.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"

static uint8_t start_cmd[8]={0x55,0xaa,0xcb,0xcc,0xcc,0xcc,0xcc,0xfb};
static uint8_t stop_cmd[8]={0x55,0xaa,0xcc,0xcc,0xcc,0xcc,0xcc,0xfc};
COM_RECV_INFO_T laser_serial_rcv_buf;

uint8_t lrd_send_buf[8] = {0};
uint8_t lrd_laser_sta;
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd, uint16_t cmd_para1,
                              uint16_t cmd_para2)
{
	SYS_LASER_STA_T* local_laser_sta;
	local_laser_sta = CONFIG_Get_Laser_Sta();

    switch (cmd)
    {
    case LZ_DETEC_STOP://待机
		memcpy(lrd_send_buf,stop_cmd,sizeof(stop_cmd));
		local_laser_sta->sys_sta = LASER_WORK_STAY;//激光测距状态
        break;

    case LZ_DETECT_FREQ_CONTINUOUS:/*连续测距*/
		memcpy(lrd_send_buf,start_cmd,sizeof(start_cmd));
        break;

    default:
        break;
    }

#if COM_LASER_IN_cmd_debug
    SEGGER_RTT_SetTerminal(1);
    SEGGER_RTT_printf(0, "COM_LASER_IN send:" );
    LZ_LRD_SEND_T *STRUCT = &lrd_send_buf;

    for(uint8_t k = 0; k < sizeof(lrd_send_buf) ; k++)
    {
        SEGGER_RTT_printf(0, "0x%x ", *STRUCT);
        STRUCT++;

        if(k == sizeof(lrd_send_buf) - 1)
        {
            SEGGER_RTT_printf(0, " \n" );
        }
    }

    SEGGER_RTT_SetTerminal(0);
#endif
    COM_API_Send_Data(COM_LASER_IN, (uint8_t *)&lrd_send_buf, sizeof(lrd_send_buf));
    //发送完了状态清空，参数清空
    CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
    CONFIG_Set_Master_Ctrl_Para(MASTER_LASER_PARA, 0);
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
    uint32_t detect_distance = 0;
    uint16_t detect_code = 0;
    data_add = UTL_ADD_CHECK_REV((uint8_t *)&data[1], (length - 2));

    //校验计算
    if(data[length - 1] != data_add)
    {
        return RECV_DATA_XOR_ERR;
    }
	SYS_LASER_STA_T* local_laser_sta;
    local_laser_sta = CONFIG_Get_Laser_Sta();

    //输出测量数据的报文
    if (data[0] == 0x5c)
    {
		uint32_t temp = data[1] |((uint32_t)data[2]) << 8 | ((uint32_t)data[3]) << 16;//激光测距值
		float temp2 = temp*0.01;
		local_laser_sta->distance[0] = temp2;
		local_laser_sta->sys_sta = LASER_WORK_CONTINUOUS;//激光测距状态
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
//    uint8_t i = 0;
    uint8_t recv_len = 0;
    recv_len = COM_REC_Data_Direct(COM_LASER_IN,laser_serial_rcv_buf.recv_buf);

    if (recv_len == 0)
    {
        return RECV_DATA_NULL;
    }

#if CLI_INCLUDE
    DEBUG_LASER_RX_PRINT("\r\n LASER_RCV:");

    for(uint16_t i = 0; i < recv_len; i++)
    {
        DEBUG_LASER_RX_PRINT("%02x ", laser_serial_rcv_buf.recv_buf[i]);
    }

#endif
    laser_process_lrd_data_in(laser_serial_rcv_buf.recv_buf, recv_len);
    return RECV_DATA_SUC;
}
#endif


