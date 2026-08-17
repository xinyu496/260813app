#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"

#if LASER_DYC_15A
#include "laser_dyc_15a.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"
#include "Common/utl_math.h"

//串口波特率115200

#define LASER_DATA_IN_HEADER_1 0xEE
#define LASER_DATA_IN_HEADER_2 0x16
#define LASER_LRD_IN_HEADER_1 0x5A
#define LASER_DATA_IN_DEVICE_ID 0x03
 

COM_RECV_INFO_T laser_serial_rcv_buf;

#define DETECT_TIME_MAX_S  	 (300)
#define DETECT_TIME_MAX_S_2  (60)
#define DETECT_TIME_MAX_S_3  (60)
#define DETECT_DELAY_MAX_S   (3000)
#define LIGHT_RANGE_MAX_1 	 (3)
#define LIGHT_RANGE_MAX_2 	 (1023)

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LASER_API_Serial_Data_Init(void)
{

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光测照
*PARAMETERS:
		cmd；LASEER_DETECT_MOD
		cmd_para1:使能或者选通值等通用参数
		cmd_para2:仅用于放照射序列码
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t dyc_laser_send_buf[11] = {0};
uint8_t lrd_laser_sta;
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd, uint16_t cmd_para1,
                              uint16_t cmd_para2)
{
    LZ_DYC_15A_SEND_T lrd_send_buf = {0};
    memset(&lrd_send_buf, 0, sizeof(LZ_DYC_15A_SEND_T));
    lrd_send_buf.header1 = LASER_DATA_IN_HEADER_1;
    lrd_send_buf.header2 = LASER_DATA_IN_HEADER_2;
	lrd_send_buf.device_ID = LASER_DATA_IN_DEVICE_ID;
	uint8_t send_para_len = 0;
	
    switch (cmd)
    {
    case LZ_DETEC_STOP://待机
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x05;
		break;

    case LZ_DETECT_SELFCHECK://自检
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x01;
		lrd_laser_sta = 0x00;
        break;

    case LZ_SINGLE_DETECT:/*单次测距*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x02;
		//lrd_laser_sta = 0x01;
        break;

    case LZ_DETECT_FREQ_CONTINUOUS:/*连续测距*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x02;
		//lrd_laser_sta = 0x02;
        break;
	
	case LZ_DETECT_FREQ_5:/*5Hz*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x02;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = 0x5;//0x01～0x0A  连续测距频率
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = 0;
        break;
	
	case LZ_DETECT_FREQ_10:/*10Hz*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0x02;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = 0xa;//0x01～0x0A  连续测距频率
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = 0;
        break;

    case LZ_GOAL_FIRST_TARGET:/*首目标测距*/
		send_para_len = 1;
		lrd_send_buf.data_length = 0x03;
		lrd_send_buf.cmd_type = 0x03;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = 0x01;
	break;
	
    case LZ_GOAL_LAST_TARGET:/*末目标测距*/
		send_para_len = 1;
		lrd_send_buf.data_length = 0x03;
		lrd_send_buf.cmd_type = 0x03;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = 0x02;
	break;
	case LZ_GOAL_MULTI_TARGET:/*多目标测距*/
		send_para_len = 1;
		lrd_send_buf.data_length = 0x03;
		lrd_send_buf.cmd_type = 0x03;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = 0x03;
	break;

	case LZ_DETECT_FREQ_SET:/*连续测距频率设置*/
		send_para_len = 2;
		lrd_send_buf.data_length = 0x04;
		lrd_send_buf.cmd_type = 0xA1;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = cmd_para1;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = 0;
        break;

	case LZ_RANGE_GATE_VALUE_MIN:/*最小选通距离设置*/
		send_para_len = 2;
		lrd_send_buf.data_length = 0x04;
		lrd_send_buf.cmd_type = 0xA2;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = ((cmd_para1>>8)&0xff);
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = cmd_para1&0xff;
        break;

	case LZ_DETECT_DISTANCE_REQ_MIN:/*查询最小选通距离*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0xA3;
        break;

	case LZ_RANGE_GATE_VALUE_MAX:/*最大选通距离设置*/
		send_para_len = 2;
		lrd_send_buf.data_length = 0x04;
		lrd_send_buf.cmd_type = 0xA4;
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = ((cmd_para1>>8)&0xff);
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = cmd_para1&0xff;
        break;

	case LZ_DETECT_DISTANCE_REQ_MAX:/*查询最大选通距离*/
		lrd_send_buf.data_length = 0x02;
		lrd_send_buf.cmd_type = 0xA5;
        break;

    default:
		return false;
        break;
    }
	memcpy(dyc_laser_send_buf,(uint8_t *)&lrd_send_buf,DYC_SEND_HEADER_LEN);
	
	if (send_para_len == 0)
	{
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN] = UTL_ADD_CHECK((uint8_t*)&dyc_laser_send_buf[3],DYC_SEND_HEADER_LEN-3);
	}
	else if (send_para_len == 1)
	{
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 1] = UTL_ADD_CHECK((uint8_t*)&dyc_laser_send_buf[3],DYC_SEND_HEADER_LEN-2);
	}
	else if (send_para_len == 2)
	{
		dyc_laser_send_buf[DYC_SEND_HEADER_LEN + 2] = UTL_ADD_CHECK((uint8_t*)&dyc_laser_send_buf[3],DYC_SEND_HEADER_LEN-1);
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
#if CLI_INCLUDE
    DEBUG_LASER_TX_PRINT("\r\n LASER_SEND:");

    for(uint16_t i = 0; i < 11; i++)
    {
        DEBUG_LASER_TX_PRINT("%02x ", dyc_laser_send_buf[i]);
    }
#endif
    COM_API_Send_Data(COM_LASER_IN, dyc_laser_send_buf, sizeof(dyc_laser_send_buf));
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
    uint32_t detect_distance = 0;
    uint16_t detect_code = 0;

    data_add = UTL_ADD_CHECK((uint8_t *)&data[3], (length - 4));

    //校验计算
    if(data[length - 1] != data_add)
    {
#if CLI_INCLUDE
		DEBUG_LASER_RX_PRINT("\r\n add_err! data_add:%x  data[length - 1]:%x",data_add,data[length - 1]);
#endif
        return RECV_DATA_XOR_ERR;
    }
	
	SYS_LASER_STA_T* local_laser_sta;
    local_laser_sta = CONFIG_Get_Laser_Sta();

    //头校验
	if ((data[0] != LASER_DATA_IN_HEADER_1)&&(data[1] != LASER_DATA_IN_HEADER_2))
	{
		return RECV_DATA_HEAD_ERR;
	}
	
    if (data[4] == 0x02 || data[4] == 0x04)/*单次/连续测距*/
    {
		uint16_t temp = ((uint16_t)data[6] << 8)|data[7];
		float temp2 = temp + (data[8]*0.01);
		local_laser_sta->distance[0] = temp2;
#if CLI_INCLUDE
		DEBUG_LASER_RX_PRINT("\r\n distance[0]: %f",local_laser_sta->distance[0]);
#endif
    }

	switch(data[4])//状态位
	{
		case 0:
			local_laser_sta->distance[0] = 0;
			local_laser_sta->sys_sta = LASER_WORK_STAY;
#if CLI_INCLUDE
			DEBUG_LASER_RX_PRINT("\r\n sys_sta:%d",local_laser_sta->sys_sta);
#endif
		break;

		case 0x01://设备自检回报报文
			local_laser_sta->return_intensity = data[6];/*回波强度*/
			
			if (APP_IS_BIT_SET(data[7],6))
			{
				local_laser_sta->laser_sta.main_lobe = 1;
			}
			else
			{
				local_laser_sta->laser_sta.main_lobe = 0;
			}

			memcpy(&local_laser_sta->laser_sta, &data[7], 1);/*自检状态*/
			local_laser_sta->LZ_Powersta = data[8];		/*电源状态*/
#if CLI_INCLUDE
			DEBUG_LASER_RX_PRINT("\r\n intensity: %d; laser_sta: %d,LZ_Powersta: %d",
			local_laser_sta->return_intensity,
			local_laser_sta->laser_sta.last_dest,
			local_laser_sta->LZ_Powersta);
#endif
			break;

		default:
			break;
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


