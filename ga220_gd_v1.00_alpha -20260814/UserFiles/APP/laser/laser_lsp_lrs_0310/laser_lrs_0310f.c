#include "Driver/drv_uart.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#include "Common/utl_check.h"
/*==============================================================
波特率 115200；
LSP-LRS-0410A，LSP-LRS-0610A 协议一致
*==============================================================*/
#if LASER_LRS_0310F
#include "laser_lrs_0310f.h"
#include "Common/config.h"
#include "Bsp/SEGGER_RTT.h"

#define LASER_DATA_IN_HEADER_1 0x55

COM_RECV_INFO_T laser_serial_rcv_buf;

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
//    SYS_LASER_STA_T laser_info = {0};
//    laser_info = CONFIG_Get_Laser_Sta();
//    laser_info.light_cnt = 1000;//激光默认照射批量为1000；
    laser_serial_rcv_buf.header1 = LASER_DATA_IN_HEADER_1;
    //和校验取反作为尾
    laser_serial_rcv_buf.tail1 = 0;
    laser_serial_rcv_buf.data_recv_len = 6;
    COM_Rcv_SerialPort_Init(COM_LASER_IN, laser_serial_rcv_buf.header1,
                            laser_serial_rcv_buf.tail1, laser_serial_rcv_buf.data_recv_len);
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
uint32_t laser_distance = 0;
LZ_LRS_SEND_T lrd_send_buf = {0};
uint8_t lrd_laser_sta = 0x0F;
static uint8_t _0310f_cmd = 0;
uint8_t Laser_Ctrl_SendHandle(SYS_LASER_DETECT_MODE_E cmd, uint16_t cmd_para1,
                              uint16_t cmd_para2)
{
    memset(&lrd_send_buf, 0, sizeof(LZ_LRS_SEND_T));
    lrd_send_buf.header = LASER_DATA_IN_HEADER_1;
	_0310f_cmd = cmd;
    switch (cmd)
    {
	case LZ_DETEC_STOP://待机
        lrd_send_buf.cmd_type1 = 0x00;
        lrd_send_buf.cmd_type2 = 0x02;
        lrd_send_buf.set_data1 = 0x00;
        lrd_send_buf.set_data2 = 0x00;

        break;
	
	case LZ_DETECT_SELFCHECK://自检
		lrd_send_buf.cmd_type1 = 0x03;
        lrd_send_buf.cmd_type2 = 0x02;
		break;

//    case LZ_DETECT_DISTANCE_REQ://距离查询
//        lrd_send_buf.cmd_type1 = 0x28;
//        lrd_send_buf.cmd_type2 = 0x02;
//        lrd_send_buf.set_data1 = 0x00;
//        lrd_send_buf.set_data2 = 0x00;
//        break;

    case LZ_SINGLE_DETECT:/*单次测距*/
        lrd_send_buf.cmd_type1 = 0x01;
        lrd_send_buf.cmd_type2 = 0x02;
        break;

    case LZ_DETECT_FREQ_1:/*1hz测距*/
        lrd_send_buf.cmd_type1 = 0x02;
        lrd_send_buf.cmd_type2 = 0x02;
        lrd_send_buf.set_data1 = 0x03;
        lrd_send_buf.set_data2 = 0xe8;
        break;

    case LZ_DETECT_FREQ_5:/*5hz测距*/
        lrd_send_buf.cmd_type1 = 0x02;
        lrd_send_buf.cmd_type2 = 0x02;
        lrd_send_buf.set_data1 = 0x00;
        lrd_send_buf.set_data2 = 0xC8;
        break;

    case LZ_DETECT_FREQ_10:/*10hz测距*/
        lrd_send_buf.cmd_type1 = 0x02;
        lrd_send_buf.cmd_type2 = 0x02;
        lrd_send_buf.set_data1 = 0x00;
        lrd_send_buf.set_data2 = 0x64;
        break;

	case LZ_DETECT_FREQ_20:/*20hz测距*/
        lrd_send_buf.cmd_type1 = 0x02;
        lrd_send_buf.cmd_type2 = 0x02;
        lrd_send_buf.set_data1 = 0x00;
        lrd_send_buf.set_data2 = 0x32;
        break;

//    case LZ_DETECT_CODE_REQ:/*模组序列号读取*/
//        lrd_send_buf.cmd_type1 = 0x0D;
//        lrd_send_buf.cmd_type2 = 0x02;
//        lrd_send_buf.set_data1 = 0x0D;
//        lrd_send_buf.set_data2 = 0x0D;
//        break;

	case LZ_VERSION: /*编号查询*/
        lrd_send_buf.cmd_type1 = 0xEB;
        lrd_send_buf.cmd_type2 = 0x02;
        break;
	
	case LZ_RANGE_GATE_VALUE: /*选通值设置*/
		lrd_send_buf.cmd_type1 = 0x04;
        lrd_send_buf.cmd_type2 = 0x02;
		lrd_send_buf.set_data1 = (cmd_para1>>8);
        lrd_send_buf.set_data2 = cmd_para1&0xff;

		break;
    default:
        break;
    }

    lrd_send_buf.sum_check = UTL_XOR_CHECK((uint8_t*)&lrd_send_buf,5);

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
LASER_RECV_DETECT_T lrd_recv_buf = {0};
uint8_t laser_process_lrs_data_in(uint8_t *data, uint32_t length)
{
    SYS_LASER_STA_T* local_laser_sta;
    uint8_t data_xor;
    uint32_t detect_distance = 0;
    uint16_t detect_code = 0;
    data_xor = UTL_XOR_CHECK(data, (length - 1));

    //校验计算
    if(data[length - 1] != data_xor)
    {
        return RECV_DATA_XOR_ERR;
    }

    local_laser_sta = CONFIG_Get_Laser_Sta();
	if (data[0] == LASER_DATA_IN_HEADER_1)
	{
		switch(_0310f_cmd)
		{
			case LZ_DETEC_STOP: //停止
				local_laser_sta->distance[0] = 0;
				local_laser_sta->sys_sta = LASER_WORK_STAY;
				break;
			case LZ_DETECT_SELFCHECK: //自检
				local_laser_sta->range_gate_value = (uint16_t)data[5] << 8 | data[6];
				break;
			case LZ_SINGLE_DETECT:
			{
				uint32_t temp = ((uint32_t)data[4] << 16)|(uint16_t)data[5] << 8 | data[6];
				float temp2 = temp *0.1;
				local_laser_sta -> distance[0] = temp2;
				local_laser_sta -> laser_sta.main_lobe = (data[3] >> 7) & 0x01;
				local_laser_sta -> laser_sta.echo_sta = (data[3] >> 6) & 0x01;
				local_laser_sta -> sys_sta = LASER_WORK_SINGLE;
			}
				break;
			case LZ_DETECT_FREQ_1:
			{
				uint32_t temp = ((uint32_t)data[4] << 16)|(uint16_t)data[5] << 8 | data[6];
				float temp2 = temp *0.1;
				local_laser_sta -> distance[0] = temp2;
				local_laser_sta -> laser_sta.main_lobe = (data[3] >> 7) & 0x01;
				local_laser_sta -> laser_sta.echo_sta = (data[3] >> 6) & 0x01;
				local_laser_sta -> sys_sta = LASER_STA_1HZ;
			}
				break;
			case LZ_DETECT_FREQ_5:
			{
				uint32_t temp = ((uint32_t)data[4] << 16)|(uint16_t)data[5] << 8 | data[6];
				float temp2 = temp *0.1;
				local_laser_sta -> distance[0] = temp2;
				local_laser_sta -> laser_sta.main_lobe = (data[3] >> 7) & 0x01;
				local_laser_sta -> laser_sta.echo_sta = (data[3] >> 6) & 0x01;
				local_laser_sta -> sys_sta = LASER_WORK_5HZ;
			}
				break;
			case LZ_DETECT_FREQ_10:
			{
				uint32_t temp = ((uint32_t)data[4] << 16)|(uint16_t)data[5] << 8 | data[6];
				float temp2 = temp *0.1;
				local_laser_sta -> distance[0] = temp2;
				local_laser_sta -> laser_sta.main_lobe = (data[3] >> 7) & 0x01;
				local_laser_sta -> laser_sta.echo_sta = (data[3] >> 6) & 0x01;
				local_laser_sta -> sys_sta = LASER_WORK_10HZ;
			}
				break;
			case LZ_DETECT_FREQ_20:
			{
				uint32_t temp = ((uint32_t)data[4] << 16)|(uint16_t)data[5] << 8 | data[6];
				float temp2 = temp *0.1;
				local_laser_sta -> distance[0] = temp2;
				local_laser_sta -> laser_sta.main_lobe = (data[3] >> 7) & 0x01;
				local_laser_sta -> laser_sta.echo_sta = (data[3] >> 6) & 0x01;
				local_laser_sta -> sys_sta = LASER_WORK_20HZ;
			}
				break;
			case LZ_VERSION: //激光编号
				local_laser_sta -> codenum = (uint16_t)data[15] << 8 | data[16];
				break;
			case LZ_RANGE_GATE_VALUE: /*选通值设置*/
				local_laser_sta -> range_gate_value = (uint16_t) data[3] << 8 | data[4];
				break;
		}
	}
	
//    CONFIG_Set_Laser_Sta(local_laser_sta);
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

    for(int i = 0; i < recv_len; i++)
    {
        DEBUG_LASER_RX_PRINT("%02x ", laser_serial_rcv_buf.recv_buf[i]);
    }

#endif
    laser_process_lrs_data_in(laser_serial_rcv_buf.recv_buf, recv_len);
    return RECV_DATA_SUC;
}
#endif


