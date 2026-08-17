#include "Driver/drv_uart.h"

#if CLI_INCLUDE
#include "../cli/cli_cmd_line.h"
#endif
#if (IR_CTRL_INCLUDE&IR_LGCS123)
#include "../ir/ir_ctrl_lgcs123.h"
#include "Common/utl_check.h"

#define IR_LGCS_RCV_DATA_1 0x55
#define IR_LGCS_RCV_DATA_2 0x04
#define IR_LGCS_SEND_DATA_1 0xAA
#define IR_LGCS_SEND_DATA_2 0x05
#define IR_LGCS_SEND_DATA_3 0x01

#define IR_LGCS_SEND_DATA_TAIL1 0xEB

static COM_RECV_INFO_T ir_lgcs_rcv_buf;

uint8_t shutter_adjust[9]= {0xAA,0x05,0x00,0x16,0x01,0x00,0xC6,0xEB,0xAA};
uint8_t shutter_off[9]= {0xAA,0x05,0x01,0x1,0x01,0x00,0xE2,0xEB,0xAA};
uint8_t contrast_read[8]= {0xAA,0x04,0x01,0x21,0x00,0xd0,0xEB,0xAA};
uint8_t light_read[8]= {0xAA,0x04,0x01,0x23,0x00,0xd2,0xEB,0xAA};
uint8_t xjh_send[10]= {0xAA,0x06,0x10,0x00,0x01,0x05,0x00,0xc6,0xEB,0xAA};

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_LGCS_API_Serial_Data_Init(void)
{
	ir_lgcs_rcv_buf.header1 = IR_LGCS_RCV_DATA_1;
	ir_lgcs_rcv_buf.tail1 = IR_LGCS_SEND_DATA_1;
	ir_lgcs_rcv_buf.data_recv_len = 9;
	COM_Rcv_SerialPort_Init(COM_IR_LGCS, ir_lgcs_rcv_buf.header1,ir_lgcs_rcv_buf.tail1,ir_lgcs_rcv_buf.data_recv_len);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外数据发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Lgcs_Light_Data_SendHandle(uint16_t light)
{
    //AA 06 01 23 01 X1 X2 X3 EB AA
    uint8_t light_data_send_buf[10]= {0xAA,0x06,0x01,0x23,0x01};
    uint8_t sum_check;

    memcpy(&light_data_send_buf[5],&light,sizeof(light));

    sum_check = UTL_ADD_CHECK(light_data_send_buf,sizeof(light_data_send_buf)-3);
    light_data_send_buf[7] = sum_check;
    light_data_send_buf[8] = IR_LGCS_SEND_DATA_TAIL1;
    light_data_send_buf[9] = IR_LGCS_SEND_DATA_1;

    COM_API_Send_Data(COM_IR_LGCS,light_data_send_buf,sizeof(light_data_send_buf));
    
    return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:红外指令发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Lgcs_Cmd_Data_SendHandle(uint8_t cmd,uint8_t adjust)
{
    IR_LGCS_DATA_T lgcs_send_buf;
    uint8_t sum_check;
    memset(&lgcs_send_buf,0x0,IR_LGCS_DATA_LEN);
    lgcs_send_buf.Header1 = IR_LGCS_SEND_DATA_1;
    lgcs_send_buf.Header2 = IR_LGCS_SEND_DATA_2;
    lgcs_send_buf.ctrl_cmd = IR_LGCS_SEND_DATA_3;
    
    switch (cmd)
    {
    case IR_DDE_RANGE://0-手动；1~8-挡位
        if (adjust > 8)
        {
            return RECV_DATA_OTHERR_ERR;
        }
        lgcs_send_buf.ctrl_data1 = 0x19;
        lgcs_send_buf.ctrl_data2 = 0x1;
        lgcs_send_buf.ctrl_data3 = adjust;
        break;

    case IR_CONTRAST_CTRL:
        lgcs_send_buf.ctrl_data1 = 0x21;
        lgcs_send_buf.ctrl_data2 = 0x2;
        lgcs_send_buf.ctrl_data3 = adjust;
        break;

    case IR_GAIN_CTRL:
        lgcs_send_buf.ctrl_data1 = 0x1E;
        lgcs_send_buf.ctrl_data2 = 0x2;
        lgcs_send_buf.ctrl_data3 = adjust;
        break;

    case IR_BLACK_WHITE://0-白，1黑
        lgcs_send_buf.ctrl_data1 = 0x42;
        lgcs_send_buf.ctrl_data2 = 0x2;
        lgcs_send_buf.ctrl_data3 = adjust;
        break;

    default:

        break;
    }

    sum_check = UTL_ADD_CHECK((uint8_t *)&lgcs_send_buf,sizeof(lgcs_send_buf)-3);
    lgcs_send_buf.ctrl_data4 = sum_check;
    lgcs_send_buf.tail_1 = IR_LGCS_SEND_DATA_TAIL1;
    lgcs_send_buf.tail_2 = IR_LGCS_SEND_DATA_1;


    COM_API_Send_Data(COM_IR_LGCS,(uint8_t *)&lgcs_send_buf,sizeof(lgcs_send_buf));
    
    return 0;
}
bool ir_lgcs_contrast_read_sta = false;
static void IR_STATUS_ReadHandle(uint8_t read_type)
{
	if (read_type == 4)
	{
		ir_lgcs_contrast_read_sta = true;
	   COM_API_Send_Data(COM_IR_LGCS,(uint8_t *)contrast_read,sizeof(contrast_read));
	}
	else if (read_type == 5)
	{
		  COM_API_Send_Data(COM_IR_LGCS,(uint8_t *)light_read,sizeof(light_read));
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:dde挡位：0-8
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool ir_lgcs_send_sta = false;
uint8_t Ir_Ctrl_Lgcs_Data_SendHandle(uint8_t send_type,uint8_t *data)
{
    IR_LGCS_DATA_T lgcs_send_buf = {0};
    uint16_t temp_buf = 0;
    
    switch(send_type)
    {
    case IR_CALIBRATION:/*图像校正*/
        if(data[0] == 1)//快门校正
        {
            memcpy(&lgcs_send_buf,shutter_adjust,IR_LGCS_DATA_LEN);
        }
        else if(data[0] == 0)//快门常闭
        {
            memcpy(&lgcs_send_buf,shutter_off,IR_LGCS_DATA_LEN);
        }
        COM_API_Send_Data(COM_IR_LGCS,(uint8_t *)&lgcs_send_buf,sizeof(lgcs_send_buf));
    break;

    case IR_CONTRAST_CTRL:
    case IR_DDE_RANGE:
    case IR_GAIN_CTRL:
    case IR_BLACK_WHITE:
        IR_Ctrl_Lgcs_Cmd_Data_SendHandle(send_type,data[1]);
        break;

    case IR_SYS_STA_REQ:
        //由于对比度设置和读取的返回值，当设置为1的时候，返回值一样，因此需要对其进行鉴别判断。
        IR_STATUS_ReadHandle(data[0]);
        break;

    case IR_LIGHT_CTRL:
        memcpy(&temp_buf,(uint16_t *)&data[1],sizeof(temp_buf));
        IR_Ctrl_Lgcs_Light_Data_SendHandle(temp_buf);
        break;
    case IR_DETAIL_XJH:
        COM_API_Send_Data(COM_IR_LGCS,(uint8_t *)xjh_send,sizeof(xjh_send));
        break;
    default:
        return RECV_DATA_OTHERR_ERR;
	break;
    }
    ir_lgcs_send_sta = true;
    
    return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:姿态通信协议
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_LGCS_PROCESS_Get(uint8_t *data,uint32_t length)
{
//	IR_LGCS_RCV_DATA_T rcv_ir_info;
    uint8_t sum_check = 0;

    SYS_IR_STA_T main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    //由于该红外机芯还有别的回报报文上报，将触发解析，因此通过发送报文来约束接收
    //如果别的报文回报，先于设置报文对应的回报报文，则该处理方式还是不够完善;如果需要，请放开这段屏蔽报文
//    if (ir_lgcs_send_sta == false)
//    {
//        return false;
//    }

    //长度校验
    if (length > (IR_LGCS_RCV_DATA_LEN + 1))
    {
        return false;
    }

    //校验尾
	if ((data[length-1] != 0xAA)&&(data[length-2] != 0xEB))
	{
		return false;
	}
    switch (data[2])
    {
    case IR_SHUTTER_SET:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\nshutter set success");
#endif
        break;

    case IR_LGCS_SHUTTER_OFF:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\nshutter off success");
#endif
        break;

    case IR_DDE_SET:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\nDDE set success");
#endif
        break;

    case IR_DETIAL_GIAN:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\ndetial gain set success");
#endif
        break;

    case IR_POLARITY_SET:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\npolarity set success");
#endif
        break;

    case IR_LGCS_CONTRAST_SET://01 AE
        if (ir_lgcs_contrast_read_sta)//当其为true时，则说明是读指令，因此直接将读回来的值进行赋值。
        {
            main_ir_info.contrast_level = data[4];
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\ncontrast read success");
#endif
            ir_lgcs_contrast_read_sta = false;
        }
        else
        {
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\ncontrast set success");
#endif
        }
        break;

    case IR_LIGHT_SET:
        if ((data[4] == 0x01)&&(data[5] == 0xb0))
        {
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\n set success");
#endif
        }
        else
        {
            memcpy(&main_ir_info.contrast_level,&data[4],sizeof(main_ir_info.contrast_level));
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\n set success");
#endif
        }
        break;


    default:
#if CLI_INCLUDE
        DEBUG_IR_PRINT("\r\n set fali");
#endif
        break;

    }
    //正确接收解算之后，要对状态位进行置位。
    ir_lgcs_send_sta = false;
    CONFIG_Set_Ir_Info(main_ir_info);
    
    return 0;
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
RECV_DATA_ERR_STA IR_API_Lgcs_Period_Handle(void)
{
	uint16_t recv_len = 0;
    
	recv_len = COM_REC_DataAnalysis_nocheck(COM_IR_LGCS,ir_lgcs_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
	IR_LGCS_PROCESS_Get(ir_lgcs_rcv_buf.recv_buf,recv_len);

	return RECV_DATA_SUC;

}

#endif
