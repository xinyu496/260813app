#include "APP/ir/ir_focus_hy5050e_2c.h"
#include "Common/utl_check.h"
#include "Common/utl_math.h"
#include "Common/config.h"
#include "Common/opt_cmd.h"
#include "Driver/drv_uart.h"
IR_HY5050E_DATA_T hy5050e_send_buf;

uint8_t Ir_Ctrl_hy5050e_Cmd_Data_send(uint8_t cmd, uint16_t param)
{
	memset(&hy5050e_send_buf, 0, sizeof(IR_HY5050E_DATA_T));
    hy5050e_send_buf.sync = 0xFF;// 帧头
    hy5050e_send_buf.addr = 0x01;
    switch(cmd)
    {
//        case IR_FOCUS_CTRL:/*单步调焦*/
//			hy5050e_send_buf.cmd2 = 0x07;
//			if(param == 1)
//			{/*+*/
//				hy5050e_send_buf.data2 = 0x58;
//			}
//			else if(param == 2)
//			{/*-*/
//				hy5050e_send_buf.data2 = 0x57;
//			}else if(param == 3)/*停*/
//			{
//			}
//            break;
//		case IR_ZOOM_CTRL:/*单步变倍*/
//			hy5050e_send_buf.cmd2 = 0x07;
//			if(param == 1)
//			{/*+*/
//				hy5050e_send_buf.data2 = 0x56;
//			}
//			else if(param == 2)
//			{/*-*/
//				hy5050e_send_buf.data2 = 0x55;
//			}
//			else if(param == 3)/*停*/
//			{
//			}
//			break;
		case IR_CH_SMA://指定变倍定位 4096
			hy5050e_send_buf.cmd2 = 0x4f;
			hy5050e_send_buf.data1 = 0x10;
			hy5050e_send_buf.data2 = 0x00;
			break;
		case IR_CH_MI://指定变倍定位 1500
			hy5050e_send_buf.cmd2 = 0x4f;
			hy5050e_send_buf.data1 = 0x05;
			hy5050e_send_buf.data2 = 0xdc;
			break;
		case IR_CH_LARGE://指定变倍定位 0
			hy5050e_send_buf.cmd2 = 0x4f;
			hy5050e_send_buf.data1 = 0x00;
			hy5050e_send_buf.data2 = 0x00;
			break;
//		case IR_ZOOM_CTRL:/*变倍*/
//			hy5050e_send_buf.cmd2 = 0x7;
//			if(param == 1)
//			{/*+*/
//				hy5050e_send_buf.data2 = 0x5a;
//			}
//			else if(param == 2)
//			{/*-*/
//				hy5050e_send_buf.data2 = 0x59;
//			}
////			else if(param == 3)/*停*/
////			{
////				hy5050e_send_buf.cmd2 = 0;
////			}
//			break;
		case IR_ZOOM_CTRL:/*变倍*/
			if(param == 1)
			{/*+*/
				hy5050e_send_buf.cmd2 = 0x20;
			}
			else if(param == 2)
			{/*-*/
				hy5050e_send_buf.cmd2 = 0x40;
			}
			else if(param == 3)/*停*/
			{
				hy5050e_send_buf.cmd2 = 0;
			}
			break;
		case IR_FOCUS_CTRL:/*调焦*/
			if(param == 1)
			{/*+*/
				hy5050e_send_buf.cmd1 = 0x01;
			}
			else if(param == 2)
			{/*-*/
				hy5050e_send_buf.cmd2 = 0x80;
			}
			else if(param == 3)/*停*/
			{
				hy5050e_send_buf.cmd2 = 0;
			}
			break;
		case IR_FOCUS_ONE_SET:
			if(param == 0)
			{
				/*开启自动聚焦*/
				hy5050e_send_buf.cmd2 = 0x07;
				hy5050e_send_buf.data2 = 0x42;
			}
			else if (param == 1)
			{
				/*关闭自动聚焦*/
				hy5050e_send_buf.cmd2 = 0x07;
				hy5050e_send_buf.data2 = 0x41;
			}
			break;
		case IR_FOCUS_AUTO:///*触发一次自动聚焦*/
			hy5050e_send_buf.cmd1 = 0x07;
			hy5050e_send_buf.data1 = 0xA2;
			CONFIG_Set_Master_Ctrl_Cmd(MASTER_IR_PARA,0);
		break;
			
        default:
            break;
    }

	hy5050e_send_buf.checksum = UTL_ADD_CHECK(&hy5050e_send_buf.addr, 
						sizeof(hy5050e_send_buf)-2);
	
    COM_API_Send_Data(COM_IR_FOCUS, (uint8_t *)&hy5050e_send_buf,
                      sizeof(hy5050e_send_buf));
    return 0;
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
uint16_t hy5050_ir_focus = 0;//焦距值
uint8_t ir_hy5050e_process_data_in(uint8_t *data, uint32_t length)
{
//    SYS_IR_STA_T* main_ir_info;
//    main_ir_info = CONFIG_Get_Ir_Info();

    //由于该红外机芯还有别的回报报文上报，将触发解析，因此通过发送报文来约束接收
    //如果别的报文回报，先于设置报文对应的回报报文，则该处理方式还是不够完善;如果需要，请放开这段屏蔽报文

    //校验--TODO
    if ((data[0] == 0xFF) && (data[1] == 0x01) && (data[2] == 0x63))
    {
		hy5050_ir_focus = (uint16_t)((data[4]<<8)|data[5]);
    }
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
uint8_t hy5050e_recv_data[7] = {0};
RECV_DATA_ERR_STA IR_API_Hy5050e_Period_Handle(void)
{
    uint16_t recv_len = 0;
    recv_len = COM_REC_Data_Direct(COM_IR_FOCUS, hy5050e_recv_data);

    if (recv_len == 0)
    {
        return RECV_DATA_NULL;
    }

    ir_hy5050e_process_data_in(hy5050e_recv_data, recv_len);
    return RECV_DATA_SUC;
}


