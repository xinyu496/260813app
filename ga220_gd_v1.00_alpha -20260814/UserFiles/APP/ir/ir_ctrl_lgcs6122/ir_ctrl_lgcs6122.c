#include "Driver/drv_uart.h"

#if CLI_INCLUDE
    #include "../cli/cli_cmd_line.h"
#endif
#if (IR_CTRL_INCLUDE&IR_LGCS6122)
#include "APP/ir/ir_ctrl_lgcs6122.h"
#include "Common/utl_check.h"
#include "Common/utl_math.h"
#include "APP/ir/ir_focus_hy5050e_2c.h"

/*文件说明:
该协议发送包为9/12个字节；帧头固定为0xAA;帧尾固定为0xEB,0xAA;
帧尾前面为一个字节的校验数据；校验方式：前面所有字节的校验和取低八位；

该协议接收包为8/12字节不定长。帧头固定为0x55;帧尾固定为0xEB,0xAA;
*/

#define IR_LGCS_RCV_DATA_1 0x55
#define IR_LGCS_RCV_DATA_2 0x04
#define IR_LGCS_TAIL_1 0xEB
#define IR_LGCS_TAIL_2 0xAA
#define IR_LGCS_SEND_DATA_1 0xAA
#define IR_LGCS_SEND_DATA_2 0x05
#define IR_LGCS_SEND_DATA_3 0x01

#define IR_LGCS_SEND_DATA_TAIL1 0xEB
static COM_RECV_INFO_T ir_lgcs_rcv_buf;

uint8_t contrast_read[8] = {0xAA, 0x04, 0x01, 0x21, 0x00, 0xd0, 0xEB, 0xAA};
uint8_t light_read[8] = {0xAA, 0x04, 0x01, 0x26, 0x00, 0xd5, 0xEB, 0xAA};
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
    ir_lgcs_rcv_buf.tail1 = IR_LGCS_TAIL_1;
    ir_lgcs_rcv_buf.tail2 = IR_LGCS_TAIL_2;
    ir_lgcs_rcv_buf.data_recv_len = 8;
    COM_Rcv_SerialPort_Init(COM_IR_LGCS6122, ir_lgcs_rcv_buf.header1,
                            (ir_lgcs_rcv_buf.tail1 << 8 | ir_lgcs_rcv_buf.tail1),
                            ir_lgcs_rcv_buf.data_recv_len);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:电子变倍
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Ir_Ctrl_Lgcs_Ele_Zoom_send(uint8_t value)
{
    //AA 0C 01 40 02 18 01 E0 00 67 01 1F 01 7A EB AA
    uint8_t zoom_send_buf1[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0, 0, 0, 0, 0x7f, 0x02, 0xFF, 0x01, 0x7A, 0xEB, 0xAA};
    uint8_t zoom_send_buf2[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xA0, 0, 0x80, 0, 0xDF, 0x01, 0x7F, 0x01, 0x79, 0xEB, 0xAA};
    uint8_t zoom_send_buf3[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xD5, 0, 0xAB, 0, 0xA9, 0x01, 0x54, 0x01, 0x78, 0xEB, 0xAA};
    uint8_t zoom_send_buf4[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xf0, 0, 0xc0, 0, 0x8f, 0x01, 0x3f, 0x01, 0x79, 0xEB, 0xAA};
    uint8_t zoom_send_buf5[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0, 0x01, 0xcd, 0, 0x7f, 0x01, 0x32, 0x01, 0x7A, 0xEB, 0xAA};
    uint8_t zoom_send_buf6[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0xb, 1, 0xd5, 0, 0x74, 0x01, 0x29, 0x01, 0x79, 0xEB, 0xAA};
    uint8_t zoom_send_buf7[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x12, 1, 0xdb, 0, 0x6c, 0x01, 0x23, 0x01, 0x78, 0xEB, 0xAA};
    uint8_t zoom_send_buf8[16] = {0xAA, 0x0C, 0x01, 0x40, 0x02, 0x18, 1, 0xe0, 0, 0x67, 0x01, 0x1f, 0x01, 0x7A, 0xEB, 0xAA};

    switch (value)
    {
        case 1:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf1, sizeof(zoom_send_buf1));
            break;

        case 2:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf2, sizeof(zoom_send_buf2));
            break;

        case 3:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf3, sizeof(zoom_send_buf3));
            break;

        case 4:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf4, sizeof(zoom_send_buf4));
            break;

        case 5:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf5, sizeof(zoom_send_buf5));
            break;

        case 6:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf6, sizeof(zoom_send_buf6));
            break;

        case 7:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf7, sizeof(zoom_send_buf7));
            break;

        case 8:
            COM_API_Send_Data(COM_IR_LGCS6122, zoom_send_buf8, sizeof(zoom_send_buf8));
            break;

        default:
            break;
    }

    return 0;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:部分指令设置
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t enhance_rank = 0;
IR_LGCS_DATA_T lgcs_send_buf;
uint8_t lgcs_contrast_level = 100;
uint8_t lgcs_ligth_level = 100;
uint8_t Ir_Ctrl_Lgcs_Cmd_Data_send(uint8_t cmd, uint8_t *adjust)
{
    
//    uint8_t sum_check;
    memset(&lgcs_send_buf, 0x0, IR_LGCS_DATA_LEN);
    lgcs_send_buf.Header1 = IR_LGCS_SEND_DATA_1;
    lgcs_send_buf.Header2 = IR_LGCS_SEND_DATA_2;
    lgcs_send_buf.ctrl_cmd = IR_LGCS_SEND_DATA_3;
	
	SYS_IR_STA_T* ir_info = CONFIG_Get_Ir_Info();

    switch (cmd)
    {
        case IR_IMAGE_ENHANCE://0-手动；1~8-挡位
            if (adjust[0] > 5)
            {
                return RECV_DATA_OTHERR_ERR;
            }
			if (adjust[0] == 1)//开
			{
				enhance_rank = 1;
			}
			else if (adjust[0] == 2)//关
			{
			  enhance_rank = 0;
			}
			else if (adjust[0] == 3)//加
			{
				if (enhance_rank == 4)
				{
					enhance_rank = 4;
				}
				else
				{
					enhance_rank++;
				}
			}
			else if (adjust[0] == 4)//减
			{
			  	if (enhance_rank == 0)
				{
					enhance_rank = 0;
				}
				else
				{
					enhance_rank--;
				}
			}
			lgcs_send_buf.ctrl_data3 = enhance_rank;
            lgcs_send_buf.ctrl_data1 = 0x19;
            lgcs_send_buf.ctrl_data2 = 0x1;
            break;

        case IR_BLACK_WHITE://0-白，1黑
            lgcs_send_buf.ctrl_data1 = 0x42;
            lgcs_send_buf.ctrl_data2 = 0x2;
            lgcs_send_buf.ctrl_data3 = adjust[0];
            break;

        case IR_CONTRAST_CTRL:
			//AA 05 01 21 02 X1 X2 EB AA；返回：55 04 21 33 01 AE EB AA
			lgcs_send_buf.ctrl_data1 = 0x21;
			lgcs_send_buf.ctrl_data2 = 0x02;
			if(adjust[0] == 0x01)
			{/*+*/
				if (lgcs_contrast_level == 250)
				{
					lgcs_contrast_level = 250;
				}
				else
				{
					lgcs_send_buf.ctrl_data3 = lgcs_contrast_level + 10;
					lgcs_contrast_level = lgcs_send_buf.ctrl_data3;
				}
			}
			else if(adjust[0] == 0x02)
			{/*-*/
				if (lgcs_contrast_level == 10)
				{
					lgcs_contrast_level = 10;
				}
				else
				{
					lgcs_send_buf.ctrl_data3 = lgcs_contrast_level - 10;
					lgcs_contrast_level = lgcs_send_buf.ctrl_data3;
				}
			}
			else if(adjust[0] == 0x03)
			{
				lgcs_send_buf.ctrl_data3 = 100;
			}
            break;

        case IR_LIGHT_CTRL:
			//AA 05 01 26 01 X1 X2 EB AA;返回：55 04 26 33 01 B3 EB AA
            lgcs_send_buf.ctrl_data1 = 0x26;
            lgcs_send_buf.ctrl_data2 = 0x01;
            if(adjust[0] == 0x01)
			{/*+*/
				if (lgcs_ligth_level == 250)
				{
					lgcs_ligth_level = 250;
				}
				else
				{
					lgcs_send_buf.ctrl_data3 = lgcs_ligth_level + 10;
					lgcs_ligth_level = lgcs_send_buf.ctrl_data3;
				}
			}
			else if(adjust[0] == 0x02)
			{/*-*/
				if (lgcs_ligth_level == 10)
				{
					lgcs_ligth_level = 10;
				}
				else
				{
					lgcs_send_buf.ctrl_data3 = lgcs_ligth_level - 10;
					lgcs_ligth_level = lgcs_send_buf.ctrl_data3;
				}
			}
			else if(adjust[0] == 0x03)
			{
				lgcs_send_buf.ctrl_data3 = 100;
			}
            break;

        default:
            break;
    }
//	CONFIG_Set_Ir_Info(ir_info);
    lgcs_send_buf.add_check = UTL_ADD_CHECK((uint8_t *)&lgcs_send_buf,
                                            sizeof(lgcs_send_buf) -3);
    lgcs_send_buf.tail_1 = IR_LGCS_SEND_DATA_TAIL1;
    lgcs_send_buf.tail_2 = IR_LGCS_SEND_DATA_1;
    COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)&lgcs_send_buf,
                      sizeof(lgcs_send_buf));
    return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:状态读取
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool ir_lgcs_contrast_read_sta = false;
void Ir_status_read(uint8_t read_type)
{
    if (read_type == IR_CONTRAST_CTRL)
    {
        ir_lgcs_contrast_read_sta = true;
        COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)contrast_read, sizeof(contrast_read));
    }
    else if (read_type == IR_LIGHT_CTRL)
    {
        COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)light_read, sizeof(light_read));
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:图像矫正
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
IR_LGCS_DATA_T lgcs_send_buf = {0};
static void Ir_Calibration_ctrl_Send(uint8_t *data)
{
    
    lgcs_send_buf.Header1 = IR_LGCS_SEND_DATA_1;
    lgcs_send_buf.Header2 = IR_LGCS_SEND_DATA_2;
    lgcs_send_buf.ctrl_cmd = 1;

    //if(APP_IS_BIT_SET(data[0], 4)) //单稳态快门
//	if (data[0] == 1)
//    {
//        lgcs_send_buf.ctrl_data1 = 2;
//        lgcs_send_buf.ctrl_data2 = 2;
//        lgcs_send_buf.ctrl_data3 = 4;
//    }
    //else if(!APP_IS_BIT_SET(data[0], 4)) //双稳态快门
	if (data[0] == 1)
    {
        lgcs_send_buf.ctrl_data1 = 2;
        lgcs_send_buf.ctrl_data2 = 2;
        lgcs_send_buf.ctrl_data3 = 1;
    }
    //else if (APP_IS_BIT_SET(data[0], 1)) //背景校正
//    {
//        lgcs_send_buf.ctrl_data1 = 2;
//        lgcs_send_buf.ctrl_data2 = 2;
//        lgcs_send_buf.ctrl_data3 = 0;
//    }
    else if (data[0] == 2) //非均匀校正 自动
    {
        lgcs_send_buf.ctrl_data1 = 1;
        lgcs_send_buf.ctrl_data2 = 1;
        lgcs_send_buf.ctrl_data3 = 1;
    }
    //else if (!APP_IS_BIT_SET(data[0], 6)) //非均匀校正 手动
	else if (data[0] == 3)
    {
        lgcs_send_buf.ctrl_data1 = 1;
        lgcs_send_buf.ctrl_data2 = 1;
        lgcs_send_buf.ctrl_data3 = 0;
    }
	else
	{
		return;
	}

    lgcs_send_buf.add_check = UTL_ADD_CHECK((uint8_t *)&lgcs_send_buf.Header1,
                                            (IR_LGCS_DATA_LEN - 3));
    lgcs_send_buf.tail_1 = IR_LGCS_TAIL_1;
    lgcs_send_buf.tail_2 = IR_LGCS_TAIL_2;

    COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)&lgcs_send_buf.Header1,
                      sizeof(lgcs_send_buf));
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:自检以及指令查询
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void Ir_Selfcheck_Send(uint8_t *data)
{
    IR_LGCS_DATA_T lgcs_send_buf = {0};
    lgcs_send_buf.Header1 = IR_LGCS_SEND_DATA_1;
    lgcs_send_buf.Header2 = IR_LGCS_SEND_DATA_2;
    lgcs_send_buf.ctrl_cmd = 1;

    if(APP_IS_BIT_SET(data[0], 4)) //单稳态快门
    {
        lgcs_send_buf.ctrl_data1 = 2;
        lgcs_send_buf.ctrl_data2 = 2;
        lgcs_send_buf.ctrl_data3 = 4;
    }

    lgcs_send_buf.add_check = UTL_ADD_CHECK((uint8_t *)&lgcs_send_buf.Header1,
                                            (IR_LGCS_DATA_LEN - 3));
    lgcs_send_buf.tail_1 = IR_LGCS_TAIL_1;
    lgcs_send_buf.tail_2 = IR_LGCS_TAIL_2;
#if COM_IR_LGCS6122_cmd_debug
    SEGGER_RTT_SetTerminal(1);
    SEGGER_RTT_printf(0, "COM_IR_LGCS6122 send:" );
    IR_LGCS_DATA_T *STRUCT = &lgcs_send_buf;

    for(uint8_t k = 0; k < sizeof(lgcs_send_buf) ; k++)
    {
        SEGGER_RTT_printf(0, "0x%x ", *STRUCT);
        STRUCT++;

        if(k == sizeof(lgcs_send_buf) - 1)
        {
            SEGGER_RTT_printf(0, " \n" );
        }
    }

    SEGGER_RTT_SetTerminal(0);
#endif
    COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)&lgcs_send_buf.Header1,
                      sizeof(lgcs_send_buf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:十字丝控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void Ir_Crosshair_Send(uint8_t *data)
{
    IR_LGCS_DATA_T lgcs_send_buf = {0};
    lgcs_send_buf.Header1 = IR_LGCS_SEND_DATA_1;
    lgcs_send_buf.Header2 = IR_LGCS_SEND_DATA_2;
    lgcs_send_buf.ctrl_cmd = 1;
    lgcs_send_buf.ctrl_data1 = 0x43;
    lgcs_send_buf.ctrl_data2 = 0x02;

    if(data[0] == 1)//十字开
    {
        lgcs_send_buf.ctrl_data3 = 0x80;
    }
    else if(data[0] == 2)//十字关
    {
        lgcs_send_buf.ctrl_data3 = 0;
    }

    lgcs_send_buf.add_check = UTL_ADD_CHECK((uint8_t *)&lgcs_send_buf.Header1,
                                            (IR_LGCS_DATA_LEN - 3));
    lgcs_send_buf.tail_1 = IR_LGCS_TAIL_1;
    lgcs_send_buf.tail_2 = IR_LGCS_TAIL_2;
    COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)&lgcs_send_buf.Header1,
                      sizeof(lgcs_send_buf));
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
uint8_t old_cmd = 0;
uint8_t image_reverse[10] = {0xAA, 0x06, 0x01, 0xDD, 0x01, 0x03, 0,0x92,0xEB, 0xAA};
uint8_t Ir_Ctrl_Lgcs_Data_send(uint8_t send_type, uint8_t *data)
{
	SYS_IR_STA_T* ir_info = CONFIG_Get_Ir_Info();
//	if (CONFIG_Get_Master_Ctrl_Cmd(MASTER_IR_PARA) == 0)
//	{
//		return false;
//	}
    switch(send_type)
    {
        case IR_CALIBRATION:/*图像校正*/
            Ir_Calibration_ctrl_Send(data);
            break;

        case IR_SYS_STA_REQ://自检
            break;
		case IR_SYS_RES:
			COM_API_Send_Data(COM_IR_LGCS6122,image_reverse,sizeof(image_reverse));
		break;
//        case IR_ZOOM_CTRL://电子变倍
//			if(data[0] == 0x01) /*+*/
//			{
//				uint8_t zoom = ir_info->zoom_position+ir_info->zoom_step;
//				Ir_Ctrl_Lgcs_Ele_Zoom_send(zoom);
//			}
//			else if(data[0] == 0x02) /*-*/
//			{
//				uint8_t zoom = ir_info->zoom_position-ir_info->zoom_step;
//				Ir_Ctrl_Lgcs_Ele_Zoom_send(zoom);
//			}
//			else if(data[0] == 0x04) /*步进设置*/
//			{
//				ir_info->zoom_step = data[1];
//			}
//			else if(data[0] == 0x10) /*变倍设置*/
//			{
//				Ir_Ctrl_Lgcs_Ele_Zoom_send(data[1]);
//			}
//            
//            break;

        case IR_LIGHT_CTRL:
        case IR_CONTRAST_CTRL:
        case IR_DDE_RANGE:
        case IR_BLACK_WHITE:
		case IR_IMAGE_ENHANCE:
            Ir_Ctrl_Lgcs_Cmd_Data_send(send_type, data);
			CONFIG_Set_Master_Ctrl_Cmd(MASTER_IR_PARA,0); //解析控制指令
            break;

        case IR_CFG_CROSSHAIR_CTRL:
            Ir_Crosshair_Send(data);
            break;
		
		case IR_CH_LARGE:
		case IR_CH_MI:
		case IR_CH_SMA:
		case IR_ZOOM_CTRL://
		case IR_FOCUS_CTRL:/*调焦*/
		case IR_FOCUS_AUTO:			/*自动聚焦*/
		case IR_FOCUS_ONE_SET:
			Ir_Ctrl_hy5050e_Cmd_Data_send(send_type,data[0]);
//			CONFIG_Set_Master_Ctrl_Cmd(MASTER_IR_PARA,0); //解析控制指令
			break;

			//Ir_Ctrl_hy5050e_Cmd_Data_send(send_type,data[0]);
		//break;

		
        default:
            return RECV_DATA_OTHERR_ERR;
    }
   
   // CONFIG_Set_Master_Ctrl_Para(MASTER_IR_PARA, 0);
    ir_lgcs_send_sta = true;
//	COM_API_Send_Data(COM_IR_LGCS6122, (uint8_t *)&lgcs_send_buf.Header1,
//                      sizeof(lgcs_send_buf));
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
uint8_t ir_lgcs_process_data_in(uint8_t *data, uint32_t length)
{
//    uint8_t sum_check = 0;
#if 0
    SYS_IR_STA_T* main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    //由于该红外机芯还有别的回报报文上报，将触发解析，因此通过发送报文来约束接收
    //如果别的报文回报，先于设置报文对应的回报报文，则该处理方式还是不够完善;如果需要，请放开这段屏蔽报文

    //校验尾
    if ((data[length - 1] != 0xAA) && (data[length - 2] != 0xEB))
    {
        return false;
    }

    switch (data[2])
    {
        case IR_SELFCHECK_RSP:
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\nshutter set success");
#endif
            //自检结果解析
            break;

        case IR_LGCS_CONTRAST_RES://01 AE
			//返回：55 04 21 33 01 AE EB AA
            main_ir_info->contrast_level = data[5];
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\ncontrast read success");
#endif
            break;
		case IR_LIGHT_RES:
			main_ir_info->light_level = data[5];
			break;

        case IR_LIGHT_SET:
            memcpy(&main_ir_info->contrast_level, &data[4],
                   sizeof(main_ir_info->contrast_level));
#if CLI_INCLUDE
            DEBUG_IR_PRINT("\r\n set success");
#endif
            break;

        default:
            break;
    }

    //正确接收解算之后，要对状态位进行置位。
    ir_lgcs_send_sta = false;
//    CONFIG_Set_Ir_Info(main_ir_info);
    return 0;
#endif
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
	recv_len = COM_REC_Data_Direct(COM_IR_LGCS6122, ir_lgcs_rcv_buf.recv_buf);
	
    if (recv_len == 0)
    {
        return RECV_DATA_NULL;
    }
    else
    {
    }

    ir_lgcs_process_data_in(ir_lgcs_rcv_buf.recv_buf, recv_len);
    return RECV_DATA_SUC;
}

#endif
