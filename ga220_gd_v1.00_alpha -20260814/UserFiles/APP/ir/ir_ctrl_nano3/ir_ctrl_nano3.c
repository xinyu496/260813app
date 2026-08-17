#include "stm32f4xx_hal.h"
#include "string.h"
#include "Common/utl_math.h"
//#include "Common/com_api.h"
#include "../cli/cli_cmd_line.h"
#include "Common/config.h"
#include "../ir/ir_ctrl_nano3.h"
#include "Common/opt_cmd.h"

/*==============================================================
串口波特率 115200；
*==============================================================*/
#if (IR_CTRL_INCLUDE&IR_NANO3)
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接受数据头判断，状态切换
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

#define IR_NANO3_HEADER 0xAA
#define IR_NANO3_LOCAL_ID 0x01
#define IR_NANO3_DATA_TAIL1 0xEB
#define IR_NANO3_DATA_TAIL2 0xAA
#define IR_NANO3_RECV_HEADER 0x55

uint8_t ir_save[8] = {0xAA,0x04,0x01,0x7F,0x02,0x30,0xEB,0xAA};			//保存设置
uint8_t ir_bright_req[8] = {0xAA,0x04,0x01,0x26,0x00,0xD5,0xEB,0xAA};	//亮度查询
uint8_t ir_contrast_req[8] = {0xAA,0x04,0x01,0x21,0x00,0xD0,0xEB,0xAA};	//对比度查询
uint8_t ir_EZoom_1[16] = {0xAA,0x0C,0x01,0x40,0x02,0x00,0x00,0x00,0x00,0x7F,0x02,0xFF,0x01,0x7A,0xEB,0xAA};	//电子变倍-1倍
uint8_t ir_EZoom_2[16] = {0xAA,0x0C,0x01,0x40,0x02,0xA0,0x00,0x80,0x00,0xDF,0x01,0x7F,0x01,0x79,0xEB,0xAA};	//电子变倍-2倍
uint8_t ir_selfcheck[8] = {0xAA,0x04,0x01,0xDC,0x00,0x8B,0xEB,0xAA};	//自检

uint8_t Brightness_SET = 0x32;//待确定
uint8_t Contrast_SET = 0x32;//待确定

static uint8_t ir_nano3_send_buf[16] = {0};
//static COM_BUF_INFO_T ir_nano3_rcv_buf;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:清除接收buf
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接受数据头判断，状态切换
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口初始化，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:报文发送指令
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_NANO3_SendHandle(uint8_t send_type,uint8_t *data)
{
    IR_NANO3_HEADER_T nano3_send_head = {0};
    uint16_t temp_buf = 0;
    bool len_sta = false;
    uint8_t total_len = 0;
	SYS_IR_STA_T *main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    nano3_send_head.Header = IR_NANO3_HEADER;
    nano3_send_head.data_len = 0x05;
    nano3_send_head.dev_id = IR_NANO3_LOCAL_ID;
	nano3_send_head.tail1 = IR_NANO3_DATA_TAIL1;
	nano3_send_head.tail2 = IR_NANO3_DATA_TAIL2;
    switch (send_type)
    {
    case IR_CALIBRATION:/*图像矫正*/
        if (data[0] == 1)//手动背景矫正
        {
            nano3_send_head.ctrl_cmd[0]= 0x02;
            nano3_send_head.ctrl_cmd[1]= 0x02;
			nano3_send_head.ctrl_cmd[2]= 0x00;
        }
        else if (data[0] == 4)//手动快门矫正
        {
            nano3_send_head.ctrl_cmd[0]= 0x02;
            nano3_send_head.ctrl_cmd[1]= 0x02;
			nano3_send_head.ctrl_cmd[2]= 0x01;
        }
        else if (data[0] == 5)//自动矫正设置
        {
			nano3_send_head.ctrl_cmd[0]= 0x01;
            nano3_send_head.ctrl_cmd[1]= 0x01;
            if(data[1] == 0)	//手动校正
            {
				nano3_send_head.ctrl_cmd[2]= 0x00;
            }
            else				//自动校正
            {
                nano3_send_head.ctrl_cmd[2]= 0x01;
            }
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_GG_CALI:			//锅盖标定 0-清除，1-标定，2-保存
		if (data[0] == 0)
		{
			nano3_send_head.ctrl_cmd[0]= 0xA1;
			nano3_send_head.ctrl_cmd[1]= 0x02;
			nano3_send_head.ctrl_cmd[2]= 0x00;
		}
		else if (data[0] == 1)
		{
			nano3_send_head.ctrl_cmd[0]= 0xA1;
			nano3_send_head.ctrl_cmd[1]= 0x01;
			nano3_send_head.ctrl_cmd[2]= 0x01;
		}
        break;

    case IR_IMAGE_ENHANCE://图像增强模式 仅开关
        if(data[0] == 0)
        {
            nano3_send_head.ctrl_cmd[0]= 0x19;
			nano3_send_head.ctrl_cmd[1]= 0x01;
			nano3_send_head.ctrl_cmd[2]= 0x00;
			main_ir_info->image_enhance_sta = 0;
        }
        else
        {
			nano3_send_head.ctrl_cmd[0]= 0x19;
			nano3_send_head.ctrl_cmd[1]= 0x01;
			nano3_send_head.ctrl_cmd[2]= 0x01;
			main_ir_info->image_enhance_sta = 1;
        }
        break;
#if 0
    case IR_RENOISE:
        if (data[0] == 0)
        {
            s640a_send_head.ctrl_cmd = IR_S640A_DENOISE_T;
        }
        else if (data[0] == 1)
        {
            s640a_send_head.ctrl_cmd = IR_S640A_DENOISE_S;
        }
        if(data[1] == 0)
        {
            temp_buf = 0;
        }
        else
        {
            temp_buf = 0xf;
        }
        break;
#endif

    case IR_CONTRAST_CTRL:
		nano3_send_head.ctrl_cmd[0]= 0x21;
		nano3_send_head.ctrl_cmd[1]= 0x02;
		if (data[0] == 0x0f)
        {
			COM_API_Send_Data(COM_IR_NANO3,ir_contrast_req,sizeof(ir_contrast_req));
        }
        else if (data[0] == 1)
		{
			if (Contrast_SET >= 0xFE)
			{
				Contrast_SET = 0xFE;
			}
			else 
			{
				Contrast_SET += 0x0A;
			}
		}
		else if (data[0] == 2)
		{
			if (Contrast_SET <= 0x01)
			{
				Contrast_SET = 0x01;
			}
			else 
			{
				Contrast_SET -= 0x0A;
			}
		}
		nano3_send_head.ctrl_cmd[2] = Contrast_SET;
        break;

    case IR_LIGHT_CTRL:
		nano3_send_head.ctrl_cmd[0]= 0x26;
		nano3_send_head.ctrl_cmd[1]= 0x01;
		if (data[0] == 0x0f)
        {
			COM_API_Send_Data(COM_IR_NANO3,ir_bright_req,sizeof(ir_bright_req));
        }
        else if (data[0] == 1)
		{
			if (Brightness_SET >= 0xFE)
			{
				Brightness_SET = 0xFE;
			}
			else 
			{
				Brightness_SET += 0x0A;
			}
		}
		else if (data[0] == 2)
		{
			if (Brightness_SET <= 0x01)
			{
				Brightness_SET = 0x01;
			}
			else 
			{
				Brightness_SET -= 0x0A;
			}
		}
		nano3_send_head.ctrl_cmd[2] = Brightness_SET;
        break;

    case IR_BLACK_WHITE:
        nano3_send_head.ctrl_cmd[0]= 0x42;
		nano3_send_head.ctrl_cmd[1]= 0x02;
        if(data[0] == 0)
        {
			nano3_send_head.ctrl_cmd[2]= 0x00;//白
			main_ir_info->ir_status.black_white = 0;
        }
        else
        {
			nano3_send_head.ctrl_cmd[2]= 0x01;//黑
			main_ir_info->ir_status.black_white = 1;
        }
        break;

    case IR_ZOOM_DIT:
        if (data[0] == 1)
        {
			COM_API_Send_Data(COM_IR_NANO3,ir_EZoom_1,sizeof(ir_EZoom_1));
        }
        else if (data[0] == 2)
        {
			COM_API_Send_Data(COM_IR_NANO3,ir_EZoom_2,sizeof(ir_EZoom_2));
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_SYS_RES://恢复默认
		nano3_send_head.ctrl_cmd[0]= 0x82;
		nano3_send_head.ctrl_cmd[1]= 0x02;
		nano3_send_head.ctrl_cmd[2]= 0x00;
        break;

    default:
        return PARA_ERR;
    }
    total_len = IR_NANO3_HEADER_LEN;
	nano3_send_head.sum_check = UTL_ADD_CHECK(&nano3_send_head.Header,total_len-3);
    memcpy(ir_nano3_send_buf,&nano3_send_head,IR_NANO3_HEADER_LEN);
    COM_API_Send_Data(COM_IR_NANO3,ir_nano3_send_buf,total_len);
    return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:濮挎€侀€氫俊鍗忚
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_NANO3_PROCESS_Get(uint8_t *data,uint32_t length)
{
    uint8_t data_change[IR_NANO3_RCV_DATA_LEN] = {0};

    IR_NANO3_RCV_DATA_T rcv_ir_info;
    uint8_t sum_check = 0;
	
    SYS_IR_STA_T *main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();
    //校验计算
    sum_check = UTL_ADD_CHECK(&data[0],(length-3));
    if (data[length-3] != sum_check)
    {
        DEBUG_IR_PRINT("\r\n sum:%x data:%x",sum_check,data[length-2]);
        return RECV_DATA_XOR_ERR;
    }
	if (data[0] == IR_NANO3_RECV_HEADER)
	{
		if (data[2] == IR_NANO3_LIGHT)
		{
			main_ir_info->light_level = data[4];
		}
		else if (data[2] == IR_NANO3_CONTRAST)
		{
			main_ir_info->contrast_level = data[4];
		}
	}

    DEBUG_IR_PRINT("\r\n亮度[%d]\r\n对比度[%d]]",
                   main_ir_info->light_level,
                   main_ir_info->contrast_level);

    CONFIG_Set_Ir_Info(main_ir_info);
    return RECV_DATA_SUC;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if 0
RECV_DATA_ERR_STA IR_API_NANO3_Period_Handle(void)
{
    if (ir_nano3_rcv_buf.buf_rcved)
    {
        IR_NANO3_PROCESS_Get(ir_nano3_rcv_buf.com_data_buf,ir_nano3_rcv_buf.length);

        IR_NANO3_RECBUFF_Clear();

        return RECV_DATA_SUC;
    }
    else
    {
        return RECV_DATA_NULL;
    }
}
#endif
#endif
