#include "stm32f4xx_hal.h"
#include "string.h"
#include "Common/utl_math.h"
//#include "Common/com_api.h"
#include "../cli/cli_cmd_line.h"
#include "Common/config.h"
#include "../ir/ir_ctrl_mino17.h"
#include "Common/opt_cmd.h"

/*==============================================================
串口波特率 115200；
*==============================================================*/
#if (IR_CTRL_INCLUDE&IR_MINO17)
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接受数据头判断，状态切换
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

uint8_t ir_save[8] = {0xAA,0x04,0x01,0x7F,0x02,0x30,0xEB,0xAA};			//保存设置

uint8_t ir_bright_req[10] = {0x6E,0x00,0x00,0x55,0x00,0x00,0x6A,0x85,0x00,0x00};	//亮度查询

uint8_t ir_brightness[12] = {0x6E,0x00,0x00,0x55,0x00,0x02,0x4A,0xC7,0x00,0x80,0x91,0x88};	//亮度设置 0x80 0-255

uint8_t ir_contrast_req[11] = {0x6E,0x00,0x00,0x00,0x00,0x01,0xCF,0x9A,0x2D,0xF5,0xCF};	//对比度查询

uint8_t ir_contrast[13] = {0x6E,0x00,0x00,0x00,0x00,0x03,0xEF,0xD8,0x2D,0x00,0x32,0xD2,0x86};	//对比度设置 0x32 0-100

uint8_t ir_sharpen_req[10] = {0x6E,0x00,0x00,0xE3,0x00,0x00,0x26,0xDA,0x00,0x00};	//锐度查询

uint8_t ir_sharpen[12] = {0x6E,0x00,0x00,0xE3,0x00,0x02,0x06,0x98,0x00,0x28,0xA5,0x6A};	//锐度设置 0x28 0-100

uint8_t ir_dde_req[11] = {0x6E,0x00,0x00,0x00,0x00,0x01,0xCF,0x9A,0x2C,0xE5,0xEE};	//增强查询

uint8_t ir_dde[13] = {0x6E,0x00,0x00,0x00,0x00,0x03,0xEF,0xD8,0x2C,0x00,0x40,0xBB,0x63};	//增强设置 0x40 0-128

uint8_t ir_pseudo_color_req[10] = {0x6E,0x00,0x00,0x10,0x00,0x00,0x9C,0xD8,0x00,0x00};	//伪彩查询

uint8_t ir_black_hot[12] = {0x6E,0x00,0x00,0x10,0x00,0x02,0xBC,0x9A,0x00,0x01,0x10,0x21};	//黑热 0x0001
uint8_t ir_white_hot[12] = {0x6E,0x00,0x00,0x10,0x00,0x02,0xBC,0x9A,0x00,0x00,0x00,0x00};	//白热 0x0000

uint8_t ir_calibration_req[10] = {0x6E,0x00,0x00,0x0B,0x00,0x00,0x2F,0x4A,0x00,0x00};	//校正模式查询 0x0000 手动模式 0x0001 自动模式

uint8_t ir_calibration_auto[12] = {0x6E,0x00,0x00,0x0B,0x00,0x02,0x0F,0x08,0x00,0x01,0x10,0x21};	//自动校正
uint8_t ir_calibration_manual[12] = {0x6E,0x00,0x00,0x0B,0x00,0x02,0x0F,0x08,0x00,0x00,0x00,0x00};	//手动校正

uint8_t ir_EZoom_req[11] = {0x6E,0x00,0x00,0x00,0x00,0x01,0xCF,0x9A,0x56,0x3A,0x33};	//电子变倍查询
uint8_t ir_EZoom_1[12] = {0x6E,0x00,0x00,0x00,0x00,0x02,0xFF,0xF9,0x56,0x01,0xB4,0x38};	//电子变倍-1倍
uint8_t ir_EZoom_2[12] = {0x6E,0x00,0x00,0x00,0x00,0x02,0xFF,0xF9,0x56,0x02,0x84,0x5B};	//电子变倍-2倍

uint8_t ir_shutter[10] = {0x6E,0x00,0x00,0x0C,0x00,0x00,0xAA,0xDA,0x00,0x00};	//进行一次快门校正

uint8_t ir_res[10] = {0x6E,0x00,0x00,0x03,0x00,0x00,0x86,0xEB,0x00,0x00};	//恢复出厂设置

uint8_t Brightness_Set = 0x80;	//待确定
uint8_t Contrast_Set = 0x32;	//待确定
uint8_t DDE_Set_Init = 0x40;	//待确定

#define IR_MINO17_RECV_HEADER 0x6E

static uint8_t ir_mino17_send_buf[13] = {0};
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
static uint8_t mino17_cmd = 0;
uint8_t IR_MINO17_SendHandle(uint8_t send_type,uint8_t *data)
{
    uint16_t temp_buf = 0;
    bool len_sta = false;
    uint8_t total_len = 0;
	SYS_IR_STA_T *main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();
	mino17_cmd = send_type;

    switch (send_type)
    {
    case IR_CALIBRATION:/*图像矫正*/
		if (data[0] == 4)//手动快门矫正
        {
			memcpy(ir_mino17_send_buf,ir_shutter,sizeof(ir_shutter));
        }
        else if (data[0] == 5)//自动矫正设置
        {
            if(data[1] == 0)	//手动校正
            {
				memcpy(ir_mino17_send_buf,ir_calibration_manual,sizeof(ir_calibration_manual));
				main_ir_info->auto_calibration = 0;
            }
            else				//自动校正
            {
				memcpy(ir_mino17_send_buf,ir_calibration_auto,sizeof(ir_calibration_auto));
				main_ir_info->auto_calibration = 1;
            }
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_IMAGE_ENHANCE://图像增强
		uint8_t DDE_Set = DDE_Set_Init;
		uint16_t temp = 0 ;
        if(data[0] == 0) //图像增强关，回到默认值
        {
			ir_dde[10] = DDE_Set_Init;
			main_ir_info->image_enhance_sta = 0;
        }
        else if(data[0] == 2) //图像增强+
        {
			if(DDE_Set >= 0x80)
			{
				DDE_Set = 0x80;
			}
			else 
			{
				DDE_Set += 0x0A;
			}
			ir_dde[10] = DDE_Set;
			temp = UTL_CRC16_CCITT(&ir_dde[0],sizeof(ir_dde)-3);
			ir_dde[11] = (uint8_t) temp&0xFF;
			ir_dde[12] = (uint8_t) (temp>>8)&0xFF;
			memcpy(ir_mino17_send_buf,ir_dde,sizeof(ir_dde));
        }
		else if(data[0] == 3) //图像增强-
		{
			if(DDE_Set <= 0x40)
			{
				DDE_Set = 0x40;
			}
			else 
			{
				DDE_Set -= 0x0A;
			}
			ir_dde[10] = DDE_Set;
			temp = UTL_CRC16_CCITT(&ir_dde[0],sizeof(ir_dde)-3);
			ir_dde[11] = (uint8_t) temp&0xFF;
			ir_dde[12] = (uint8_t) (temp>>8)&0xFF;
			memcpy(ir_mino17_send_buf,ir_dde,sizeof(ir_dde));
		}
		else if(data[0] == 0x0F) //图像增强查询
		{
			memcpy(ir_mino17_send_buf,ir_dde_req,sizeof(ir_dde_req));
		}
		main_ir_info->image_enhance_sta = 1;//放接收
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

    case IR_CONTRAST_CTRL://1-加；2-减
		uint16_t temp1 = 0;
		if (data[0] == 0x0f)
        {
			memcpy(ir_mino17_send_buf,ir_contrast_req,sizeof(ir_contrast_req));
        }
        else if (data[0] == 1)
		{
			if (Contrast_Set >= 0x64)
			{
				Contrast_Set = 0x64;
			}
			else 
			{
				Contrast_Set += 0x0A;
			}
			ir_contrast[10] = Contrast_Set;
			temp1 = UTL_CRC16_CCITT(&ir_contrast[0],sizeof(ir_contrast)-3);
			ir_contrast[11] = (uint8_t) temp1&0xFF;
			ir_contrast[12] = (uint8_t) (temp1>>8)&0xFF;
			memcpy(ir_mino17_send_buf,ir_contrast,sizeof(ir_contrast));
		}
		else if (data[0] == 2)
		{
			if (Contrast_Set <= 0x01)
			{
				Contrast_Set = 0x01;
			}
			else
			{
				Contrast_Set -= 0x0A;
			}
			ir_contrast[10] = Contrast_Set;
			temp1 = UTL_CRC16_CCITT(&ir_contrast[0],sizeof(ir_contrast)-3);
			ir_contrast[11] = (uint8_t) temp1&0xFF;
			ir_contrast[12] = (uint8_t) (temp1>>8)&0xFF;
			memcpy(ir_mino17_send_buf,ir_contrast,sizeof(ir_contrast));
		}
        break;

    case IR_LIGHT_CTRL:
		uint16_t temp2 = 0;
		if (data[0] == 0x0f)
        {
			memcpy(ir_mino17_send_buf,ir_bright_req,sizeof(ir_bright_req));
        }
        else if (data[0] == 1)
		{
			if (Brightness_Set >= 0xff)
			{
				Brightness_Set = 0xff;
			}
			else 
			{
				Brightness_Set += 0x0A;
			}
			ir_brightness[9] = Brightness_Set;
			temp2 = UTL_CRC16_CCITT(&ir_brightness[0],sizeof(ir_brightness)-3);
			ir_brightness[10] = (uint8_t) temp2&0xFF;
			ir_brightness[11] = (uint8_t) (temp2>>8)&0xFF;
			memcpy(ir_mino17_send_buf,ir_brightness,sizeof(ir_brightness));
		}
		else if (data[0] == 2)
		{
			if (Brightness_Set <= 0x01)
			{
				Brightness_Set = 0x01;
			}
			else 
			{
				Brightness_Set -= 0x0A;
			}
		}
		ir_brightness[9] = Brightness_Set;
		temp2 = UTL_CRC16_CCITT(&ir_brightness[0],sizeof(ir_brightness)-3);
		ir_brightness[10] = (uint8_t) temp2&0xFF;
		ir_brightness[11] = (uint8_t) (temp2>>8)&0xFF;
		memcpy(ir_mino17_send_buf,ir_brightness,sizeof(ir_brightness));
        break;

    case IR_BLACK_WHITE:
        if(data[0] == 0)
        {
			memcpy(ir_mino17_send_buf,ir_white_hot,sizeof(ir_white_hot));
			main_ir_info->ir_status.black_white = 0;
        }
        else
        {
			memcpy(ir_mino17_send_buf,ir_black_hot,sizeof(ir_black_hot));
			main_ir_info->ir_status.black_white = 1;
        }
        break;

    case IR_ZOOM_DIT:
        if (data[0] == 1)
        {
			memcpy(ir_mino17_send_buf,ir_EZoom_1,sizeof(ir_EZoom_1));
        }
        else if (data[0] == 2)
        {
			memcpy(ir_mino17_send_buf,ir_EZoom_2,sizeof(ir_EZoom_2));
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_SYS_RES://恢复默认
		memcpy(ir_mino17_send_buf,ir_res,sizeof(ir_res));
        break;

    default:
        return PARA_ERR;
    }
    total_len = sizeof(ir_mino17_send_buf);
    COM_API_Send_Data(COM_IR_MINO17,ir_mino17_send_buf,total_len);
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
uint8_t IR_MINO17_PROCESS_Get(uint8_t *data,uint32_t length)
{

    IR_MINO17_RCV_DATA_T rcv_ir_info;
    uint8_t sum_check1 = 0;
	uint8_t sum_check2 = 0;
	uint16_t temp_sum = 0;
	
    SYS_IR_STA_T *main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();
    //校验计算UTL_Htons
    temp_sum = UTL_CRC16_CCITT(&data[0],(length-2));
	sum_check2 = (uint8_t) (temp_sum>>8)&0xFF;
	sum_check1 = (uint8_t) temp_sum&0xFF;
	
    if ((data[length-2] != sum_check1) && (data[length-1] != sum_check2))
    {
//        DEBUG_IR_PRINT("\r\n sum_h:%x sum_l:%x data_h:%x data_l:%x",sum_check1,sum_check2,data[length-2],data[length-1]);
        return RECV_DATA_XOR_ERR;
    }
	if (data[0] == IR_MINO17_RECV_HEADER)
	{
		switch (mino17_cmd)
	 {
		case IR_IMAGE_ENHANCE:
			if (data[length-5] == 0x2C)
				main_ir_info->DDE_level = data[length-3];
			break;
			
		case IR_CONTRAST_CTRL:
			if (data[length-5] == 0x2D)
				main_ir_info->contrast_level = data[length-3];
			break;
		case IR_LIGHT_CTRL:
			if (data[length-5] == 0xC7)
				main_ir_info->light_level = data[length-3];
			break;
		default:
			break;
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
