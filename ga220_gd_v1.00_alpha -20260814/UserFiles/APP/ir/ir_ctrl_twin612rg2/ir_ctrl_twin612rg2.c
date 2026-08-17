#include "stm32f4xx_hal.h"
#include "string.h"
#include "Common/utl_math.h"
#include "Common/config.h"
#include "ir_ctrl_twin612rg2.h"
#include "Common/opt_cmd.h"

#if (IR_CTRL_INCLUDE&IR_TWIN612RG2)
//默认波特率115200，定焦镜头；
#define IR_TWIN612RG2_HEADER1 0x55
#define IR_TWIN612RG2_HEADER2 0xAA
#define IR_TWIN612RG2_TRAILER 0xF0

static uint8_t ir_twin612rg2_send_buf[12];
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:报文发送控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_calibration_shutter[12] = 	{0x55,0xAA,0x07,0x02,0x01,0x08,0x00,0x00,0x00,0x01,0x0D,0xF0}; //快门补偿
uint8_t IR_calibration_back[12] = 		{0x55,0xAA,0x07,0x02,0x01,0x07,0x00,0x00,0x00,0x01,0x02,0xF0}; //场景补偿
uint8_t Enable_image_freeze[12] = 		{0x55,0xAA,0x07,0x01,0x00,0x02,0x00,0x00,0x00,0x01,0x05,0xF0}; //图像定格开
uint8_t Disable_image_freeze[12] = 		{0x55,0xAA,0x07,0x01,0x00,0x02,0x00,0x00,0x00,0x00,0x04,0xF0}; //图像定格关
uint8_t White_hot[12] = 				{0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x00,0x01,0xF0}; //白热
uint8_t Black_hot[12] = 				{0x55,0xAA,0x07,0x02,0x00,0x04,0x00,0x00,0x00,0x09,0x08,0xF0}; //黑热
uint8_t Warm_color[12] = 				{0x55,0xAA,0x07,0x02,0x02,0x19,0x00,0x00,0x00,0x00,0x1E,0xF0}; //暖色
uint8_t Cold_color[12] = 				{0x55,0xAA,0x07,0x02,0x02,0x19,0x00,0x00,0x00,0x01,0x1F,0xF0}; //冷色
uint8_t Denoise[12] = 					{0x55,0xAA,0x07,0x02,0x02,0x1C,0x00,0x00,0x00,0x01,0x1A,0xF0}; //去噪 1-3 
uint8_t DDE[12] = 						{0x55,0xAA,0x07,0x02,0x02,0x1D,0x00,0x00,0x00,0x01,0x1B,0xF0}; //增强 1-4
uint8_t Brightness[12] = 				{0x55,0xAA,0x07,0x02,0x02,0x1E,0x00,0x00,0x00,0x01,0x18,0xF0}; //亮度 1-5
uint8_t Contrast[12] = 					{0x55,0xAA,0x07,0x02,0x02,0x1F,0x00,0x00,0x00,0x01,0x19,0xF0}; //对比度 1-5
uint8_t FPS_50[12] = 					{0x55,0xAA,0x07,0x02,0x01,0x05,0x00,0x00,0x00,0x03,0x02,0xF0}; //50Hz
uint8_t FPS_30[12] = 					{0x55,0xAA,0x07,0x02,0x01,0x05,0x00,0x00,0x00,0x00,0x01,0xF0}; //30Hz
uint8_t FPS_25[12] = 					{0x55,0xAA,0x07,0x02,0x01,0x05,0x00,0x00,0x00,0x01,0x00,0xF0}; //25Hz
uint8_t Save_setting[12] = 				{0x55,0xAA,0x07,0x01,0x00,0x04,0x00,0x00,0x00,0x01,0x03,0xF0}; //保存设置
uint8_t Factory_reset[12] = 			{0x55,0xAA,0x07,0x01,0x00,0x05,0x00,0x00,0x00,0x01,0x02,0xF0}; //恢复出厂设置
uint8_t Denoise_SET = 0x01;
uint8_t DDE_SET = 0x01;
uint8_t Brightness_SET = 0x01;
uint8_t Contrast_SET = 0x01;

uint8_t IR_TWIN612RG2_SendHandle(uint8_t send_type,uint8_t *data)
{
    IR_TWIN612RG2_HEADER_T twin612rg2_send_head = {0};
    uint16_t temp_buf = 0;
    bool len_sta = false;
    uint8_t total_len = 0;
    uint8_t temp_change_l = 0;
    uint8_t temp_change_h = 0;
	int DATElen;

	SYS_IR_STA_T* main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    switch (send_type)
    {
		case IR_CALIBRATION:/*图像矫正*/
        if (data[0] == 1)//手动背景矫正 //参数1：bit1-背景矫正；bit2-虚焦校正，bit3-档板校正，bit4-快门校正，bit5-自动矫正设置；
        {
			DATElen = sizeof(IR_calibration_back);
			memcpy(ir_twin612rg2_send_buf,IR_calibration_back,DATElen);
        }
        else if (data[0] == 4)//手动快门矫正
        {
			DATElen = sizeof(IR_calibration_shutter);
			memcpy(ir_twin612rg2_send_buf,IR_calibration_shutter,DATElen);
        }
        else
        {
            return PARA_ERR;
        }
		break;

		case IR_IMAGE_ENHANCE:/*图像增强*/

        if(data[0] == 2)				/*加*/
        {
			DATElen = sizeof(DDE);
			if(DDE_SET>=0x04)
			{
				DDE_SET = 0x04;
			}
			else
			{
				DDE_SET++;
			}
			DDE[9] = DDE_SET;
			DDE[DATElen-2] = 0x00;
			for(int i=2;i<DATElen-2;i++)
			{
				DDE[DATElen-2] ^= DDE[i];
			}
			memcpy(ir_twin612rg2_send_buf,DDE,DATElen);
        }
        else if(data[0] == 3)		/*减*/
        {
			DATElen = sizeof(DDE);
			if(DDE_SET<=0x01)
				DDE_SET = 0x01;
			else
				DDE_SET--;
				DDE[9] = DDE_SET;
				DDE[DATElen-2] = 0x00;
			for(int i=2;i<DATElen-2;i++)
			{
				DDE[DATElen-2] ^= DDE[i];
			}
			memcpy(ir_twin612rg2_send_buf,DDE,DATElen);
        }
		main_ir_info->DDE_level = DDE_SET;
				break;
#if 1
    case IR_RENOISE:/*去噪*/
        if (data[0] == 1) /*去噪等级1*/
		{
			Denoise[9] = 0x01;
			DATElen = sizeof(Denoise);
			memcpy(ir_twin612rg2_send_buf,Denoise,DATElen);
        }
        else if (data[0] == 2)
        {
			for(int i=2;i<DATElen-2;i++)
			{
				Denoise[9] = 0x02;
				Denoise[DATElen-2] ^= Denoise[i];
			}
			memcpy(ir_twin612rg2_send_buf,Denoise,DATElen);
        }
				else if (data[0] == 3)
        {
			for(int i=2;i<DATElen-2;i++)
			{
				Denoise[9] = 0x03;
				Denoise[DATElen-2] ^= Denoise[i];
			}
			memcpy(ir_twin612rg2_send_buf,Denoise,DATElen);
        }
				 else
        {
            return PARA_ERR;
        }

        break;
#endif

    case IR_LIGHT_CTRL:/*亮度设置*/
		if(data[0] == 1)				/*加*/
        {
				DATElen = sizeof(Brightness);
				if(Brightness_SET>=0x05)
					Brightness_SET = 0x05;
				else
					Brightness_SET++;
				Brightness[9] = Brightness_SET;
				Brightness[DATElen-2] = 0x00;
				for(int i=2;i<DATElen-2;i++)
				{
					Brightness[DATElen-2] ^= Brightness[i];
				}
				memcpy(ir_twin612rg2_send_buf,Brightness,DATElen);
				
        }
				else if(data[0] == 2)				/*减*/
        {
				DATElen = sizeof(Brightness);
				if(Brightness_SET<=0x01)
					Brightness_SET = 0x01;
				else
					Brightness_SET--;
				Brightness[9] = Brightness_SET;
				Brightness[DATElen-2] = 0x00;
				for(int i=2;i<DATElen-2;i++)
				{
					Brightness[DATElen-2] ^= Brightness[i];
				} 
				memcpy(ir_twin612rg2_send_buf,Brightness,DATElen);
        }
		main_ir_info->light_level = Brightness_SET;
        break;

    case IR_CONTRAST_CTRL:/*对比度设置*/
		if(data[0] == 1)				/*加*/
        {
						DATElen = sizeof(Contrast);
					if(Contrast_SET>=0x05)
							Contrast_SET = 0x05;
					else
							Contrast_SET++;
					Contrast[9] = Contrast_SET;
					Contrast[DATElen-2] = 0x00;
					for(int i=2;i<DATElen-2;i++)
					{
						Contrast[DATElen-2] ^= Contrast[i];
					}
					memcpy(ir_twin612rg2_send_buf,Contrast,DATElen);
        }
				else if(data[0] == 2)				/*减*/
        {
					DATElen = sizeof(Contrast);
					if(Contrast_SET<=0x01)
							Contrast_SET = 0x01;
					else
							Contrast_SET--;
					Contrast[9] = Contrast_SET;
					Contrast[DATElen-2] = 0x00;
					for(int i=2;i<DATElen-2;i++)
					{
						Contrast[DATElen-2] ^= Contrast[i];
					}
					memcpy(ir_twin612rg2_send_buf,Contrast,DATElen);
        }
		main_ir_info->contrast_level = Contrast_SET;
        break;

    case IR_BLACK_WHITE:/*黑/白热*/
        if(data[0] == 0)
        {
            temp_buf = 0;//白
			memcpy(ir_twin612rg2_send_buf,White_hot,sizeof(White_hot));
        }
        else
        {
            temp_buf = 0xf;//黑
			memcpy(ir_twin612rg2_send_buf,Black_hot,sizeof(Black_hot));
        }
        break;

    case IR_IMAGE_TONE:/*图像色调*/
        if(data[0] == 0)//暖色
        {
            temp_buf = 0;
			memcpy(ir_twin612rg2_send_buf,Warm_color,sizeof(Warm_color));
        }
        else
        {
            temp_buf = 0xf;
			memcpy(ir_twin612rg2_send_buf,Cold_color,sizeof(Cold_color));
        }
        break;
#if 0
    case IR_FOCUS_AUTO:
        twin612rg2_send_head.ctrl_cmd = IR_S640A_FOCUS_AUTO;
        twin612rg2_send_head.data_len = 1;
        break;

    case IR_FOCUS_STOP:
        twin612rg2_send_head.ctrl_cmd = IR_S640A_FOCUS_STOP;
        twin612rg2_send_head.data_len = 1;
        break;

    case IR_SHUTTER_ON:
        twin612rg2_send_head.ctrl_cmd = IR_S640A_SHUTTER_ON;
        twin612rg2_send_head.data_len = 1;
        break;
#endif
		default:
			return PARA_ERR;
		break;
    }
	
    COM_API_Send_Data(component_map[4].com_type_in,ir_twin612rg2_send_buf,sizeof(ir_twin612rg2_send_buf));
    return CMD_SUCESS;
}


#endif
