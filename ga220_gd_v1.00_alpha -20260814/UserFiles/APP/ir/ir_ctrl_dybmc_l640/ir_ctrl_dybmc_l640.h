#ifndef __IR_CTRL_DYBMC_L640_H
#define __IR_CTRL_DYBMC_L640_H
#include "Common/base_inc.h"

#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
//协议格式
#if 1
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t ctrl_cmd;
    uint8_t ctrl_data1;
    uint8_t ctrl_data2;
    uint8_t ctrl_data3;
    uint8_t ctrl_data4;
    uint8_t tail_1;
    uint8_t tail_2;

} IR_DATA_CTRL_T;
#pragma pack()
#define IR_CTRL_DATA_LEN  sizeof(IR_DATA_CTRL_T)
#endif
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t ir_status;
    uint8_t light_level;
    uint8_t contrast_level;
    uint8_t elezoom;
    uint8_t enhancement;
    uint8_t res2;
    uint8_t itr_range;
    uint16_t integration_time;
    uint16_t crosshair_x;
    uint16_t crosshair_y;
    uint32_t work_time;
    uint16_t startup_cnt;
    uint16_t detect_temp;
    uint16_t ambient_temp;
    uint16_t res3;
    uint16_t grayscale;
    uint16_t zoom_position;
    uint16_t focus_position;
    uint8_t lens_temp;
    uint8_t res4;
    uint8_t tail;
} IR_DATA_RECV_T;
#pragma pack()
#define IR_RECV_DATA_LEN  sizeof(IR_DATA_RECV_T)


typedef enum
{
    IR_SET_CROSSHAIR_CHOOSE = 0x1,	/*十字开关*/
    IR_SET_BLACK_WHITE_CHOOSE = 0x2,/*黑/白热*/
    IR_SET_ZOOM_ELECTRONIC = 0x3,	/*电子变倍*/
    IR_SET_CONTRAET_LEVEL = 0x4,	/*对比度*/
    IR_SET_LIGHT_LEVEL = 0x5,		/*亮度*/
    IR_SET_DDE_ENHANCE = 0x6,		/*DDE增强开/关*/
    IR_SET_DDE_LEVEL = 0x10,		/*DDE档位*/
    IR_SET_IMAGE_FLIP = 0x7,		/*图像翻转*/
    IR_SET_DIGITAL_VIDEO = 0x9,		/*数字视频选择*/
    IR_RESET = 0x0A,				/*恢复默认参数*/
    IR_SET_SAVE = 0x0B,				/*保存图像参数*/
    IR_SET_IMAGE_STA = 0xC,			/*图像状态*/
    IR_SET_TEMP_COLLECT = 0xD,		/*温度采集*/
    IR_INTERGRATION_TIME = 0xE,		/*积分时间设置*/
    IR_SET_FILTER = 0x13,			/*滤波*/
    IR_SET_X_POSITION = 0x14,		/*横坐标设置*/
    IR_SET_Y_POSITION = 0x15,		/*纵坐标设置*/
    IR_SET_CROSSHAIR_RIGHT = 0x16,	/*十字右移*/
    IR_SET_CROSSHAIR_LEFT = 0x17,	/*十字左移*/
    IR_SET_CROSSHAIR_DOWN = 0x18,	/*十字下移*/
    IR_SET_CROSSHAIR_UP = 0x19, 	/*十字上移*/
    IR_SET_DEAD_PIXEL_AUTO = 0x1B,	/*自动去盲元*/
    IR_SET_DEAD_PIXEL_HAND = 0x1D,	/*手动去盲元*/
    IR_SET_SYNC_MODE = 0x20,		/*同步模式*/
    IR_REQ_IMAGE_PARA = 0xFC,		/*查询图像参数*/
    IR_SET_ZOOM_ADD = 0x30,			/*变倍加*/
    IR_SET_ZOOM_MINUS = 0x31,		/*变倍减*/
    IR_SET_ZOOM_STOP = 0x32,		/*变倍停*/
    IR_SET_ZOOM_STEP_ADD = 0x33,	/*变倍步进加*/
    IR_SET_ZOOM_STEP_MINUS = 0x34,	/*变倍步进减*/
    IR_SET_FOCUS_ADD = 0x35,		/*调焦加*/
    IR_SET_FOCUS_MINUS = 0x36,		/*调焦减*/
    IR_SET_FOCUS_STOP = 0x37,		/*调焦停*/
    IR_SET_FOCUS_STEP_ADD = 0x38,	/*调焦步进加*/
    IR_SET_FOCUS_STEP_MINUS = 0x39, /*调焦步进减*/
    IR_SET_FOCUS_AUTO = 0x3A,		/*自动聚焦*/
    IR_SET_ZOOM_POSITION = 0x3D,	/*变倍位置设置*/
    IR_SET_FOCUS_POSITION = 0x40,	/*调焦位置设置*/
    IR_SET_FOCUS_ONE_STEP = 0x60, 	/*一键聚焦*/
    IR_SET_CALIBRATION = 0xAD, 	/*矫正*/

    IR_SET_CMD_END,
} IR_DATA_SEND_TYPE;

//typedef enum
//{
//	IRFOCUS_STEP = 2,
//	IRFZOOM_STEP = 2,
//	IRFOCUS_STEP_STEP = 1,
//	IRZOOM_STEP_STEP = 1,
//	IRLIGHT_RESET = 100,
//	IRCONTRST_RESET = 100,
//	IRDDE_RESET = 100,

//	IRLIGHT_STEP = 1,
//	IRCONTRST_STEP = 1,
//	IRDDE_STEP = 1,

//	IR_PARA_END,
//}IR_CTRL_PARA_TYPE;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_Dybmc_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Dybmc_SendHandle(uint8_t send_type,uint8_t *data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Dybmc_Period_Handle(void);
#endif
#endif
