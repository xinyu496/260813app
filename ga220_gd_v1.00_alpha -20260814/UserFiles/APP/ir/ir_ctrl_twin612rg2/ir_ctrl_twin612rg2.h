#ifndef __IR_CTRL_TWIN612RG2_H
#define __IR_CTRL_TWIN612RG2_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "Common/opt_module.h"
#include "Common/config.h"

#if (IR_CTRL_INCLUDE&IR_TWIN612RG2)

#pragma pack(1)
typedef struct
{
    uint8_t Header1;
		uint8_t Header2;
    uint8_t data_len;
//    uint8_t dev_id;
    uint8_t ctrl_cmd;
		uint8_t trailer;
		uint8_t sum_check;
} IR_TWIN612RG2_HEADER_T;
#pragma pack()
#define IR_TWIN612RG2_HEADER_LEN  sizeof(IR_TWIN612RG2_HEADER_T)

#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t data_len;
    uint8_t dev_id;
    uint8_t ctrl_cmd;
    struct
    {
        uint8_t crosshair:1;
        uint8_t w_b_sta:1;
        uint8_t auto_cal_sta:1;
        uint8_t image_enhance_sta:1;
    } ir_sta;
    uint8_t contrast;
    uint8_t light_degree;
    uint16_t crosshair_x;
    uint16_t crosshair_y;
    uint8_t res[9];
    uint8_t sum_check;
    uint8_t tail;
} IR_TWIN612RG2_RCV_DATA_T;
#pragma pack()
#define IR_TWIN612RG2_RCV_DATA_LEN  sizeof(IR_TWIN612RG2_RCV_DATA_T)

typedef enum
{
    IR_TWIN612RG2_STA_REQ = 0x0,				/*状态查询*/
    IR_TWIN612RG2_CALIBRATION_BACK = 0x2,    /*手动背景校正*/
    IR_TWIN612RG2_CALIBRATION_SHUTTER = 0x3, /*手动快门校正*/
    IR_TWIN612RG2_CALIBRATION_AUTO = 0x07,	/*自动校正设置*/
    IR_TWIN612RG2_SHUTTER_OFF = 0x41,		/*快门关闭*/
    IR_TWIN612RG2_SHUTTER_ON = 0x40,			/*快门打开*/
    IR_TWIN612RG2_IMG_ENAHNCE = 0xe,			/*图像增强*/
    IR_TWIN612RG2_DENOISE_T = 0xd,			/*时域降噪*/
    IR_TWIN612RG2_DENOISE_S = 0xf,			/*空域降噪*/
    IR_TWIN612RG2_CONTRAST = 0x9,			/*对比度设置*/
    IR_TWIN612RG2_LIGHT = 0xa,				/*亮度设置*/
    IR_TWIN612RG2_B_W_CH = 0x5,				/*极性设置*/
    IR_TWIN612RG2_CROSS_HAIR = 0x4,			/*十字光标显示*/
    IR_TWIN612RG2_CROSS_X = 0xb,				/*十字光标横坐标*/
    IR_TWIN612RG2_CROSS_Y = 0xc,				/*十字光标纵坐标*/
    IR_TWIN612RG2_MIRROR = 0x16,				/*镜像*/
    IR_TWIN612RG2_ZOOM_ELE = 0x8,				/*电子变倍设置*/
//	IR_TWIN612RG2_ZOOM_STEP = 0x8,			/*手动背景校正*/
    IR_TWIN612RG2_ZOOM = 0x11,		/*变倍*/
    IR_TWIN612RG2_FOCUS = 0x1,				/*调焦*/
    IR_TWIN612RG2_FOCUS_AUTO = 0x34,			/*自动调焦*/
    IR_TWIN612RG2_FOCUS_STOP = 0x10,			/*调焦停止*/
    IR_TWIN612RG2_SAVE_CFG  = 0x81,			/*保存用户设置*/
    IR_TWIN612RG2_RESET = 0x80,				/*复位*/
    //IR_S640A_SYS_RESET = 0x80,
    IR_TWIN612RG2_RECV_END,
} IR_TWIN612RG2_CORE_SEND_TYPE;




/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_TWIN612RG2_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_TWIN612RG2_SendHandle(uint8_t send_type,uint8_t *data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
RECV_DATA_ERR_STA IR_API_A640s_Period_Handle(void);
#endif
#endif
