#ifndef __VISIBLE_F15_H
#define __VISIBLE_F15_H
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "Common/opt_module.h"

#if (VISIBLE_INCLUDE&VL_F15_300)
typedef enum
{
    VL_F15_SET_ZOOM = 0x1,
    VL_F15_APERTURE = 0x2,
    VL_F15_SET_FOCUS = 0x3,
    VL_F15_LIGHT_CTRL = 0x4,
    VL_F15_ELE_ZOOM = 0x5,
    VL_F15_EXPOSURE = 0x6,
    VL_F15_ENHANCE = 0x7,
    VL_F15_CONTRAST = 0x8,
    VL_F15_WHITE_BALANCE = 0x9,
    VL_F15_VIEW_CHANGE = 0x10,
    VL_F15_SECT_CONTROL = 0x11,
    VL_F15_FILTER_LIGHT = 0x12,
    VL_F15_ENHANCE_CTRL = 0x18,
    VL_F15_HLC = 0x20,
    VL_F15_BLC = 0x21,
    VL_F15_WHITE_BLACK = 0x25,
    VL_F15_CROSSHAIR = 0x26,
    VL_F15_PICTURE_CHANGE = 0x27,
    VL_F15_PICTURE_FPS = 0x28,
    _TURB_SUPPRESS = 0x29,
    _COLOR_ENHANCE = 0x30,
    _LOW_POWER = 0x31,
    _SET_ACUITY = 0x32,//锐度

    VISIBLE_SET_CMD_END,
} VISIBLE_DATA_SEND_TYPE;

#if 1
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t connect_method;
    uint8_t ctrl_cmd;
    uint8_t ctrl_cmd1;
    uint8_t ctrl_cmd2;
    uint8_t ctrl_para[4];
    uint8_t add_check;
    uint8_t tail_1;
    uint8_t tail_2;
} VISIBLE_F15_SEND_T;
#pragma pack()
#define VISIBLE_SEND_DATA_LEN  sizeof(VISIBLE_F15_SEND_T)
#endif
//接收报文结构体
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint8_t connect_mode;
    uint8_t video_sta;
    uint8_t cmd_sta;
    uint8_t focus_sta;
    struct
    {
        uint8_t view_ch:4;
        uint8_t res0:4;
    } view_sta;
    uint16_t zoom_position;
    uint8_t enhance_degree;
    struct
    {
        uint8_t view_hlc:1;
        uint8_t res_1:1;
        uint8_t view_blc:1;
        uint8_t res_2:5;
    } light_sta;
    uint8_t zoom_ele;
    uint8_t video_fps;
    uint8_t light_degree;
    uint8_t gain_degree;
    uint8_t contrast_degree;
    uint8_t video_alarm;
    uint8_t visible_ver_1;
    uint8_t visible_ver_2;
    uint8_t res1[10];
    uint8_t add_check;
    uint8_t tail1;
    uint8_t tail2;
} VISIBLE_F15_RECV_T;
#pragma pack()
#define VISIBLE_RECV_DATA_LEN  sizeof(VISIBLE_F15_RECV_T)

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_F15_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:控制指令；
			ctrl_type:指令类型；
			ctrl_data:数据内容；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t Visible_F15_Ctrl_SendHandle(uint8_t send_type,uint8_t *ctrl_data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t VISIABLE_F15_Period_Handle(void);
#endif

#endif
