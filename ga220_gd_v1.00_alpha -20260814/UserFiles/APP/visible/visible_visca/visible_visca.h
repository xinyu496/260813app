#ifndef __VISIBLE_VISCA_H
#define __VISIBLE_VISCA_H
#include "Common/base_inc.h"

#if VISIBLE_INCLUDE
#if (VISIBLE_INCLUDE&VL_CM8230)
#define VLPixel		2.0
#define VLMinFocus	5.2
#define VLMaxFocus	148.4
#define VLMinRange	2.14
#define VLMaxRange	58.1
#define VLPNum_H	1920
#define VLPNum_V	1080
#endif

#if (VISIBLE_INCLUDE&VL_F15_300)
#define VLPixel		3.45//像元尺寸um
#define VLMinFocus	15	//最小焦距值
#define VLMaxFocus	300	//最大焦距值
#define VLMinRange	1.27
#define VLMaxRange	24.9
#define VLPNum_H	1920//水平方向的像素个数
#define VLPNum_V	1080//垂直方向的像素个数
#endif

#define VL_SATURATION_INIT 0x04 //饱和度初始值 范围0~E
#define VL_CONTRAST_INIT   0x7F //对比度初始值
#define VL_IMAGE_ANHANCE_INIT 0x7F //锐化初始值
#define VL_LIGHT_INIT    0x7F         //亮度初始值
#define VL_EXPOSURE_INIT 0x7F//曝光初始值  范围0~FF

typedef enum
{
    VL_VISCA_ZOOM = 0x7,		/*变倍加减*/
    VL_VISCA_FOCUS = 0x8,		/*调焦*/
    VL_VISCA_GAIN = 0xC,		/*增益加减*/
    VL_VISCA_LIGHT_CTRL = 0xD,	/*亮度加减*/
    VL_VISCA_FOCUS_AUTO = 0x18,	/*自动聚焦*/
	VL_VISCA_LIGHT_ENHIBIT = 0x33,	/*强光抑制*/
    VL_VISCA_D_ZOOM = 0x36, 		/*d-zoom分离*/
	VL_VISCA_DEFOG = 0x37,			/*透雾*/
    VL_VISCA_FOUCUS_HANDLE = 0x38, /*手动聚焦，vs2030无需切手动*/
    VL_VISCA_IMAGE_AUTO = 0x39,	/*图像自动调节 00*/
    VL_VISCA_IMAGE_ENAHNCE = 0x3d,	/*图像增强 （注：实际使用的不是这个）*/
	VL_VISCA_APERTURE = 0x42,		/*光圈设置*/
    VL_VISCA_EXPOSURE_CTRL = 0x3E,	/*曝光补偿开*/
//	VL_VISCA_EXPOSURE_OFF = 0xE,	/*曝光补偿重置*/
	VL_VISCA_EXPOSURE_SET = 0x4e,	/*曝光补偿设置*/
    VL_VISCA_ZOOM_ELE = 0x06,		/*电子变倍*/
    VL_VISCA_VIEW_SET = 0x47, 		/*视场角切换*/
    VL_VISCA_FOCUS_REQ = 0x48, 	/*焦距查询*/
	VL_VISCA_CONTRAST = 0xA2,		/*对比度*/
	VL_VISCA_SHAR = 0x42,			/*锐度*/
	VL_VISCA_SATURATION = 0x49,	/*饱和度设置*/

} VL_CMD_CTRL_TYPE1;

typedef enum
{
	ADJUST_START,
	ADJUST_ADD,
	ADJUST_MINUS,
	ADJUST_STOP,
	ADJUST_STEP_SET,
	ADJUST_VALUE,
	ADJUST_REQ,
}ADJUST_TYPE_E;

//接收报文结构体
#pragma pack(1)
typedef struct
{
    uint8_t Header1;
    uint8_t Header2;
    uint32_t data_para;
    uint8_t tail1;
    uint8_t tail2;
}VISIBLE_VISCA_RECV_T;
#pragma pack()
#define VISIBLE_RECV_DATA_LEN  sizeof(VISIBLE_VISCA_RECV_T)
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_VISCA_Serial_Data_Init(void);
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
uint8_t Visible_VISCA_Ctrl_SendHandle(uint8_t send_type,uint8_t *ctrl_data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t VISIABLE_VISCA_Period_Handle(void);
#endif
#endif
