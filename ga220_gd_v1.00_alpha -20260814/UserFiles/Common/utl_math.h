/*===================================================================
 *
 * COPYRIGHT:
 *
 * FILE NAME:
 *	utl_math.h
 * DESCRIPTION:
 *	N/A
 * HISTORY:
 *
 *=================================================================*/
#ifndef __UTL_MATH_H__
#define __UTL_MATH_H__
#include "Common/base_inc.h"
#define PI 3.141592653
#define deg2rad(degree) ((degree)/180*PI)/*度转弧度*/
#define rad2deg(radian) ((radian)*180/PI)/*弧度转度*/

typedef struct
{
    double position_d;
    double position_m;
    double positon_s;
}POSITION_TO_DMS_T;


/*判断目标字节里面的某一位是否置高 bit从0开始*/
#ifndef APP_IS_BIT_SET
#define APP_IS_BIT_SET(value,bit) ((value&(1<<bit))?1:0)
#endif
/*将目标字节里面的某一位置高*/
#ifndef APP_SET_BIT
#define APP_SET_BIT(value,bit) (value |= 1<<bit)
#endif
/*将目标字节里面的某一位置低*/
#ifndef APP_CLEAR_BIT
#define APP_CLEAR_BIT(value,bit) ((value &= ~(1<<bit)))
#endif
/*===================================================================
 * FUNCTION NAME:
 *	UTL_Pow
 * DESCRIPTION:
 *	To calc x^n; such as 2^3 = 8
 * PARAMETERS:
 *	x         :
 *	n         :
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.9.11 by LmnyL, Create
 *=================================================================*/
int UTL_Pow(int x, int n);
/*===================================================================
 * FUNCTION NAME:
 *	UTL_DecToBcd
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	x         :
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.9.11 by LmnyL, Create
 *=================================================================*/
int UTL_DecToBcd(int x);

/*===================================================================
 * FUNCTION NAME:
 *	UTL_BcdToDec
 * DESCRIPTION:
 *	Translate BCD value to int value
 * PARAMETERS:
 *	x         :
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.9.11 by LmnyL, Create
 *=================================================================*/
int UTL_BcdToDec(int x);
/*===================================================================
 * FUNCTION NAME:
 *	UTL_Rand_X
 * DESCRIPTION:
 *	产生从-X到X之间的随机数
 * PARAMETERS:
 *	x         :
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2016.12.27 by LmnyL, Create
 *=================================================================*/
uint32_t UTL_Rand_X(uint32_t x);

#if (CPU_BYTE_ORDER == CPU_LITTLE_ENDIAN)
/*-------------------------------------------------------------------
 * Function Name:
 *	UTL_Htons
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
uint16_t UTL_Htons(uint16_t data);
/*-------------------------------------------------------------------
 * Function Name:
 *	UTL_Htonl
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
uint32_t UTL_Htonl(uint32_t n);
/*-------------------------------------------------------------------
 * Function Name:
 *	UTL_Ntohl
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
uint32_t UTL_Ntohl(uint32_t n);
#endif
/*-------------------------------------------------------------------
 * Function Name:
 *	UTL_Ntohll
 * Description:
 *	N/A
 * Parameters:
 *	val       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
float UTL_Focus_Calc(float pixel,uint16_t pixel_num,float view);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION：焦距计算视场角
*PARAMETERS：float pixel,uint16_t pixel_num,float focus(mm)
*RETURN:视场角值（度）
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_View_Calc(float pixel,uint16_t pixel_num,float focus);
/*==============================================================
*FUNCTION NAME:度转度分秒
*DISCRIPTION：度转度分秒
*PARAMETERS:IN-度
*RETURN:OUT 度、分、秒
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
POSITION_TO_DMS_T UTL_Position_D_To_Dms(double angle_d);
/*==============================================================
*FUNCTION NAME:角度归一化函数
*DISCRIPTION:使角度落在[-180,180之间]
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_Normalize_Angle(float angle);
#endif
