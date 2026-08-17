/*===================================================================
 *
 * COPYRIGHT:
 *
 * FILE NAME:
 *	utl_math.c
 * DESCRIPTION:
 *	N/A
 * HISTORY:
 *	V1.00 2013.3.27 By LmnyL, Create/Update
 *
 *=================================================================*/
#include "Common/base_inc.h"
#include "Common/utl_math.h"

//gps起始时间1980年1月6日（星期日）
#define GPS_EPOCH_YEAR 1980
#define GPS_EPOCH_MONTH 1
#define GPS_EPOCH_DAY 6

//每月天数表，非润年
static const uint8_t days_in_month[] =
{31,28,31,30,31,30,31,31,30,31,30,31};

//闰年判断
static bool is_leap_year(uint32_t year)
{
    return (year%4 == 0 && year%100 != 0)||(year%400 == 0);
}
/*==============================================================
*FUNCTION NAME:GPS时间转换函数
*DISCRIPTION:计算基准日期到目标日期的天数
*PARAMETERS:传入周，周内秒；传出年月日时分秒。
*RETURN:true-执行成功；fasle-执行失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool gps_week_to_date(uint32_t gps_week,uint32_t gps_seconds,uint32_t *year,uint32_t *month,uint32_t *day,uint8_t *hour,uint8_t *minutes,uint8_t *sec)
{
    //输入有效性检查
    if (gps_seconds > 604800)
    {
        return false;
    }
    //计算总天数和剩余秒数
    uint32_t total_days = gps_week*7 + gps_seconds/86400;
    //初始化日期为GPS起始时间
    int y = GPS_EPOCH_YEAR;
    int m = GPS_EPOCH_MONTH;
    int d = GPS_EPOCH_DAY;
    //添加天数
    d += total_days;

    hour[0] = gps_seconds / 3600;//小时
    minutes[0] = (gps_seconds % 3600)/60;//分钟
    sec[0] = (gps_seconds % 3600)%60;

    while(1)
    {
        //调整年月日
        uint8_t dim = days_in_month[m-1];
        //处理润二月
        if (m == 2 && is_leap_year(y))
            dim++;
        if (d > dim)
        {
            d -= dim;
            m++;
            if (m>12)
            {
                m = 1;
                m++;
            }
        }
        else
        {
            break;
        }
    }
    //返回结果
    *year = y;
    *month = m;
    *day = d;
    return true;
}
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
 *=================================================================*/
int UTL_Pow(int x, int n)
{
	int i;
	int sum = 1;

	if (n < -1) return 0;

	if (n == -1)
	{
		sum = 1 / x;
	}
	else
	{
		for (i = 0; i < n; i++)
		{
			sum *= x;
		}
	}

	return sum;
}
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
 *=================================================================*/
int UTL_DecToBcd(int x)
{
	int ret_val = 0;
	int value_size = 0;

	do
	{
		if (value_size)
		{
			ret_val += ( (x % 10) * UTL_Pow(16, value_size));
		}
		else
		{
			ret_val += (x % 10);
		}

		value_size ++;
	}
	while (x /= 10);

	return ret_val;
}

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
int UTL_BcdToDec(int x)
{
	int ret_val = 0;
	int value_size = 0;

	do
	{
		if (value_size)
		{
			ret_val += ( (x % 16) * UTL_Pow(10, value_size));
		}
		else
		{
			ret_val += (x % 16);
		}

		value_size ++;
	}while (x /= 16);

	return ret_val;
}
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
 *=================================================================*/
uint32_t UTL_Rand_X(uint32_t x)
{
	return ((rand() % ((x * 2) + 1)) - x);
}

#if (CPU_BYTE_ORDER == CPU_LITTLE_ENDIAN)
/*-------------------------------------------------------------------
 * Function Name:
 *	UTL_Ntohs
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint16_t UTL_Htons(uint16_t data)
{
	return ((data & 0xff) << 8)|((data & 0xff00) >> 8);
}
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
 *-----------------------------------------------------------------*/
uint32_t UTL_Htonl(uint32_t n)
{
	return ((n & 0xff) << 24) |
		((n & 0xff00) << 8) |
		((n & 0xff0000UL) >> 8) |
		((n & 0xff000000UL) >> 24);
}
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
 *-----------------------------------------------------------------*/
uint32_t UTL_Ntohl(uint32_t n)
{
	return UTL_Htonl(n);
}
/*==============================================================
*FUNCTION NAME:角度归一化函数
*DISCRIPTION:使角度落在[-180,180之间]
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_Normalize_Angle(float angle)
{
    if (angle >= 180.0f)
    {
        angle = angle - 360.0f;
    }

    if (angle < -180.0f)
    {
        angle = angle + 360.0f;
    }
    return angle;
}
/*==============================================================
*FUNCTION NAME:角度归一化函数,
*DISCRIPTION:使角度落在[0,360之间]
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_Normalize_Angle_360(float angle)
{
    if (angle >= 360.0f)
    {
        angle = angle - 360.0f;
    }

    if (angle < -180.0f)
    {
        angle = angle + 360.0f;
    }
    return angle;
}
/*==============================================================
*FUNCTION NAME:度转度分秒
*DISCRIPTION：度转度分秒
*PARAMETERS:IN-度
*RETURN:OUT 度、分、秒
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
POSITION_TO_DMS_T UTL_Position_D_To_Dms(double angle_d)
{
//	int32_t tempdata;
    POSITION_TO_DMS_T turn_data;
    //处理符号并取绝对值
    int sign = (angle_d >= 0)?1:-1;
    double abs_degree = fabs(angle_d);
    //提取度
    turn_data.position_d = (int)abs_degree;
    double remainder = (abs_degree - turn_data.position_d)*60.0f;
    //提取分
    turn_data.position_m = (int)remainder;
    //提取秒
    turn_data.positon_s = (remainder - turn_data.position_m)*60.0;
    //恢复符号位到度
    turn_data.position_d *= sign;
    //处理浮点精度问题
    if (turn_data.positon_s >= 60.0)
    {
        turn_data.positon_s -= 60.0;
        turn_data.position_m += 1;
    }
    if (turn_data.position_m >= 60.0)
    {
        turn_data.position_m -= 60.0;
        turn_data.position_d += (turn_data.position_d >= 0)?1:-1;
    }
    return turn_data;
}
/*==============================================================
*FUNCTION NAME:浮点数比大小
*DISCRIPTION：如果两个的差值小于拟定的浮点精度，则为true,认为两个浮点数相等；
否则为false,认为两个浮点数不相等
*PARAMETERS：IN-浮点数
*RETURN:OUT
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#define FLOAT_PRECISION   (0.000001f)    // 浮点数精度
uint8_t float_number_equal(double num1, double num2)
{
    return (fabs(num1 - num2) < FLOAT_PRECISION);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION：视场角计算焦距
*PARAMETERS：float pixel(像元大小),uint16_t pixel_num(分辨率),float view(度)
*RETURN:焦距值(mm)
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_Focus_Calc(float pixel,uint16_t pixel_num,float view)
{
	if ((view > 180)||(view < 0))
	{
		return 0;
	}
	float sensor_size = ((pixel*0.001f)*pixel_num);
	float half_view = view*0.5f;
	float focus = sensor_size/(2.0*tan(half_view));
    return focus;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION：焦距计算视场角
*PARAMETERS：float pixel(像元大小),uint16_t pixel_num(分辨率),float focus(mm)
*RETURN:视场角值（度）半角
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float UTL_View_Calc(float pixel,uint16_t pixel_num,float focus)
{
	float sensor_size = ((pixel*0.001f) * pixel_num);
	float view = (atan(sensor_size/(2.0* focus)));
	view = (view*180)/PI;
    return view;
}
#endif

