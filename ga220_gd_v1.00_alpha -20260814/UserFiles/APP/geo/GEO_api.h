#ifndef __GEO_API_H
#define __GEO_API_H

/*地理数据帧*/
__packed typedef struct
{
    /*地理跟踪外部输入坐标*/
    float TargetLon;
    float TargetLat;
    float TargetAlt;
    /*地理跟踪坐标斜距
    单位：m*/
    float TargetDis_Calu;

    /*地理跟踪伺服引导值 :以下两组变量根据实际情况调用
    单位：度（°） */
    /*基于外惯导：该值为相对于机械零位的差值*/
    float Heading_FrPT;
    float Pitch_FrPT;
    /*基于内惯导：该值为相对于目标的差值*/
    float Heading_FrInner;
    float Pitch_FrInner;

    /*LMC伺服引导值
    单位：度（°/s） */
    float LMC_Fw;
    float LMC_Fy;

    /*目标斜距
    来源1：激光测距值
    来源2：计算斜距（可由EO姿态得出）*/
    float Slant_Dis;

    /*目标定位计算的坐标值*/
    double CalcLon;
    double CalcLat;
    double CalcAlt;
} GEO_TRACK_DATA_T;
/***********************************************************/
void USER_Ctrl_GEO_process(void);
float TGT3(void);
#endif