#include "math.h"
#include "GEO_Track_C.h"
#include "../geo/GEO_File.h"
//--------------地理跟踪------------//
//-----------多目标定位------------//
//-----------目标速度估算---------//
float V_LaserDistance;
float LaserDistance;
extern double EOTarget_Lon,EOTarget_Lat;
extern float EOTarget_Alt;
extern INSTypeDef INS;
INSTypeDef INS_Inner;
Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result_Inner;
OrientLoadTypeDef OrientLoad_PT;
OrientLoadTypeDef OrientLoad_Inner;


void GEO_para_init(void)
{
    TGTV.Pitch = 0;
    TGTV.Roll = 0;
    TGTV.Yaw = 0;

    GPS_Orient_Result.height = 0;
    GPS_Orient_Result.lattitude = 0;
    GPS_Orient_Result.longitude = 0;
    Target_Motion.Vel = 0;
    Target_Motion.Yaw = 0;

    Triangle_Fun_Cal_Result.Cos_Fw = 0;
    Triangle_Fun_Cal_Result.Sin_Fw = 0;
    Triangle_Fun_Cal_Result.Cos_Fy = 0;
    Triangle_Fun_Cal_Result.Sin_Fy = 0;
    Triangle_Fun_Cal_Result.Cos_Hg = 0;
    Triangle_Fun_Cal_Result.Sin_Hg = 0;
    Triangle_Fun_Cal_Result.Cos_Pitch = 0;
    Triangle_Fun_Cal_Result.Sin_Pitch = 0;
    Triangle_Fun_Cal_Result.Cos_Roll = 0;
    Triangle_Fun_Cal_Result.Sin_Roll = 0;
    Triangle_Fun_Cal_Result.Cos_Yaw = 0;
    Triangle_Fun_Cal_Result.Sin_Yaw = 0;

    OrientLoad_PT.Orient_Load_FW = 0;
    OrientLoad_PT.Orient_Load_FY = 0;
    OrientLoad_PT.Orient_X = 0;
    OrientLoad_PT.Orient_Y = 0;
    OrientLoad_PT.Orient_Z = 0;

    Vel_Cal_Result.w_Fw = 0;
    Vel_Cal_Result.w_Fy = 0;
    Vel_Cal_Result.w_Pitch = 0;
    Vel_Cal_Result.w_Roll = 0;
    Vel_Cal_Result.w_Yaw = 0;

    Orient.Orient_Pitch = 0;
    Orient.Orient_Roll = 0;
    Orient.Orient_Yaw = 0;

    LMC.Fw = 0;
    LMC.Fy = 0;

    for(uint8_t n=0; n<10; n++)
    {
        MT_GPS_Orient_Result.height[n] = 0;
        MT_GPS_Orient_Result.lattitude[n] = 0;
        MT_GPS_Orient_Result.longitude[n] = 0;
    }
}

void GEO(void)
{
    TGTV = TGT(Target,INS_Inner);//目标与载机连线在大地系下的角度

    Triangle_Fun_Cal_Result = Triangle_Fun_Cal(INS,PT_Angle);
    OrientLoad_PT = OrientLoadCal(TGTV.Pitch,TGTV.Yaw,Triangle_Fun_Cal_Result);

    Triangle_Fun_Cal_Result_Inner = Triangle_Fun_Cal(INS_Inner,PT_Angle);
    OrientLoad_Inner = OrientLoadCal(TGTV.Pitch,TGTV.Yaw,Triangle_Fun_Cal_Result_Inner);
}

void Target_Orient_Fun(void)
{
//------------解算----------------//
    Triangle_Fun_Cal_Result = Triangle_Fun_Cal(INS,PT_Angle);
    Orient = Orient_Convert_1(Triangle_Fun_Cal_Result);
    GPS_Orient_Result = Target_GPS_Cal(INS,LaserDistance,Orient);
}

float target_vel = 0.0,target_dir = 0.0;
void Target_Vel_Estimate(void)
{
    Vel_Cal_Result = Vel_Cal(INS,PT_Angle,0.01);
    Target_Motion = Target_Motion_Cal(Triangle_Fun_Cal_Result,INS,Vel_Cal_Result,LaserDistance,V_LaserDistance);
//  Target_Motion = Target_Motion_Cal(Triangle_Fun_Cal_Result,INS,Vel_Cal_Result,LaserDistance,0);
}
void LMC_Vel_Cal()
{
    Triangle_Fun_Cal_Result = Triangle_Fun_Cal(INS,PT_Angle);

   // LMC = LMC_Cal(Triangle_Fun_Cal_Result,INS,alt); // 注意：待处理
}

/****************************************************
 功能：目标距离计算
 输入：载机当前地理位置 目标当前地理坐标位置
 输出：TargeTDis两个地理位置之间的斜距
 ***************************************************/
#define a 6378137.000
#define dEcc1 0.006694380066764658
float GEO_Target_Get_Dis(void)
{
    float TargeTDis = 0.0;


    float dX_G[2], dY_G[2], dZ_G[2];
    // float dX_NED, dY_NED ,dZ_NED ,dX_NED1, dY_NED1 ,dZ_NED1;
    // float dX_NED_T, dY_NED_T ,dZ_NED_T;
    float dNr[2];


    float Sin_PT_long = 0.0;
    float Cos_PT_long = 1.0;
    float Sin_PT_lat = 0.0;
    float Cos_PT_lat = 1.0;


    float Sin_Target_long = 0.0;
    float Cos_Target_long = 1.0;
    float Sin_Target_lat = 0.0;
    float Cos_Target_lat = 1.0;


    Sin_Target_long = sin(Target.longitude);
    Cos_Target_long = cos(Target.longitude);
    Sin_Target_lat = sin(Target.lattitude);
    Cos_Target_lat = cos(Target.lattitude);


    Sin_PT_long = sin(INS.longitude);
    Cos_PT_long = cos(INS.longitude);
    Sin_PT_lat = sin(INS.lattitude);
    Cos_PT_lat = cos(INS.lattitude);


    dNr[0] = a / (sqrt(1 - dEcc1 * pow(Sin_PT_lat, 2)));
    dX_G[0] = (dNr[0] + INS.height) * Cos_PT_lat * Cos_PT_long;
    dY_G[0] = (dNr[0] + INS.height) * Cos_PT_lat * Sin_PT_long;
    dZ_G[0] = (dNr[0] * (1 - dEcc1) + INS.height) * Sin_PT_lat;


    dNr[1] = a / (sqrt(1 - dEcc1 * pow(Sin_Target_lat, 2)));
    dX_G[1] = (dNr[1] + Target.height) * Cos_Target_lat * Cos_Target_long;
    dY_G[1] = (dNr[1] + Target.height) * Cos_Target_lat * Sin_Target_long;
    dZ_G[1] = (dNr[1] * (1 - dEcc1) + Target.height) * Sin_Target_lat;


    TargeTDis = sqrt(pow((dX_G[0] - dX_G[1]), 2) + pow((dY_G[0] - dY_G[1]), 2) + pow((dZ_G[0] - dZ_G[1]), 2));
    return TargeTDis;
}

