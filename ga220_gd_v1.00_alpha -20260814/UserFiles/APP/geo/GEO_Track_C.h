#ifndef __GEO_TRACK_C_H
#define __GEO_TRACK_C_H

//---------光轴惯性空间指向角相关----------------//
typedef struct
{
    double longitude;
    double lattitude;
    double height;
    float Roll;
    float Pitch;
    float Yaw;
    float Vel_E;//东向速度
    float Vel_N;//北向速度
    float Vel_S;//天向速度，东北天坐标系
} INSTypeDef;
extern INSTypeDef INS;//惯导,经纬度为弧度，姿态角为角度,速度为m/s,不再区分惯导来源
extern INSTypeDef INS_Inner;//惯导,经纬度为弧度，姿态角为角度,速度为m/s,不再区分惯导来源

typedef struct
{
    float Fw;
    float Fy;
    float Hg;
} PTangleTypeDef;
extern PTangleTypeDef PT_Angle;//平台角,角度

typedef struct
{
    double longitude;
    double lattitude;
    float height;
} TargetTypeDef;
extern TargetTypeDef Target;//目标地理,弧度

typedef struct
{
    double longitude;
    double lattitude;
    float height;
} GPSOrientTypeDef;
extern GPSOrientTypeDef GPS_Orient_Result;//单目标地理定位,弧度

typedef struct
{
    double longitude[10];
    double lattitude[10];
    float height[10];
    int   type[10];
} MT_GPSOrientTypeDef;
extern MT_GPSOrientTypeDef MT_GPS_Orient_Result;//多目标地理定位,弧度,基于图像

typedef struct
{
    float Angletbl_X[10];
    float Angletbl_Y[10];
} Target_Angletbl_in_Pic_TypeDef;
extern Target_Angletbl_in_Pic_TypeDef Target_Angletbl_in_Pic;//tblx/f,tbly/f,f为焦距；

typedef struct
{
    float Vel;
    float Yaw;
} Target_Motion_TypeDef;
extern Target_Motion_TypeDef Target_Motion;//计算出的目标在二维平面上的移动速度和移动方向，m/s，°

typedef struct
{
    float Fw;
    float Fy;
} LMC_TypeDef;
extern LMC_TypeDef LMC;//°/s


typedef struct
{
    float Orient_Pitch;
    float Orient_Roll;
    float Orient_Yaw;
} OrientTypeDef;
extern OrientTypeDef Orient;//平台空间指向角，°

typedef struct
{
    float Orient_X;
    float Orient_Y;
    float Orient_Z;
    float Orient_Load_FY;
    float Orient_Load_FW;//载荷自身坐标系下指向角
} OrientLoadTypeDef;
extern OrientLoadTypeDef OrientLoad_PT;//平台自身指向角，°
extern OrientLoadTypeDef OrientLoad_Inner;

typedef struct
{
    float Pitch;
    float Yaw;
    float Roll;
} TGTTypeDef;
extern TGTTypeDef TGTV,TGTH_R_P;//目标引导角，空间坐标系，°

typedef struct
{
    float Fw;
    float Fy;
    float Hg;
} BaseVelTypeDef;
extern BaseVelTypeDef BaseVel;//基座角速度，°/s

typedef struct
{
    float Fw;
    float Fy;
    float Hg;
} Load_fr_BaseVelTypeDef;
extern Load_fr_BaseVelTypeDef Load_fr_BaseVel;//基座传递到载体的扰动角速度

typedef struct
{
    float w_Yaw;
    float w_Pitch;
    float w_Roll;
    float w_Fw;
    float w_Fy;
} Vel_CalTypeDef;
extern Vel_CalTypeDef Vel_Cal_Result;//基座传递到载体的扰动角速度

typedef struct
{
    float Sin_Pitch;
    float Cos_Pitch;
    float Sin_Yaw;
    float Cos_Yaw;
    float Sin_Roll;
    float Cos_Roll;	//惯空角度三角函数

    float Sin_Fw;
    float Cos_Fw;
    float Sin_Fy;
    float Cos_Fy;
    float Sin_Hg;
    float Cos_Hg;			//平台角度三角函数
} Triangle_Fun_Cal_TypeDef;
extern Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result;//基座传递到载体的扰动角速度
extern Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result_Inner;//基座传递到载体的扰动角速度

//-----------------功能函数---------------------------//
//-------------三角函数统一计算--------------------//
Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal(INSTypeDef INS,PTangleTypeDef PT_Angle);
//--------------输入角度单位：°----------------//;
//---------角度单位：°---------------//

//---------------平台自身角度至空间指向角转换计算---------------------//
OrientTypeDef Orient_Convert(Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result);//方位、俯仰
OrientTypeDef Orient_Convert_1(Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result);//横滚、俯仰
//-------------平台空间指角至自身角度转换计算---------------//
OrientLoadTypeDef OrientLoadCal(float Exp_Orient_Pitch,float Exp_Orient_Yaw,Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result);
//---------角度单位：°---------------//

//-------------------目标地理跟踪解算，结果为平台空间指向角-----------//
TGTTypeDef TGT(TargetTypeDef T,INSTypeDef INS);
//---------经纬度单位：rad---------------//

//-------------------目标地理跟踪解算，结果为平台自身横滚、俯仰角，待验证-----------//
TGTTypeDef TGT_R_P(TargetTypeDef T,INSTypeDef INS);
//---------经纬度单位：rad---------------//
//-----------目标地理定位-------------------//
GPSOrientTypeDef Target_GPS_Cal(INSTypeDef INS,float Laser_Distance,OrientTypeDef Orient); //计算目标经纬高,此处INS是安装在载荷舱内的平台惯导数据，可通过机载惯导和平台自身角度计算出
MT_GPSOrientTypeDef Multi_Target_GPS_Cal(INSTypeDef INS,float Laser_Distance,OrientTypeDef Orient,Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result,Target_Angletbl_in_Pic_TypeDef Target_Angletbl_in_Pic,int Targetnum);//多目标定位,此处INS是安装在载荷舱内的平台惯导数据，可通过机载惯导和平台自身角度计算出
//------经纬度单位：rad；距离单位m;角度单位：°-----------------------//

//-----------------------------基座角速度到载荷角速度耦合计算-------------------------//
Load_fr_BaseVelTypeDef Load_fr_Base_VelCal(Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result,BaseVelTypeDef BaseVel);
//--------------------LMC---------------------------//
LMC_TypeDef LMC_Cal(Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result,INSTypeDef INS,float detaH);
//---------速度单位：m/s,输出角速度单位°/s---------------//
//--------------目标速度估计-------------------//
Target_Motion_TypeDef Target_Motion_Cal(Triangle_Fun_Cal_TypeDef Triangle_Fun_Cal_Result,INSTypeDef INS,Vel_CalTypeDef Vel_Cal_Result,float D_Laser,float V_D_Laser);
//---------输入速度单位：m/s,输入角速度rad/s，输出角速度单位°/s,输出角度单位°---------------//
//------------运算函数---------------------//
void Matrix_Multiply(float In_A[3][3],float In_B[3][3],float Out[3][3]);
void Matrix_Multiply_4_1(float In_A[4][4],float In_B[4][1],float Out[4][1]);
void Matrix_Multiply_4_4(float In_A[4][4],float In_B[4][4],float Out[4][4]);
void Matrix_inv_cal(float **A_matrix,int N,float **A_matrix_inv);
//------------------三角函数计算------------------//
float Get_Sinvalue(float Angle);//取值范围-180~180，分辨率0.1
float Get_Cosvalue(float Angle);//取值范围-180~180，分辨率0.1
float Get_Tanvalue(float Angle);//取值范围-180~180，分辨率0.1
//------------------------------------------------//
Vel_CalTypeDef Vel_Cal(INSTypeDef INS,PTangleTypeDef PT_Angle,float SamT_INS);
void far_near_dist(float temp_x, float temp_y, float temp_r, float angle, float obj_real[3], int flag);
//--------------输入角度单位：°----------------------------//

#endif
