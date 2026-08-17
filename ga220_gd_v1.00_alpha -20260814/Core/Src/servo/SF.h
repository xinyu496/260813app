#include "stdint.h"
#include "SFlibhead.h"
////----------变量定义---------------//
////-----------伺服指令------------//

extern float FY_Mech_Angle;

#define Brake              0x01   //刹车
#define VLead              0x02   //速度运动
#define ALead              0x03   //角度运动
#define GEOLead            0x04   //地理引导
#define Track              0x05   //跟踪模式
#define Withdrawal         0x06   //收藏
#define LOCK               0x07   //上锁
#define DELOCK             0x08   //解锁
#define CLB                0x09   //校正
#define STEP               0x0A   //步进

//------------伺服工作模式----------//
#define P_Brake              0x01   //刹车
#define P_VLead              0x02   //速度运动
#define P_ALead              0x03   //角度运动
#define P_GEOLead            0x04   //地理引导
#define P_Track              0x05   //跟踪模式
#define P_Withdrawal         0x06   //收藏
#define P_LOCK               0x07   //上锁
#define P_DELOCK             0x08   //解锁
#define P_CLB                0x09   //校正
#define P_STEP               0x0A   //步进



#define GET_FOV_UNIT_RADIAN   1
#define PI (acos(-1.0f))
#define Pi 3.1415926
#define Bmq_cof  0.000171661377//21位编码器
#define Bmq_cof_rad 0.157379
#define Angle_cof_rad 0.01745326
#define Angle_cof_degree 57.29578

#define Up_Limit 85
#define Down_Limit  -3  
#define Left_Limit  -181
#define Right_Limit  181

//------------调试变量定义-------------//
#define FY_axis 1
#define FW_axis 2

#define FY_O    0
#define FY_C    1
#define FW_O    0
#define FW_C    1

#define FY_IL   1
#define FY_AL   2
#define FY_VL   3
#define FY_TL   4
#define FY_GL   5
#define FY_EVL  6
#define FY_PL   7

#define FW_IL   1
#define FW_AL   2
#define FW_VL   3
#define FW_TL   4
#define FW_GL   5
#define FW_EVL  6
#define FW_PL   7

#define TRACK   1
#define KEEP    2
#define STOP    3
#define MOVE    4
//--------------伺服变量-------------------//
////----------伺服变量-----------//
//--------高低--------------//
typedef struct
{
  float U_give;
	float Kp_I;
	float Ki_I;
	float SamT_I;
	float Umax;
	float I_give;
	float I_fb;//电流环控制参数
	
	float Kp_A;
	float Ki_A;
	float SamT;
	float Kg_A;
	float BW_A;
	float Imax;
	float A_give;
	float A_fb;//加速度
	
	float Kp_V;
	float Ki_V;
	float Amax;
  float V_give;
	float V_fb;//速度
	
	float Kp_Eacc;
	float Ki_Eacc;
	float Eamax;
  float Eacc_give;
	float Eacc_fb;//轴角加速度
	
	float Kp_Ev;
	float Ki_Ev;
  float Ev_give;
	float Ev_fb;//轴角速度
	
	float Kp_P;
	float Ki_P;
	float bound_P;
	float Ev_Set;
	float Evmax;
	float P_give;
	float P_fb;//角位置
	
	float Kp_1_T1;
	float Ki_1_T1;
	float bound_T1;
	float Kp_2_T1;
	float Ki_2_T1;
	float Vmax;
	
	float Kp_1_T2;
	float Ki_1_T2;
	float bound_T2;
	float Kp_2_T2;
	float Ki_2_T2;
	
	float Kp_1_T3;
	float Ki_1_T3;
	float bound_T3;
	float Kp_2_T3;
	float Ki_2_T3;
	
	float Kp_G;
	float Ki_G;
	float bound_G;
	float V_Set;
	float GEO_give;
	float GEO_fb;
	
	float T_fb;
	
	int timer;
	int Danger_count;
	float T_disturb;
  }GDControlTypeDef;
extern GDControlTypeDef GDControl;
//---------方位---------------//
typedef struct
{
	float U_give;
	
  float Kp_I;
	float Ki_I;
	float SamT_I;
	float Umax;
	float I_give;
	float I_fb;//电流环控制参数
	
	float Kp_A[6];
	float Ki_A[6];
	float SamT;
	float Kg_A;
	float BW_A[6];
	float Imax;
	float A_give;
	float A_fb;//加速度
	float bound_A[5];
	
	float Kp_V[6];
	float Ki_V[6];
	float Amax;
  float V_give;
	float V_fb;//速度
	float bound_V[5];
	
	float Kp_Eacc;
	float Ki_Eacc;
	float Eamax;
  float Eacc_give;
	float Eacc_fb;//轴角加速度
	
	float Kp_Ev;
	float Ki_Ev;
  float Ev_give;
	float Ev_fb;//轴角速度
	
	float Kp_P;
	float Ki_P;
	float bound_P;
	float Ev_Set;
	float Evmax;
	float P_give;
	float P_fb;//角位置
	
	float Kp_1_T1;
	float Ki_1_T1;
	float bound_T1;
	float Kp_2_T1;
	float Ki_2_T1;
	float Vmax;
	
	float Kp_1_T2;
	float Ki_1_T2;
	float bound_T2;
	float Kp_2_T2;
	float Ki_2_T2;
	
	float Kp_1_T3;
	float Ki_1_T3;
	float bound_T3;
	float Kp_2_T3;
	float Ki_2_T3;
	
	float Kp_G;
	float Ki_G;
	float bound_G;
	float V_Set;
	float GEO_give;
	float GEO_fb;
	
	float T_fb;
	
	int timer;
	int Danger_count;
	float T_disturb;
  }FWControlTypeDef;
extern FWControlTypeDef FWControl;
//---------伺服数据处理------------//
//---------高低----------------//
typedef struct  
{
	float Current;
	uint32_t BMQDATA_origin;
	uint32_t BMQDATA_origin_old;
	float BMQDATA_old;
	float BMQDATA_change;
	float BMQDATA_change0;
	int32_t BMQDATA_zero;
  int32_t BMQDATA;
	int32_t lap;
	double angle_Vel;
	
	float GEO_angle;
	float gyro;
	float gyrobias;
	float acc;
	float angle;
	float angle_init;
	float ev;
	float eacc;
	int8_t scandir;
	float sacn_v_limit;
	
	float Tbl_angle;
	
	float Cos_GD;
	float Sec_GD;

	char XW_state;//0：中间；1：上限位；2：下限位	
}GDSensorTypeDef;
extern GDSensorTypeDef GDSensor;
//---------方位----------------//
typedef struct  
{
	float Current;
	uint32_t BMQDATA_origin;
	uint32_t BMQDATA_origin_old;
	int32_t BMQDATA_zero;
  int32_t BMQDATA;
	int32_t lap;
	double angle_Vel;
	
	float GEO_angle;
	float gyro;
	float gyrobias;
	float gyro_Hg;
	float gyrobias_Hg;
	float acc;
	float acc_Hg;
	float angle;
	float angle_image;
	float angle_init;
	float ev;
	float eacc;
	int8_t scandir;
	float sacn_v_limit;
	
	float Tbl_angle;

	char XW_state;//0：中间；1：左限位；2：右限位
}FWSensorTypeDef;
extern FWSensorTypeDef FWSensor;


/*伺服动作相关结构体   start*/
typedef struct
{
	uint8_t Cmd;
	uint8_t DJEnable;
	int16_t Para;
	uint8_t SSCtrl;
}FWServoTypedef;
extern FWServoTypedef FWServo;

typedef struct
{
	uint8_t Cmd;
	uint8_t DJEnable;
	int16_t Para;
	uint8_t SSCtrl;
}FYServoTypedef;
extern FYServoTypedef FYServo;

//限位结构体
typedef struct 
{
  uint8_t UP_XW_flag;
	uint8_t Down_XW_flag;
	uint8_t FY_XW_flag;
	uint8_t Left_XW_flag;
	uint8_t Right_XW_flag;
	uint8_t FW_XW_flag;
}SevroXWStateTypDef;
extern SevroXWStateTypDef SevroXWState;



//---------数据处理---------//
void gdbmqdispose(void);
void fwbmqdispose(void);
void fwbmqdispose_1(void);
void gyrodispose(void);
void fbdispose(void);
void ReadXW(void);
void tb2angle(void);
void dgfil(void);
//---------------//
//----电机控制------//
void gd_en(void);
void fw_en(void);
void FW_PWM(float fwugive);
void gd_brake(void);
void fw_brake(void);
void all_brake(void);
//-------------------//
//--------功能函数----//
void initcontrolpar(void);
void initdatapar(void);
void alldeal(void);
void allprocess(void);
void commandprocess(void);
void Data_format_Trans(void);
void InitCommand(void);
void CurrentInit(void);
//-------调试函数---------//
void Test(char Axis,char Loop,char O_or_C,float Amp,float Freq,float Time);//轴向，环路，开闭环、幅值、频率，测试时间；
//------定义完毕-----------//