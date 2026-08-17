#ifndef _SFLIBHEAD_H_
#define _SFLIBHEAD_H_

#define degree_to_rad 0.01745329
#define rad_to_degree 57.2957795
//------------伺服变量定义-------------//
//---------高低----------------//
typedef struct {
    float igive;  //电流环
    float ierror;
    float ierr[2];
    float icon[2];
    float ugive;

    float agive;  //加速度环
    float aerror;
    float aerr[2];
    float acon[2];
    float amodel_in[2];
    float amodel_out[2];
    float acmp_in[2];
    float acmp_out[2];

    float vgive;  //速度环
    float scanvgive;
    float verror;
    float verr[2];
    float vcon[2];

    float eagive;  //轴角加速度环
    float eaerror;
    float eaerr[2];
    float eacon[2];

    float evgive;  //轴角速度环
    float everror;
    float everr[2];
    float evcon[2];

    float pgive;  //位置环
    float perror;
    float perr[2];
    float pcon[2];

    float terror_1;  //跟踪环
    float terr_1[2];
    float tcon_1[2];
    float tcmp_in_1[2];
    float tcmp_out_1[2];

    float terror_2;  //跟踪环
    float terr_2[2];
    float tcon_2[2];
    float tcmp_in_2[2];
    float tcmp_out_2[2];

    float terror_3;  //跟踪环
    float terr_3[2];
    float tcon_3[2];
    float tcmp_in_3[2];
    float tcmp_out_3[2];

    float ggive;  //GEO
    float gerror;
    float gerr[2];
    float gcon[2];
    float gcmp_in[2];
    float gcmp_out[2];

    //-------------控制器参数----------//
    float icon_e_cof[2];
    float icon_u_cof[2];
    float U_limit;

    float acon_e_cof[2];
    float acon_u_cof[2];
    float amodel_e_cof[2];
    float amodel_u_cof[2];
    float acmp_e_cof[2];
    float acmp_u_cof[2];
    float i_limit;

    float vcon_e_cof[2];
    float vcon_u_cof[2];
    float a_limit;

    float eacon_e_cof[2];
    float eacon_u_cof[2];

    float evcon_e_cof[2];
    float evcon_u_cof[2];
    float ea_limit;

    float pcon_e_cof[2];
    float pcon_u_cof[2];
    float ev_limit;

    float tcon_e_cof_1[2];
    float tcon_u_cof_1[2];
    float tcon_e_cof_2[2];
    float tcon_u_cof_2[2];
    float tbl_linear_set;
    float v_limit;

    float gcon_e_cof[2];
    float gcon_u_cof[2];
    float gerr_linear_set;
    float gcmp_e_cof[2];
    float gcmp_u_cof[2];
    //---------------------------//
} GDservoTypeDef;
extern GDservoTypeDef GDservo;
//---------方位----------------//
typedef struct {
    float igive;  //电流环
    float ierror;
    float ierr[2];
    float icon[2];
    float ugive;

    float agive;  //加速度环
    float aerror;
    float aerr[2];
    float acon[2];
    float amodel_in[2];
    float amodel_out[2];
    float acmp_in[2];
    float acmp_out[2];

    float vgive;  //速度环
    float scanvgive;
    float verror;
    float verr[2];
    float vcon[2];

    float eagive;  //轴角加速度环
    float eaerror;
    float eaerr[2];
    float eacon[2];

    float evgive;  //轴角速度环
    float everror;
    float everr[2];
    float evcon[2];

    float pgive;  //位置环
    float perror;
    float perr[2];
    float pcon[2];

    float terror_1;  //跟踪环
    float terr_1[2];
    float tcon_1[2];
    float tcmp_in_1[2];
    float tcmp_out_1[2];

    float terror_2;  //跟踪环
    float terr_2[2];
    float tcon_2[2];
    float tcmp_in_2[2];
    float tcmp_out_2[2];

    float terror_3;  //跟踪环
    float terr_3[2];
    float tcon_3[2];
    float tcmp_in_3[2];
    float tcmp_out_3[2];

    float ggive;  //GEO
    float gerror;
    float gerr[2];
    float gcon[2];
    float gcmp_in[2];
    float gcmp_out[2];

    float T_disturb;  //模拟干扰
                      //-------------控制器参数----------//
    float icon_e_cof[2];
    float icon_u_cof[2];
    float U_limit;

    float acon_e_cof[2];
    float acon_u_cof[2];
    float amodel_e_cof[2];
    float amodel_u_cof[2];
    float acmp_e_cof[2];
    float acmp_u_cof[2];
    float i_limit;

    float vcon_e_cof[2];
    float vcon_u_cof[2];
    float a_limit;

    float eacon_e_cof[2];
    float eacon_u_cof[2];

    float evcon_e_cof[2];
    float evcon_u_cof[2];
    float ea_limit;

    float pcon_e_cof[2];
    float pcon_u_cof[2];
    float ev_limit;

    float tcon_e_cof_1[2];
    float tcon_u_cof_1[2];
    float tcon_e_cof_2[2];
    float tcon_u_cof_2[2];
    float tbl_linear_set;
    float v_limit;
    float v_max;

    float gcon_e_cof[2];
    float gcon_u_cof[2];
    float gerr_linear_set;
    float gcmp_e_cof[2];
    float gcmp_u_cof[2];
    //------------------------//
} FWservoTypeDef;
extern FWservoTypeDef FWservo;
//-----------------数据处理-----------------//
//---------伺服数据处理------------//
//---------高低----------------//
typedef struct {
    float current_in[3];
    float current_out[3];

    float gyrobutter_in[3];
    float gyrobutter_out[3];

    float gyronotch_1_in[3];
    float gyronotch_1_out[3];
    float gyronotch_2_in[3];
    float gyronotch_2_out[3];
    float gyronotch_3_in[3];
    float gyronotch_3_out[3];

    float acc_dis_in[2];
    float acc_dis_out[2];

    float eacc_dis_in[2];
    float eacc_dis_out[2];

    float ev_in[3];
    float ev_out[3];

    float dg_in[2];
    float dg_out[2];
    //----------处理参数----------//
    float current_e_cof[3];
    float current_u_cof[3];

    float gyrobutter_e_cof[3];
    float gyrobutter_u_cof[3];

    float gyronotch_1_e_cof[3];
    float gyronotch_1_u_cof[3];
    float gyronotch_2_e_cof[3];
    float gyronotch_2_u_cof[3];
    float gyronotch_3_e_cof[3];
    float gyronotch_3_u_cof[3];

    float acc_dis_e_cof[2];
    float acc_dis_u_cof[2];

    float eacc_dis_e_cof[2];
    float eacc_dis_u_cof[2];

    float ev_e_cof[3];
    float ev_u_cof[3];

    float dg_e_cof[2];
    float dg_u_cof[2];
} GDDataTypeDef;
extern GDDataTypeDef GDData;
//---------方位----------------//
typedef struct {
    float current_in[3];
    float current_out[3];

    float gyrobutter_fw_in[3];
    float gyrobutter_fw_out[3];

    float gyronotch_fw_1_in[3];
    float gyronotch_fw_1_out[3];
    float gyronotch_fw_2_in[3];
    float gyronotch_fw_2_out[3];
    float gyronotch_fw_3_in[3];
    float gyronotch_fw_3_out[3];

    float gyrobutter_hg_in[3];
    float gyrobutter_hg_out[3];

    float gyronotch_hg_1_in[3];
    float gyronotch_hg_1_out[3];
    float gyronotch_hg_2_in[3];
    float gyronotch_hg_2_out[3];
    float gyronotch_hg_3_in[3];
    float gyronotch_hg_3_out[3];

    float acc_dis_fw_in[2];
    float acc_dis_fw_out[2];
    float acc_dis_hg_in[2];
    float acc_dis_hg_out[2];

    float eacc_dis_fw_in[2];
    float eacc_dis_fw_out[2];

    double ev_in[3];
    double ev_out[3];

    float dg_in[2];
    float dg_out[2];
    //----------处理参数----------//
    float current_e_cof[3];
    float current_u_cof[3];

    float gyrobutter_e_cof[3];
    float gyrobutter_u_cof[3];

    float gyronotch_1_e_cof[3];
    float gyronotch_1_u_cof[3];
    float gyronotch_2_e_cof[3];
    float gyronotch_2_u_cof[3];
    float gyronotch_3_e_cof[3];
    float gyronotch_3_u_cof[3];

    float acc_dis_e_cof[2];
    float acc_dis_u_cof[2];

    float eacc_dis_e_cof[2];
    float eacc_dis_u_cof[2];

    float ev_e_cof[3];
    float ev_u_cof[3];

    float dg_e_cof[2];
    float dg_u_cof[2];
} FWDataTypeDef;
extern FWDataTypeDef FWData;

//--------伺服算法函数定义----------//
float gdiloop(float Kp, float Ki, float SamT, float Umax, float I_give, float I_fb);
//高低电流环：PI比例系数，PI积分系数，采样周期，输出控制电压最大值，电流给定，电流反馈

float gdaloop(float Kp, float Ki, float SamT, float Kg, float BW, float Imax, float A_give, float A_fb);
//高低陀螺加速度环：PI比例系数，PI积分系数，采样周期，加速度传函增益、补偿带宽、输出电流控制量最大值，加速度给定，加速度反馈；

float gdvloop(float Kp, float Ki, float SamT, float Amax, float Up_Limit, float Dwn_Limit, float V_give,
              float V_fb, float gd_angle);
//高低陀螺速度环：PI比例系数，PI积分系数，采样周期，输出加速度控制量最大值，上限位角度，下限位角度，速度给定，速度反馈，高低角；

float gdealoop(float Kp, float Ki, float SamT, float Imax, float eA_give, float eA_fb, char deadzone_flag);
//高低轴角加速度环：PI比例系数，PI积分系数，采样周期，输出电流控制量最大值，加速度给定，加速度反馈；

float gdevloop(float Kp, float Ki, float SamT, float Eamax, float Up_Limit, float Dwn_Limit, float Ev_give,
               float Ev_fb, float gd_angle, char deadzone_flag);
//高低轴角速度环：PI比例系数，PI积分系数，采样周期，输出电流控制量最大值，上限位角度，下限位角度，轴角速度给定，轴角速度反馈，高低角；

float gdploop(float Kp, float Ki, float SamT, float Up_Limit, float Dwn_Limit, float bound, float Ev_Set,
              float Evmax, float P_give, float P_fb, char deadzone_flag);
//高低轴角位置环：PI比例系数，PI积分系数，采样周期，上限位角度，下限位角度，控制切换边界角度，固定逼近速度，最大允许速度，角度给定，角度反馈；

// bound 用来判断当前脱靶量是“小误差区”还是“大误差区”。脱靶量小时用一套参数精细跟踪，脱靶量大时用另一套参数快速拉回目标。
float gdtloop_TV1(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
//高低视频源1跟踪环：PI比例系数，PI积分系数，控制切换边界角度，PI比例系数，PI积分系数，采样周期，最大允许跟踪速度，脱靶量角度反馈；

float gdtloop_TV2(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
//高低视频源2跟踪环：PI比例系数，PI积分系数，控制切换边界角度，PI比例系数，PI积分系数，采样周期，最大允许跟踪速度，脱靶量角度反馈；

float gdtloop_TV3(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb);
//高低视频源3跟踪环：PI比例系数，PI积分系数，控制切换边界角度，PI比例系数，PI积分系数，采样周期，最大允许跟踪速度，脱靶量角度反馈；

float gdgeo(float Kp, float Ki, float SamT, float bound, float V_Set, float Vmax, float GEO_give,
            float GEO_fb);
//高低GEO跟踪环：PI比例系数，PI积分系数，采样周期，控制切换边界角度，固定逼近速度，最大允许跟踪速度，GEO角度给定，GEO角度反馈；

float fwiloop(float Kp, float Ki, float SamT, float Umax, float I_give, float I_fb);

float fwaloop(float *Kp, float *Ki, float SamT, float Kg, float *BW, float Imax, float A_give, float A_fb,
              float gd_angle, float *bound);
//方位陀螺加速度环：PI比例系数数组，PI积分系数数组，采样周期，加速度传函增益、补偿带宽、输出电流控制量最大值，加速度给定，加速度反馈，高低角，分段设定角度数组,正割补偿系数；

float fwvloop(float *Kp, float *Ki, float SamT, float Amax, float L_Limit, float R_limit, float V_give,
              float V_fb, float fw_angle, float gd_angle, float *bound);
//方位陀螺速度环：PI比例系数数组，PI积分系数数组，采样周期，输出加速度控制量最大值，左限位角度，右限位角度，速度给定，速度反馈，方位角，高低角，分段设定角度数组；

float fwealoop(float Kp, float Ki, float SamT, float Eamax, float eA_give, float eA_fb, char deadzone_flag);
//方位轴角加速度环：PI比例系数，PI积分系数，采样周期，输出电流控制量最大值，加速度给定，加速度反馈；

float fwevloop(float Kp, float Ki, float SamT, float Imax, float L_Limit, float R_limit, float Ev_give,
               float Ev_fb, float fw_angle, char deadzone_flag);

float fwploop(float Kp, float Ki, float SamT, float L_Limit, float R_Limit, float bound, float Ev_Set,
              float Evmax, float P_give, float P_fb, char deadzone_flag);
float fwtloop_TV1(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
//方位视频源1跟踪环：PI比例系数，PI积分系数，控制切换边界角度，PI比例系数，PI积分系数，采样周期，最大允许跟踪速度，脱靶量角度反馈,正割值；
float fwtloop_TV2(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
float fwtloop_TV3(float Kp_1, float Ki_1, float bound, float Kp_2, float Ki_2, float SamT, float Vmax,
                  float Tbl_angle_fb, float Sec);
float fwgeo(float Kp, float Ki, float SamT, float bound, float V_Set, float Vmax, float GEO_give,
            float GEO_fb, float Sec);
//-------寄存器清除-------//
void clr_gd_i(void);
void clr_gd_a(void);
void clr_gd_v(void);
void clr_gd_ea(void);
void clr_gd_ev(void);
void clr_gd_p(void);
void clr_gd_t_1(void);
void clr_gd_t_2(void);
void clr_gd_t_3(void);
void clr_gd_g(void);
void clr_gd_all(void);

void clr_fw_i(void);
void clr_fw_a(void);
void clr_fw_v(void);
void clr_fw_ea(void);
void clr_fw_ev(void);
void clr_fw_p(void);
void clr_fw_t_1(void);
void clr_fw_t_2(void);
void clr_fw_t_3(void);
void clr_fw_g(void);
void clr_fw_all(void);

void clr_all(void);
//-----------数据处理---------------//
float GdCurrentButter(int order, int BW, float SamT, float Current);
//高低电机电流滤波：阶次,带宽，采样周期，电流输入
float GdGyroButter(int order, float BW, float SamT, float Gyro);
//高低陀螺滤波：阶次,带宽，采样周期，陀螺输入
float GdGyroNotch_1(float freq, float SamT, float Gyro);
float GdGyroNotch_2(float freq, float SamT, float Gyro);
float GdGyroNotch_3(float freq, float SamT, float Gyro);
//高低陀螺陷波波：陷波频率，采样周期，陀螺输入
float GdAccDis(float BW, float SamT, float Gyro);
//高低加速度处理：带宽，采样周期，陀螺输入
float GdAxisEvDis(int order, float BW, float SamT, float angle);
float GdeAccDis(float BW, float SamT, float Gd_ev);
//高低轴角速度处理：阶次、带宽，采样周期，角度输入
float GddgDis(float SamT, float dg);
//高低单杆处理：采样周期，单杆输入

float FwCurrentButter(int order, int BW, float SamT, float Current);
float FwGyroNotch_1(float freq, float SamT, float Gyro);
float FwGyroNotch_2(float freq, float SamT, float Gyro);
float FwGyroNotch_3(float freq, float SamT, float Gyro);
//方位陀螺陷波波：阶次,陷波频率，采样周期，陀螺原始输入，先陷波再滤波
float FwGyroButter(int order, float BW, float SamT, float Gyro, float Sec);
//方位陀螺滤波：阶次,带宽，采样周期，陀螺输入，正割值

float HgGyroNotch_1(float freq, float SamT, float Gyro);
float HgGyroNotch_2(float freq, float SamT, float Gyro);
float HgGyroNotch_3(float freq, float SamT, float Gyro);
//横滚陀螺陷波波：阶次,陷波频率，采样周期，陀螺原始输入，先陷波再滤波
float HgGyroButter(int order, float BW, float SamT, float Gyro, float CSC);
//横滚陀螺滤波：阶次,带宽，采样周期，陀螺输入，余割值
float FwAccDis(float BW, float SamT, float Gyro, float Sec);
float HgAccDis(float BW, float SamT, float Gyro, float CSC);
double FwAxisEvDis(int order, float BW, float SamT, double angle);
float FweAccDis(float BW, float SamT, float Fw_ev);
float FwdgDis(float SamT, float dg);

void clr_gddg(void);
void clr_fwdg(void);
void clr_dg(void);

#endif /* _SFLIBHEAD_H_ */
