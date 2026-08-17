#ifndef __CONFIG_H
#define __CONFIG_H
#include "Common/base_inc.h"
#include "Bsp/bsp_uart.h"
/***********************************全局参数***************************************/

//相机自身参数

#if (IR_CTRL_INCLUDE&IR_S640A_C1)
#define IRPixel		15.0
#define IRPNum_H	640
#define IRPNum_V	512
#define IRMaxZoom   550
#define IRMinZoom   45
#define IRMinRange	1
#define IRMaxRange	18
#endif

#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
#define IRPixel		15.0
#define IRPNum_H	640
#define IRPNum_V	512
#define IRMaxZoom   550
#define IRMinZoom   45
#define IRMinRange	1
#define IRMaxRange	18
#endif


//视场默认设置
#define VLSetLarView  (51.0f)
#define VLSetMidView  (30.0f)
#define VLSetSmaView  (8)
#define VLSetMinView  (3.2f)

#define IRSetLarView  (12.17f)
#define IRSetMidView  (10)
#define IRSetSmaView  (3)
#define IRSetMinView  (1)
//当前主视频
#define MainVideo_VL 1
#define MainVideo_MWIR 2
#define MainVideo_LWIR 3

//初始化数据函数
#define CHECK_RESET(member,type)\
	if(member == ((type)0xFFFF))member=0
//函数取极限值
#define CHECK_MAX(member,type)\
	if(member == ((type)0xFFFF))member=0xff

/*红外可见光大中小视场*/
#define CMD_ERR 0x1
#define PARA_ERR 0x2
#define CMD_SUCESS 0x0

#define CMD_ENABLE 0x1
#define CMD_DISABLE 0x2
//版本号结构体	
__packed typedef struct
{
    uint8_t h_version;
    uint8_t m_version;
    uint8_t s_version;
} VersionInfo_T;

__packed typedef struct
{
	VersionInfo_T version;
	uint32_t code_time_ymd;//编译时间年月日
	uint32_t code_time_hms;//编译时间时分秒
}VERSION_T;

/*================串口参数配置=========================*/

typedef struct
{
    uint8_t component_type;
    uint8_t com_type_in;//COM_TYPE_E
} COMPONENT_TYEP_MAP_T;

extern COMPONENT_TYEP_MAP_T component_map[];

/*===============网口参数配置======================*/

/*===============系统时间配置======================*/
typedef struct
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} SYS_TIME_T;

/*=========================外惯导数据=============================*/
#if ZH_PLANE_GD
typedef struct
{
    uint8_t data_len;
    uint16_t week;
    double week_sec;
    float yaw;	//偏航角--方位角
    float pitch;//俯仰
    float roll;	//横滚角
    float speed_east;
    float speed_north;
    float speed_sky;
    double longtitude;
    double latitude;
    float alt;
//    float gyro_speed_x;
//    float gyro_speed_y;
//    float gyro_speed_z;
//    float gyro_acc_x;
//    float gyro_acc_y;
//    float gyro_acc_z;
    uint8_t main_sat;
    uint8_t sec_sat;
    uint8_t work_sta;
    struct
    {
        uint16_t ref_sta:2;
        uint16_t position_valid:1;
        uint16_t ori_valid:1;
        uint16_t gps_data:1;
    } gnss_sta;
    /*
    Bit2-Bit0：
    =0：无效
    =1：单点定位
    =2：伪距差分
    =3：RTK差分定位
    Bit3：位置速度数据有效性 0：无效
    Bit4：GNSS双天线航向有效性 0：无效
    Bit5：GPS时数据有效 0：无效
    Bit6-Bit15：预留为0
    */
//    struct
//    {
//        uint16_t imu_x_err:1;
//        uint16_t imu_y_err:1;
//        uint16_t imu_z_err:1;
//        uint16_t acc_x_err:1;
//        uint16_t acc_y_err:1;
//        uint16_t acc_z_err:1;
//        uint16_t gnss_hw:1;
//    } sys_err;
//    /*
//    Bit0：X轴陀螺故障字 0：正常
//    Bit1：Y轴陀螺故障字 0：正常
//    Bit2：Z轴陀螺故障字 0：正常
//    Bit3：X轴加速度故障字 0：正常
//    Bit4：Y轴加速度故障字 0：正常
//    Bit5：Z轴加速度故障字 0：正常
//    Bit6：GNSS板卡硬件故障字0：正常
//    Bit7-Bit15：预留为0
//    */
    /*计算得到*/
//    uint16_t year;
//    uint8_t month;
//    uint8_t day;
//    uint8_t hour;
//    uint8_t min;
//    uint8_t second;
//    uint16_t msecond;
} EXT_GD_DATA_T;
#endif

#if JV_MINS_3C
typedef struct
{
    uint8_t datasta;    //数据有效性
    double longtitude;  //FJ经度
    double latitude;    //FJ维度
    double alt;         //FJ海拔高度
    double yaw;     	//FJ航向角
    double pitch;       //FJ俯仰角
    double roll;    	//FJ横滚角 
    double speed_east;  //FJ东向速度
    double speed_north;  //FJ北向速度
    double speed_sky;    // FJ天向速度
//    float gyro_speed_x;    //31~32 机体X轴角速度
//    float gyro_speed_y;    //33~34 机体Y轴角速度
//    float gyro_speed_z;    //35~36 机体Z轴角速度
//    float gyro_acc_x;       //37~38 机体X轴加速度
//    float gyro_acc_y;       //39~40 机体Y轴加速度
//    float gyro_acc_z;       //41~42 机体Z轴加速度
//    uint16_t year;               //43~44 年
//    uint8_t month;               //45    月
//    uint8_t day;                 //46    日
//    uint8_t hour;                //47    时
//    uint8_t min;                 //48    分
//    uint8_t second;              //49    秒
//    uint16_t msecond;            //50~51 毫秒
    double gnss_longitude;      //52~55 GNSS经度
    double gnss_latitude;       //56~59 GNSS纬度
    double gnss_height;         //60~63 GNSS高度
    double gnss_spped_east;     //64~65 GNSS东向速度
    double gnss_speed_north;    //66~67 GNSS北向速度
    double gnss_speed_sky;      //68~69 GNSS天向速度
    double gnss_datasta;        //70 GNSS数据有效状态
    uint8_t NumberofFJ[8];       //71~78 飞机编号
} EXT_GD_DATA_T;
#endif

#if FK_GD
typedef struct
{
    double yaw;
    double pitch;
    double roll;
    double speed_east;
    double speed_north;
    double speed_sky;
    double latitude;
    double longtitude;
    double alt;
//    float gyro_speed_x;
//    float gyro_speed_y;
//    float gyro_speed_z;
//    float gyro_acc_x;
//    float gyro_acc_y;
//    float gyro_acc_z;
    uint8_t work_sta;
    /*GNSS状态字*/
    struct
    {
        uint16_t ref_sta:2;
        uint16_t position_valid:1;
        uint16_t ori_valid:1;
        uint16_t gps_data:1;
    } gnss_sta;
    /*
    Bit2-Bit0：
    =0：无效
    =1：单点定位
    =2：伪距差分
    =3：RTK差分定位
    Bit3：位置速度数据有效性 0：无效
    Bit4：GNSS双天线航向有效性 0：无效
    Bit5：GPS时数据有效 0：无效
    Bit6-Bit15：预留为0
    */
    /*故障状态字*/
//    struct
//    {
//        uint16_t imu_x_err:1;
//        uint16_t imu_y_err:1;
//        uint16_t imu_z_err:1;
//        uint16_t acc_x_err:1;
//        uint16_t acc_y_err:1;
//        uint16_t acc_z_err:1;
//        uint16_t gnss_hw:1;
//    } sys_err;
    /*
    Bit0：X轴陀螺故障字 0：正常
    Bit1：Y轴陀螺故障字 0：正常
    Bit2：Z轴陀螺故障字 0：正常
    Bit3：X轴加速度故障字 0：正常
    Bit4：Y轴加速度故障字 0：正常
    Bit5：Z轴加速度故障字 0：正常
    Bit6：GNSS板卡硬件故障字0：正常
    Bit7-Bit15：预留为0
    */
} EXT_GD_DATA_T;
#endif


/*=========================外惯导数据=============================*/

/*=========================内惯导数据=============================*/
#pragma pack(1)
typedef struct
{
//    uint8_t produc_logo;
    float pitch;
    float roll;
    float yaw;
    float Temperature;
    double longtitude;
    double latitude;
    double alt;
//    uint32_t send_cnt;
//    uint32_t product_num;
//    uint8_t main_version;
//    uint8_t sec_version;
    struct
    {
        uint8_t ref_sta:1;
        uint8_t gnss_heading:1;
        uint8_t gnss_cog:1;
        uint8_t DH_sta:1;
        uint8_t acc:1;
        uint8_t augular:1;
        uint8_t dif_sta:1;
        uint8_t gesture_sta:1;
    } Sys_sta; /*第0位：0-初始对准，1-正在导航
					 第1位：GNSS航向角，0-无效，1有效
					 第2位：GNSS航迹角，0-无效，1-有效
					 第3位：导航模式，0-惯性，1-组合
					 第4位：加速度：0正常；1-超限【1】
					 第5位：角速度：0-正常；1-超限【2】
					 第6位：差分状态，0-无效，1-有效
					 第7位：姿态：0-正常；1-超限【3】
					*/
    uint8_t Aim_Sta;//1 ? 静态对准（地面对准; 2 ? 动态对准（空中对准）
} INSIDE_GD_DATA_T;
#pragma pack()
/*========================内惯导数据==============================*/

/*========================激光数据==============================*/
/*激光测照和激光照设时间*/
#define LASER_DETECT_TIME  (60)
#define LASER_LIGHT_TIME   (60)
/*故障告警*/
typedef struct
{
	uint8_t power_u5v:1;		//1：5v故障
	uint8_t apd_err:1;			//1：APD故障
	uint8_t laser_system:1;	//1：系统异常
	uint8_t overheating:1;	//1：超温告警
	uint8_t temperature_err:1;//1:温度异常
	uint8_t pd_err:1;		 		//1:PD异常
	uint8_t light_enable:1;	//1:禁止出光，0：允许
	uint8_t overhang:1;				/*0:超距,1:正常*/
} SYS_LASER_ALARM_T;

/*激光目标状态*/
typedef struct
{
	uint8_t static_status:1;//1：静态
	uint8_t dest_type:1; 	//1：末目标
	uint8_t apd_lock:1; 	//1：APD闭锁
	uint8_t last_dest:1; 	//1：有后目标
	uint8_t echo_sta:1;		//1：有回波
	uint8_t first_dest:1;	//1：有前目标
	uint8_t main_lobe:1;	//1：有主波
	uint8_t light_timeout:1;//1：照射超时
} SYS_LASER_DETECT_T;
typedef enum
{
	LASER_RCV_CONNECT_ERR,/*串口通信异常*/
	LASER_RCV_CHECK_ERR,/*校验错误*/
}LASER_RCV_ABNORMAL;

//激光模式
typedef enum
{
    LASER_WORK_SINGLE,/*单次测距*/
	LASER_STA_1HZ,/*连续测距（默认1Hz）*/
    LASER_WORK_5HZ,
	LASER_WORK_10HZ,
	LASER_WORK_20HZ,
	LASER_WORK_MARK,
	LASER_WORK_MULTI,/*多目标-这里指3目标*/
    LASER_WORK_IRRA,//测照
	LASER_WORK_CONTINUOUS, //连续测距 不定频率
    LASER_WORK_STAY = 0x0F,//测照/测距停
} CONFIG_LASER_STA;

typedef struct
{
	float distance[3];	/*目标距离m:
									distance[0]:第一目标距离；（默认使用distance[0]）
									distance[1]:第二目标距离；
									distance[2]:第三目标距离*/
	CONFIG_LASER_STA sys_sta;	/*激光工作状态*/
	
    uint16_t codenum;				/*激光编码序号*/
    uint32_t light_total_cnt;		/*出光总次数*/
    SYS_LASER_ALARM_T laser_alarm;	/*故障告警*/
    SYS_LASER_DETECT_T laser_sta;	/*激光目标状态*/
    uint16_t range_gate_value;		/*选通值距离*/
    float light_delay;	/*照射延时*/
    int8_t temp;					/*环境温度*/
    int8_t ld_temp;				/*LD温度*/
    uint16_t apd_u;				/*APD电压*/
    uint8_t ld_pulse;			/*ld驱动脉宽*/
    uint16_t light_cnt;		/*出光次数*/
    uint8_t current_gear;	/*当前档位*/
    float laser_power;    /*激光能量*/
    uint8_t laser_ver[6];	/*激光版本号	
							===LRS_0610A===
							D0-D2, 主控单片机
							D0:(uint8)主版本号（1-9，L\F\Y阶段为1，D阶段为2，P阶段为3）
							D1:(uint8)次版本号（0-99，功能或性能发生变化时）
							D2:(uint8)维护版本号（0-99，维护性或勘误性变化时）
							D3-D5, FPGA
							D3:(uint8)主版本号
							D4:(uint8)次版本号
							D5:(uint8)维护版本号
							===LSP_0410====
							D0-D1:电路板编号
							D2-D3:软件版本号
							*/

    char LZ_Powersta;			/*激光上电状态 1-上电；0-下电*/
	float blind_area; 			/*激光盲区*/
	int16_t apd_temp;			/*APD温度*/
	uint8_t return_intensity;	/*回波强度*/
} SYS_LASER_STA_T;
/*========================激光数据==============================*/

/*========================红外参数==============================*/
typedef struct
{
    struct
    {
        uint8_t filter_sta:1;	  /*滤波开关（0关1开）*/
        uint8_t refrigeration:1;  /*制冷状态（0-未到温/1-到温）*/
        uint8_t video_output:1; /*数字视频输出（0-8bit/1-14bit）*/
        uint8_t auto_light_cmp:1;/*自动亮度对比度开关（0关1开）*/
        uint8_t image_change:2;/*图像翻转*/
        uint8_t black_white:1;/*极性（0白1黑）*/
        uint8_t crosshair_on:1;/*十字开关（0关1开）*/
    } ir_status;
    uint8_t light_level;	/*亮度（0~255）*/
    uint8_t contrast_level;	/*对比度（0~255）*/
    uint8_t elezoom;		/*电子变倍*/
    uint8_t DDE_level;	    /*增强参数（0~255）*/
    uint8_t itr_range;		//积分时间档位（0-3）
    uint16_t integration_time;//积分时间
    uint16_t crosshair_x;	//十字线X坐标
    uint16_t crosshair_y;	//十字线Y坐标
    uint32_t work_time;		//累计工作时间
    uint16_t startup_cnt;	//开机次数
    uint16_t detect_temp;	//探测器制冷温度
    uint16_t ambient_temp;	//18B20环境温度
    uint16_t grayscale;		//灰度均值
    uint16_t zoom_position;	//变倍值
    uint16_t ir_focus_value;//焦距值
    uint8_t lens_temp;		//镜头温度
    uint8_t image_enhance_sta; //图像增强（0-关，1-开）
    uint8_t auto_calibration;//自动校正设置（0-关，1-开）

    float IRRange_H;/*水平视场角-弧度*/
    float IRRange_V;/*垂直视场角-弧度*/
    uint8_t irview_Now;//当前红外档位(根据焦距值判断)
    uint8_t focus_step;//调焦步进
    uint8_t gain_step;//增益步进
    uint8_t zoom_step;//变倍步进
    uint8_t contrast_step;//对比度步进值
    uint8_t IR_PowerSta;//红外上电状态
	uint8_t dde_gears;//图像增强挡位
} SYS_IR_STA_T;
/*========================红外参数==============================*/

/*=========================可见光参数===========================*/
#if (VISIBLE_INCLUDE & VL_F15_300)
#define VL_ZOOM_STEP_LEN 0x24//0x20（低）~ 0x27（快）
#define VL_FOCUS_STEP_LEN 0x34//0x30（低）~ 0x37（快）
#define VL_GAIN_STEP_LEN 8//0-240
#else
#define VL_ZOOM_STEP_LEN 20//10~255
#define VL_FOCUS_STEP_LEN 16//10~255
#define VL_GAIN_STEP_LEN 8//0-240
#endif

typedef struct
{
    uint16_t focus_value; //焦距值
    uint16_t zoom_value;   //变倍值
    uint8_t ele_zoom;     //电子变倍
    uint8_t visible_fps;  //帧频
    uint8_t light_degree;/*亮度（0~255）*/
    uint8_t gain_degree;/*增益（0~240）*/
    uint8_t contrast_degree;/*对比度（0~3）*/
    uint8_t view_ch;//当前视场（1-5）
    uint8_t core_err;//机芯故障
    uint8_t enhance_degree;//增强等级（0-3）
	uint8_t saturation;//饱和度
	uint8_t defog;//透雾档位
	uint8_t shar_degree;//锐度等级
	uint8_t exposure_degree;//曝光补偿等级
    struct
    {
        uint8_t res:1;
        uint8_t core_err:1;
        uint8_t res1:6;
    } err_alarm;

    struct
    {
        uint8_t work_mode:1;	//工作模式	0:正常 1:低功耗
        uint8_t ccd_sta:2;		//变焦CCD状态 0:正常 1:正在自检 3:故障
        uint8_t vis_color:1;	//变焦电视颜色	0：彩色 1：黑白
        uint8_t light_circle:1;	//光圈（保留） 0:手动 1:自动
        uint8_t focus_sta:1;	//调焦	0:手动 1:自动
        uint8_t light_sta:1;	//亮度	0:手动 1:自动
        uint8_t shutter_sta:1;	//快门(曝光)	0:手动 1:自动
    } vis_sta_1;

    struct
    {
        uint8_t gain_sta:1;		//增益 		0:手动 1:自动
        uint8_t contrast_sta:1;	//对比度	0:手动 1:自动
        uint8_t white_balance:1;//白平衡 	0:关  1:开
        uint8_t filt_lidht:1;	//滤光片	0:关  1:开
        uint8_t vis_hlc:1;		//逆光补偿	0:关  1:开
        uint8_t vis_blc:1;		//强光抑制	0:关  1:开
    } vis_sta_2;

    struct
    {
        uint8_t res2:2;
        uint8_t long_focus:1;//长焦限位（最小） 0:不限位1:限位
        uint8_t short_Limit:1;//短焦限位（最大）
        uint8_t focus_far:1;//聚焦远限位
        uint8_t focus_near:1;//聚焦近限位
        uint8_t res3:2;
    } focus_limit;

    float VLRange_H ;/*水平视场角-弧度*/
    float VLRange_V;/*垂直视场角-弧度*/
    uint8_t vlview_Now;//当前可见光视场挡位
    uint8_t focus_step;//调焦步进
    uint8_t gain_step;//增益步进
    uint8_t zoom_step;//变倍步进
} SYS_VISIBLE_DATA_T;
/*=========================可见光参数===========================*/

/*=========================伺服数据=============================*/

typedef struct
{
    uint8_t ext_data:4;
    uint8_t encoder_connect:1;//编码器通信状态
    uint8_t encoder_data_sta:1;
    uint8_t drv_connect:1;//驱动器通信状态
    uint8_t drv_function:1;
} SF_SELFCHECK_T;

typedef union
{
	uint8_t u8data;
	SF_SELFCHECK_T tdata;
}SF_SELFCHECK_union;

typedef struct
{
    uint8_t sf_sta;		//伺服状态
    float FW_Combine;	//方位角（合成-编码器）
    float FY_Combine;	//俯仰角（合成-编码器）
	float HG_Combine;	//横滚角（合成-编码器）
    float WFW_Angle;	//外方位角
    float WFY_Angle;	//外俯仰角
	float WHG_Angle;	//外横滚角
    float NFW_Angle;	//内方位角
    float NFY_Angle;	//内俯仰角
	float NHG_Angle;	//外横滚角
    float FW_Speed_a;	//方位轴加速度
    float FY_Speed_a;	//俯仰轴加速度
	float HG_Speed_a;	//俯仰轴加速度
    float amuzith_gyro;	//方位陀螺速度
	float pitch_gyro;	//俯仰陀                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        螺速度
	float roll_gyro;	//横滚陀螺速度
    SF_SELFCHECK_union check_WFW;//外方位自检状态；若无内外之分，则直接代表方位
    SF_SELFCHECK_union check_WFY;//外俯仰自检状态；若无内外之分，则直接代表俯仰
    SF_SELFCHECK_union check_NFW;//内方位自检状态
    SF_SELFCHECK_union check_NFY;//内俯仰自检状态
	struct 
	{
		uint8_t gyrox_connect:1;
		uint8_t gyroy_connect:1;
		uint8_t gyroz_connect:1;
		uint8_t res:1;
		uint8_t gyrox_data:1;
		uint8_t gyroy_data:1;
		uint8_t gyroz_data:1;
		uint8_t rev:1;
	}imuself_check; /*陀螺自检状态
						Bit0：陀螺x通信状态
						Bit1：陀螺y通信状态
						Bit2：陀螺z通信状态
						Bit4：陀螺x数据状态
						Bit5：陀螺y数据状态
						Bit6：陀螺z数据状态
						Bit3、bit7：预留
						0：正常；1：异常*/

	float fw_motor_i;	//方位电机电流
	float fy_motor_i;	//俯仰电机电流
	float hg_motor_i;	//横滚电机电流
} SYS_SF_DATA_T;
/*=========================伺服数据=============================*/



/*=========================视频跟踪器数据=========================*/
/*跟踪器跟踪状态*/
typedef enum
{
    TRA_NO = 0,
    TRA_NOW = 1,
    TRA_MEM = 2,
    TRA_MISS = 3,
    TRASTA_END,
} TRACK_TRASTA_E;


/*跟踪器视频模式*/
typedef enum
{
    TRA_MODE_NULL = 0,
    TRA_MODE_VL = 1,// 视频显示模式――可见光单显
    TRA_MODE_VL_IR = 2, // 视频显示模式――可见光主/原始红外全局子
    TRA_MODE_VL_IR2X = 3,	// 视频显示模式――可见光主/原始红外2x子
    TRA_MODE_VL_VL1X = 4, // 视频显示模式――可见光主/可见光中心子
    TRA_MODE_VL_VL2X = 5, // 视频显示模式――可见光主/可见光中心2x变倍子
    TRA_MODE_IR = 9,  // 视频显示模式――红外单显
    TRA_MODE_IR_VL = 10,	// 视频显示模式――红外主/可见全局子
    TRA_MODE_IR_VL2X = 11,	 // 视频显示模式――红外主/可见2x子
    TRA_MODE_IR_IR1X = 12, // 视频显示模式――红外主/红外中心子
    TRA_MODE_IR_IR2X = 13, // 视频显示模式――红外主/红外中心2x子
} TRACK_VIDEO_MODE_ENUM;
/*跟踪器状态*/
#pragma pack(1)
typedef struct
{
    uint8_t track_sta;		// 2 跟踪器工作状态 000：无操作;001：正在跟踪;   010：记忆跟踪;011：目标丢失
    int16_t fw_miss; 		// 3 水平脱靶量
    int16_t fy_miss;        // 5 高低脱靶量
}TRACK_IMP_INFO_T;

typedef struct
{
	TRACK_IMP_INFO_T track_main_info;
    struct
    {
        uint8_t CAMInput:1;				//可见光视频输入 1：有
        uint8_t INFInput:1;				//红外光视频输入 1：有
        uint8_t SDIOutput:1;			//SDI视频输出        1：有
        uint8_t NetworkOutput:1;		//网口视频输出         1：有
        uint8_t StorageStatus:1;		//存储状态 0：未满1：已满
        uint8_t CompressionRatio:1;		//目标模版更新状态0：失败；1：成功
        uint8_t TransportProtocol:1;   	//传输协议0：UDP 1：RTSP
        uint8_t CommunicationStatus:1; 	//通讯状态0：正常1：异常
    } SelfChecking;        		   		// 7 自检状态
    uint8_t SurplusStorageCapacity;		// 8 剩余存储容量
    uint8_t MultiTrackSta;			//1--多目标识别开，0--多目标识别关
    int8_t  Fw_cross;     		 	//10 方位十字丝偏移量
    int8_t  Fy_cross;     		 	//11 高低十字丝偏移量
    struct
    {
        uint8_t port1:1;
        uint8_t port2:1;
        uint8_t video_src_sta:1;
        uint8_t res1:4;
        uint8_t data_mode:1;
    } dest_port;		//12跟踪器端口以及状态
    uint8_t VideoBiterate; //14 当前视频压缩码率
    uint8_t VL_Frame;      //15 可见原始图像每秒输出帧数
    uint8_t IR_Frame;      //16 红外原始图像每秒输出帧数
    int8_t Temp;           //17温度
    uint8_t GZBM_Width;		//波门宽度
    uint8_t GZBM_Height;	//波门高度

    uint8_t recognize_sta;//识别状态
    uint8_t wavegate_sta;//波们状态
    uint8_t track_id;//编号跟踪的编号
    uint8_t video_color;//EO视频字符颜色/显示单位切换（TXTP）
    char SysTrack_mode;	/*跟踪模式 0 - 中心点跟踪 1 - 像素/搜索跟踪*/
} SYS_TRACK_DATA_T;
#pragma pack()
/*=========================视频跟踪器数据===========================*/
/*=========================外惯导数据========================*/
typedef struct
{
    struct
    {
        uint8_t dev_valid:4;//地面维护设备有效性
        uint8_t wheel_sta:4;//轮载状态
    } fcu_status;
    struct
    {
        uint8_t rev1:1;
        uint8_t time:1;//时间有效状态
        uint8_t rev2:1;
        uint8_t rev3:1;
        uint8_t fwfy:1;//方位俯仰有效状态
        uint8_t dis:1;//目标距离有效状态
        uint8_t alt:1;//目标高度有效状态
        uint8_t postion:1;//经纬度有效状态
    } data_valid; //1-有效；0-无效
    uint8_t destnation;	//目标源
    struct
    {

        int16_t slant;
        float FW;
        float FY;
    } target; //目标信息
	SYS_TIME_T exgd_time;
    uint8_t systime_s;
    uint16_t systime_ms;
    uint8_t local_id1;//本机地址1
    uint8_t local_id2;//本机地址2
} SYS_FCU_STATUS;
/*=========================外惯导数据========================*/
/*========================通用参数管理===========================*/
typedef struct
{
    uint32_t dest_ip;
    uint16_t dest_port;
    uint16_t s_port;
} UDP_CONFIG_T;
typedef struct
{
    struct udp_pcb *udp_server;
    UDP_CONFIG_T cli_udp_cfg;
} UDP_PCB_T;

typedef struct
{
    uint32_t ip;
    uint32_t mask;
    uint32_t gw;
} SYS_LOCAL_IP_T;

typedef enum
{
    RECV_DATA_SUC = 0,
    RECV_DATA_NULL = 1,
    RECV_DATA_HEAD_ERR = 2,
    RECV_DATA_LEN_ERR = 3,
    RECV_DATA_XOR_ERR = 4,
    RECV_DATA_OTHERR_ERR = 5,
    RECV_DATA_ERR_END,
} RECV_DATA_ERR_STA;

/*系统模式
0x00 -N/A
0x13-MAINT
0x15-IBIT
0x16-NORMAL
其余无效
*/
typedef enum
{
    EOMODE_N_A = 0x0,
    EOMODE_MAINT = 0x13,
    EOMODE_IBIT = 0x15,
    EOMODE_NORMAL = 0x16,

    MASTER_CMD_END,
} MASTER_WORK_MODE_INFO;


/*载荷信息上报结构体  红外*/
__packed typedef struct
{
	uint16_t	focal_length;	/*红外光焦距*/	
	uint16_t 	field_view;		/*红外视场角*/
	uint8_t		luminance;		/*红外亮度*/
	uint8_t		contrast;		/*红外对比度*/
	uint8_t		lens_temp;		/*红外镜头温度*/
	uint8_t		image_enh;		/*红外图像增强*/
	uint8_t		d_zoom;			/*电子变倍*/
	uint8_t		fault_warn;		/*故障告警*/
}IR_REPORT_INFO_T;

/*载荷信息上报结构体  红外*/
__packed typedef struct
{
	uint8_t		device_sta;		/*激光器状态*/
	uint16_t	distance;		/*激光测距值*/
	uint16_t	distance_src;	/*激光编码*/
	uint8_t		temperature;	/*激光温度*/
	uint8_t		fault_warn;		/*故障告警*/
}LASER_REPORT_INFO_T;


#pragma pack(1)
typedef struct
{
    int32_t geo_fy_adj; /*GEO调整量*/
    int32_t geo_fw_adj;
    int32_t geo_fw_zero; /*GEO零位修正 存储*/
    int32_t geo_fy_zero;
	uint32_t mech_fw_adj; /*机械零位调整量*/
	int32_t mech_fy_adj;
	uint32_t bmq_fw_zero;/*编码器初始零位*/
	int32_t bmq_fy_zero;
	
	int32_t arm_outx;// 
	int32_t arm_outy;//
	int32_t arm_outz; // 杆臂零位调整量,分辨率0.01°/bit    uint32_t mech_fw_adj;

	uint32_t card_num;//铭牌编号
	uint32_t spalt_num;//平台编号
	uint32_t device_num;//设备编号
	struct
	{
		uint8_t ir_shutter_time;// 快门定时校正时间;分钟
		uint8_t it_shutter_on;//
		uint16_t ir_resve;
	}ir_save_info;
	uint16_t crc;  // 用于读取后的校验
}FLASH_SAVE_INFO_T;//对设备操作 STM32FALSH_Servo_ADDR； ADDR_FLASH_SECTOR_10
#pragma pack()
//飞机数据有效性
typedef enum
{
    CTRL_STA_DATA_INVALID,
    CTRL_STA_DATA_VALID,
} DATA_VALIDITY_ENUM;

//视场角枚举
typedef enum
{
    LARGE_VIEW_FOCUS,
    MID_VIEW_FOCUS,
    SMA_VIEW_FOCUS,
    MIN_VIEW_FOCUS,
    VIEW_FOCUS_END,
} VIEW_FOCUS_T;

typedef enum
{
    ADJUST_NULL,
    ADJUST_UP,
    ADJUST_DOWN,
    ADJUST_RIGHT,
    ADJUST_LEFT,
	ADJUST_RES,//复位
} ANY_ADJUST_DIRECTION_ENUM;

typedef struct
{
    float azimuth_para;//方位参数-32000 ~ +32000
    float pitch_para;//俯仰参数-32000 ~ +32000
} ATTITUDE_PAPA_T;

typedef struct
{
	double longitude;
	double latitude;
	float alt;
}POSITION_INFO_T;
#if 0
typedef struct
{
	int32_t longitude;//LSB=10-7°
	int32_t latitude;//LSB=10-7°
	int16_t alt;//LSB=0.5m
}POSITION_INFO_T;
#endif
typedef struct
{
	uint8_t force_sta;//引导状态
	uint16_t force_difinition;//引导分辨率
}FORCE_GUIDE_INFO_T;

typedef struct
{
	int16_t fw_miss; 			// 3 水平脱靶量
    int16_t fy_miss;        	// 5 高低脱靶量
}TRACK_MISS_T;
/*=========================用户数据=============================*/
typedef struct
{
    /*
    系统参数
    注：根据使用需求修改
    */
    UDP_CONFIG_T udp_config_fk;		/*udp ip配置*/
    UDP_CONFIG_T udp_config_fiber;	/*udp ip配置*/
    UDP_CONFIG_T udp_config_master;	/*udp ip配置*/
    SYS_LOCAL_IP_T local_ip;
    SYS_TIME_T code_time;	 //编译时间
	SOFTWARE_ID_E board_type;//板卡类型
	FLASH_SAVE_INFO_T flash_save_info;	/*FLASH中存储的数据*/
//	VERSION_T software_version[MASTER_PARA_ENUM];/*接收所有软件的版本号*/
    /*
    外设参数
    注：来源于外设，不可修
    */
    EXT_GD_DATA_T fj_gd;			/*外惯导数据*/
    INSIDE_GD_DATA_T inner_gd;		/*内惯导数据*/
    SYS_LASER_STA_T laser_info;		/*激光测照器参数*/
    SYS_IR_STA_T ir_info;			/*红外参数*/
    SYS_VISIBLE_DATA_T vl_info;		/*可见光参数*/
    SYS_SF_DATA_T sf_info;			/*伺服参数*/
    SYS_TRACK_DATA_T track_info;	/*跟踪器参数*/
    SYS_FCU_STATUS plane_Data;		/*飞机输入数据（外惯导数据)*/
    /*
	用户变量
	注：根据实际使用需求增改
    */
	MASTER_CTRL_TYPE_ENUM  master_ctrl_sta; 	/*系统控制指令分配类型*/
    uint8_t master_ctrl_cmd[MASTER_PARA_END];	/*接收的控制指令*/
	uint8_t master_ctrl_buf[MASTER_PARA_END][8];/*接收的控制指令参数*/
    POSITION_INFO_T dst_position;				/*上位机下发的目标经纬高*/
	FORCE_GUIDE_INFO_T force_guide[2];
    /*
    用户变量
    注：根据实际使用需求增改
    */
    DATA_VALIDITY_ENUM fj_gd_valid;		/*外惯导有效性*/
    ATTITUDE_PAPA_T sf_attitude;		/*传给伺服的方位角和俯仰角*/
	
    MASTER_WORK_MODE_INFO sys_work_sta;	/*当前系统模式 自检、正常等*/
    SYS_CTRL_MODE_E sys_work_mode;  	/*系统指令控制枚举*/

	SF_MODE_E sf_mode;		/*当前伺服模式-以接收解算到的伺服模式为准*/
//    SF_SYS_MODE_ENUM aim_sf_mode;   /*当前控制伺服模式*/
	uint8_t sf_reach_sta;			//伺服到位判断
    bool LMCSta;					/*LMC状态 0-关 1-开*/
    bool tdc_sta;					/*(temperature drift compensation)温漂补偿状态*/
	uint8_t TargetLost;    			//目标识别态，但无识别计数
	uint8_t Mulit_On;      			/*目标识别1-开、0-关*/
	
    MAIN_VIDEO_INFO_E main_video;		/*当前主视频*/
    float MianView_H;				/*当前主视频水平视场角*/
    float MianView_V;				/*当前主视频垂直视场角*/
    bool view_match_sta;			/*视场匹配状态*/
	TRACK_MISS_T dev_miss;			/*脱靶量*/

	bool stable_sta;       			/*稳定状态*/
    bool Axis_sta;		   			/*在线校轴状态(跟踪器)*/
    char Traoffcenter_flg; 			/*偏心跟踪状态*/

    bool VFollowMove_sta;			/*单杆微调状态 VFollowMove_STEP(微调步长)*/

    bool GEOMove_sta;				/*随动微调状态*/
    float GEOMove_STEP; 			/*随动微调步长*/
    uint8_t adj_sta;				/*(ANY_ADJUST_DIRECTION_ENUM)微调状态*/

    /*定档位视场角对应的焦距值*/
    float vl_view_of_focus[VIEW_FOCUS_END];
    float ir_view_of_focus[VIEW_FOCUS_END];

} MASTER_CONFIG_INFO_T;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_CONFIG_INFO_T CONFIG_GET_SYS_Main_Info(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Printf_Dump(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Monitor_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Laser_Sta(SYS_LASER_STA_T laser_curr_sta);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_LASER_STA_T *CONFIG_Get_Laser_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Ext_position(EXT_GD_DATA_T position);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
EXT_GD_DATA_T CONFIG_Get_Ext_position(void);

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Internal_position(INSIDE_GD_DATA_T position);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
INSIDE_GD_DATA_T CONFIG_Get_Internal_position(void);
/*=============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Visible_Info(SYS_VISIBLE_DATA_T visi_info);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_VISIBLE_DATA_T CONFIG_Get_Visible_Info(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外信息获取
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_IR_STA_T* CONFIG_Get_Ir_Info(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Ir_Info(SYS_IR_STA_T* ir_info);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sf_Info(SYS_SF_DATA_T *sf_info);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_SF_DATA_T *CONFIG_Get_Sf_Info(void);

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_TRACK_DATA_T CONFIG_Get_Track_Info(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Info(SYS_TRACK_DATA_T cam_info);

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Plane_Info(SYS_FCU_STATUS plane_Data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_FCU_STATUS CONFIG_Get_Plane_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
//void CONFIG_Set_FKGD_Info(EXT_GD_DATA_T fj_gd);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
//EXT_GD_DATA_T CONFIG_Get_FKGD_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_LOCAL_IP_T CONFIG_Get_Current_ip(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
UDP_CONFIG_T CONFIG_Get_Current_Udpfk(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
UDP_CONFIG_T CONFIG_Get_Current_Udpfiber(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
UDP_CONFIG_T CONFIG_Get_Current_Udpmaster(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_TIME_T CONFIG_Get_Sys_Time(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Send_sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Send_sta(bool status);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SF_MODE_E CONFIG_Get_SF_Mode(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_SF_Mode(SF_MODE_E info);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS: 
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_TDC_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_TDC_Sta(bool tdc_sta);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_LMC_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_LMC_Sta(bool value);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Axis_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Axis_Sta(bool value);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_View_Match_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_View_Match_Sta(bool flag);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Master_Ctrl_Cmd(MASTER_PARA_ENUM object);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Master_Ctrl_Cmd(MASTER_PARA_ENUM object,uint8_t cmd);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
MASTER_CTRL_TYPE_ENUM CONFIG_Get_Master_Ctrl_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_TYPE_ENUM cmd_sta);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Decenter_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Decenter_Sta(bool value);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Main_Video(void);
void CONFIG_Set_Main_Video(uint8_t main_video);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
SYS_CTRL_MODE_E CONFIG_Get_Sys_Work_Mode(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sys_Work_Mode(SYS_CTRL_MODE_E work_mode);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Adjust_Sta(void);
void CONFIG_Set_Adjust_Sta(uint8_t value);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Plane_Data_Sta(void);
void CONFIG_Set_Plane_Data_Sta(DATA_VALIDITY_ENUM status);
void CONFIG_Set_Sf_Attitude(ATTITUDE_PAPA_T attitude);
ATTITUDE_PAPA_T CONFIG_Get_Sf_Attitude(void);
void CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_TYPE_ENUM set_type,uint8_t *value);
void CONFIG_Get_Master_Ctrl_Para(uint8_t ctrl_type,uint8_t *data_para);
void CONFIG_Get_Cali_vlFocus(float *focus);
void CONFIG_Get_Cali_irFocus(float *focus);
FLASH_SAVE_INFO_T CONFIG_Get_Geo_Zero(void);
void CONFIG_Set_Geo_Zero(FLASH_SAVE_INFO_T* geo_info);
void CONFIG_Net_ipif_Init(void);  
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Dst_Position(POSITION_INFO_T position);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
POSITION_INFO_T CONFIG_Get_Dst_Position(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Sf_Reach_Sta(uint8_t status);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Sf_Reach_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Stable_Sta(bool status);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool CONFIG_Get_Track_Stable_Sta(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取板卡类型
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t CONFIG_Get_Board_Type(void);
/*==============================================================*/

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
//void save_sys_userdata(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
//void load_sys_userdata(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
TRACK_MISS_T CONFIG_Get_Track_Miss(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Track_Miss(TRACK_MISS_T miss);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Set_Laser_Dist(const SYS_LASER_STA_T *laser_curr_dist);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CONFIG_Get_Laser_Dist(uint16_t laser_curr_dist);
#endif
