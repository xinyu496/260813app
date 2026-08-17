#ifndef __MASTER_CTRL_H
#define __MASTER_CTRL_H

#include "Common/base_inc.h"
#include "Common/config.h"
//接收主控数据帧
//与主控板通讯处理

#if MASTER_INCLUDE

#define ANS_STATEYES	0x03
#define ANS_STATENO		0x0C

#pragma pack(1)
//上位机下发控制帧
typedef struct{
	uint8_t header1;				/*0帧头1 0xAA*/
    uint8_t header2;   		    	/*1帧头2 0xA1*/
    uint16_t data_len;				/*2-3数据长度*/
    uint8_t target_device;			/*4目标设备*/
    uint8_t ctrl_object;     		/*5控制对象*/
    uint8_t ctrl_cmd;      			/*6控制指令*/
    int16_t param1;     			/*7-8参数1*/
    int16_t param2;     			/*9-10参数2*/
    int16_t param3;     			/*11-12参数3*/
    int16_t param4;     			/*13-14参数4*/
    int16_t param5;     			/*15-16参数5*/
    int16_t param6;     			/*17-18参数6*/
    int16_t slew_az;				/*19-20操纵杆方位值*/
    int16_t slew_el;				/*21-22操纵杆俯仰值*/
    int32_t target_jd;				/*23-26目标点经度 -180°~180° 东经为正 LSV=10-7°*/
    int32_t target_wd;				/*27-30目标点纬度 -90°~90° 北纬为正 LSV=10-7°*/
	int16_t	target_alt;			  	/*31-32目标海拔高*/
	uint16_t ir_res;			  	/*33-34红外分辨率*/
	uint16_t vis_res;				/*35-36可见光分辨率*/
	uint8_t res[12];				/*37-48预留12字节*/
	uint8_t cnt;					/*49帧计数*/
    uint16_t checksum;				/*50-51校验和 2-49字节求和*/
}MASTER_RCV_CTRL_CMD_T;
#define MASTER_RCV_CTRL_CMD_LEN  sizeof(MASTER_RCV_CTRL_CMD_T)

//常规应答帧
typedef struct
{
	uint8_t header1;				/*0帧头1 0xXX*/
    uint8_t header2;   		    /*1帧头2 0xAA*/
    uint16_t data_len;			/*2-3数据长度*/
    uint8_t ack_id;				/*4应答帧标识*/
    uint8_t sys_mode;     		/*5系统模式*/
    uint8_t ctrl_object;      	/*6控制对象*/
    uint8_t cmd_result;      	/*7指令执行结果*/	
	int16_t fw_angle;         	/*8-9方位角 分辨率0.01°/bit*/
	int16_t fy_angle;         	/*10-11俯仰角 分辨率0.01°/bit*/	
	int16_t rol_angle;        	/*12-13横滚角 分辨率0.01°/bit*/	
	int16_t fw_angle_speed;   	/*14-15方位角速度 分辨率0.01°/s/bit*/		
	int16_t fy_angle_speed;   	/*16-17俯仰角速度 分辨率0.01°/s/bit*/		
	int16_t rol_angle_speed;  	/*18-19横滚角速度 分辨率0.01°/s/bit*/	
	int16_t fw_offset;        	/*20-21方位偏移 分辨率0.01°/bit*/
	int16_t fy_offset;        	/*22-23俯仰偏移 分辨率0.01°/bit*/	
	int16_t rol_offset;       	/*24-25横滚偏移 分辨率0.01°/bit*/
	int16_t frame_fw;       	/*26-27框架方位角 分辨率0.01°/bit*/
	int16_t frame_fy;       	/*28-29框架俯仰角 分辨率0.01°/bit*/
	int16_t frame_rol;       	/*30-31框架横滚角 分辨率0.01°/bit*/	
	int16_t ext_ins_yaw;      	/*32-33外惯导航向角 分辨率0.01°/bit*/
	int16_t ext_ins_pitch;    	/*34-35外惯俯仰角 分辨率0.01°/bit*/
	int16_t ext_ins_rol;      	/*36-37外惯横滚角 分辨率0.01°/bit*/
	int16_t fw_calib;      		/*38-39方位校靶值 分辨率0.01°/bit*/
	int16_t fy_calib;      		/*40-41俯仰校靶值 分辨率0.01°/bit*/
	int16_t rol_calib;      	/*42-43横滚校靶值 分辨率0.01°/bit*/	
	int16_t hor_miss;      		/*44-45水平脱靶量 可见：±960 红外：±640*/		
	int16_t ver_miss;      		/*46-47高低脱靶量 可见：±540 红外：±512*/	
	__packed  struct
	 {
		uint32_t vis_self :1;   /*可见光自检结果*/
		uint32_t ir_self :1;	/*红外自检结果*/
		uint32_t vis2_self :1; 	/*可见光2自检结果*/
		uint32_t ir2_self :1;	/*红外2自检结果*/
		uint32_t vidio_self :1; /*视频处理模块自检结果*/
		uint32_t fwbmq_self :1;	/*方位编码器通信状态*/
		uint32_t fwtra_self :1;	/*方位驱动通信状态*/
		uint32_t fybmq_self :1;	/*俯仰编码器通信状态*/
		uint32_t fytra_self :1;	/*俯仰驱动通信状态*/
		uint32_t rolbmq_self :1; /*横滚编码器通信状态*/
		uint32_t roltra_self :1; /*横滚驱动通信状态*/
		uint32_t toptra_self :1;/*陀螺通信状态*/
		uint32_t fwsftra_self :1; /*方位伺服板通信状态*/
		uint32_t fysftra_self :1; /*俯仰伺服板通信状态*/		 
	 }selfcheck;            	/*48-51自检结果 0：故障 1：正常*/
	uint8_t track_src;        	/*52当前跟踪源*/
	uint8_t payload_nom;		/*53载荷序号*/
    uint16_t vis_fov;			/*54-55可见光视场角 分辨率：0.1度*/
    uint16_t ir_fov;			/*56-57红外视场角 分辨率：0.1度*/		
	__packed  struct
	 {
		 uint8_t vis_pow_sta:1;	//可见光上电状态 
		 uint8_t ir_pow_sta:1;	//红外上电状态 
		 uint8_t vis2_pow_sta:1;//可见光2上电状态 
		 uint8_t ir2_pow_sta:1;	//红外2上电状态 
		 uint8_t laser_pow_sta:2;//激光器上电状态 			 
	 }PowerStatus;           	 /*58部件上电状态 0x01:上电 0x00：下电*/
	__packed  struct
	 {
		 uint8_t track_vis_in_sta:2;//跟踪器可见光视频输入状态
		 uint8_t track_ir_in_sta:2;	//跟踪器红外视频输入状态
	 }vid_in_sta;             		/*59视频输入状态  0x01:输入 0x02：无输入 */
	__packed  struct
	{
		 uint8_t out_sta:2;		//输出状态 0x01:输出 0x02：不输出
		 uint8_t out_freq:2;	//输出频率帧  帧率： 1帧/bit
	}vis_out_sta;            	/*60可见光原始输出图像状态*/
	__packed  struct
	{
		 uint8_t out_sta:2;		//输出状态 0x01:输出 0x02：不输出
		 uint8_t out_freq:2;	//输出频率帧  帧率： 1帧/bit
	}ir_out_sta;            	/*61红外原始输出图像状态*/		
	uint16_t vis_focus;      	/*62-63可见光焦距 分辨率:0.1mm*/
	uint16_t ir_focus;       	/*64-65红外焦距 分辨率:0.1mm*/	
	uint8_t compress_bit;    	/*66压缩码率 1-100 分辨率：0.1Mbps*/
	__packed  struct
	{
		 uint8_t group_ins:3;	//输出状态 0x00:数据无效 0x01:数据有效 0x02：纯惯导有效
		 uint8_t gnss:1;		//0:GNSS无效  1:GNSS有效
	}ext_ins_sta; 		       	/*67外部惯导状态  0xFF:未收到*/
	__packed  struct
	{
		 uint8_t state:1;		//0:惯性  1:组合
		 uint8_t alig:1;		//0:静态对准  1:动态对准
		 uint8_t stae1:1;		//0:初始对准  1:正在导航				
	}int_ins_sta; 		       	/*68内惯导状态 */	 
	int8_t payload_temp;     	/*69载荷舱温度*/
	uint8_t payload_humi;    	/*70载荷舱湿度*/	
	uint8_t gate_size;       	/*71波门大小*/
    int32_t target_jd;			/*72-75目标点经度 -180°~180° 东经为正 LSV=10-7°*/
    int32_t target_wd;			/*76-79目标点纬度 -90°~90° 北纬为正 LSV=10-7°*/		
	int16_t	target_high;		/*80-81目标高度 分辨率：LSB=0.5m*/	
	uint16_t target_dis;      	/*82-83目标距离 分辨率：1/bit 单位m*/
	__packed  struct
	{
		 uint8_t mode:6;//激光模式
		 uint8_t send_mode:1;//0:发送禁能  1:发送使能
		 uint8_t resv_mode:1;//0:接收禁能  1:接收使能			
	}laser_sta;		            /*84激光状态*/
	uint16_t laser_dis;       /*85-86激光测距值*/
	uint16_t laser_nom;       /*87-88激光编码*/
	uint8_t laser_temp;       /*89激光温度*/
	uint8_t ir_light;         /*90红外亮度*/
	uint8_t ir_contrast;      /*91红外对比度*/	
	uint8_t ir_head_temp;     /*92红外镜头*/
	uint8_t ir_enhance;       /*93红外图像增强 0-7档*/
	uint8_t vis_light;        /*94可见光亮度*/
	uint8_t vis_enhance;      /*95可见光增强*/
	uint8_t vis_satuation;    /*96可见光饱和度*/
	uint8_t vis_contrast;     /*97可见光对比度*/		
	uint8_t vis_defog;        /*98可见光透雾挡位*/
	__packed  struct
	{
		 uint8_t vis_zoom:4;	//可见光
		 uint8_t ir_zoom:4;		//红外光		
	}ele_zoom;				   	/*99电子变倍*/	
	uint8_t heat_state;        /*100加热状态*/
	uint8_t temp_compen_state; /*101温漂补偿状态*/
	//原预留字节
	int16_t	Target_A_Speed;	   /*102-103目标方位角速度*/
	int16_t	Target_E_Speed;		/*104-105目标俯仰角速度*/
	int16_t	Target_Speed;		/*106-107目标运动速度*/		
	int16_t	Target_Direct;		/*108-109目标运动方向*/				
	uint8_t res[5];           	/*110-113*/
    uint16_t checksum;				/*114-115校验和 校验前所有字节0-113的和*/
}MASTER_PERIOD_SEND_T;
#define MASTER_SEND_PERIOD_LEN  sizeof(MASTER_PERIOD_SEND_T)
#pragma pack()

/*版本上报帧*/
typedef struct
{
    uint8_t header_sender;      //0 发送方0xA1
    uint8_t header_recever;     //1 接收方0xAA
    uint16_t data_len;			//2-3 数据长度
    uint8_t ack_id;				//4 应答帧标识
    uint32_t hardware_id;       //5-8 硬件编号
    uint32_t nameplate_id;		//9-12 铭牌编号
    uint16_t soft_version_cnt;  //13-14 软件版本个数
    uint8_t mc_id;				//15 主控ID
	VERSION_T mc_ver;			//15 俯仰伺服版本号
    uint32_t mc_board_id;		//27-30 主控板卡号
	 uint8_t sf_id;				//31 伺服ID
    VERSION_T  sf_ver;			//32-34 伺服版本号
    uint32_t sf_board_id;		//43-46 伺服板卡号
    uint8_t fysf_id;			//31 俯仰伺服ID
    VERSION_T fysf_ver;			//32-34 俯仰伺服版本号
    uint32_t fysf_board_id;		//43-46 俯仰伺服板卡号
    uint8_t fwsfid;				//47 方位伺服ID
    VERSION_T fwsf_ver;			//48-50 方位伺服版本号
    uint32_t fwsf_board_id;		//59-62 方位伺服板卡号
    uint32_t tuoluoNo[3];		//63-74 陀螺编号1-3
    uint32_t zaiheNo[3];		//75-86 载荷编号1-3
    uint32_t bianmaqiNo[3];		//87-98 编码器编号1-3
    uint32_t qudongNo[3];		//99-110 驱动编号1-3
    uint16_t TV_version1;		//111-112 可见光版本
    uint16_t IR_version1;		//113-114 红外版本
    uint16_t Laser_version;	  	//115-116 激光版本
    uint16_t TV_version2;		//117-118 可见光2版本
    uint16_t IR_version2;		//119-120 红外2版本
    uint8_t res[2];				//121-122 预留
    uint16_t sum;				//123-124 校验和 0-122相加
}MASTER_VERSION_RSP_T;
#define MASTER_VERSION_RSP_LEN  sizeof(MASTER_VERSION_RSP_T)
//设备标定应答帧
typedef struct
{
    uint8_t header_sender;      //0 发送方0xA1
    uint8_t header_recever;     //1 接收方0xAA
    uint16_t data_len;			//2-3 数据长度
    uint16_t ack_id;			//4-5 应答帧标识
    int16_t big_arm_x;			//6-7 大杆臂X
    int16_t big_arm_y;			//8-9 大杆臂y
    int16_t big_arm_z;			//10-11 大杆臂Z
    int16_t small_arm_x;		//12-13 小杆臂X
    int16_t small_arm_y;		//14-15 小杆臂y
    int16_t small_arm_z;		//16-17 小杆臂Z
    int16_t fw_encoder_zero;	//18-19 方位编码器零位
    int16_t fy_encoder_zero;	//20-21 俯仰编码器零位
    int16_t roll_encoder_zero;	//22-23 滚筒编码器零位
    uint8_t adj_status;         //24 询零状态
    uint32_t sf_gain;			//25-28 伺服增益
    int16_t fw_calib_angle;     //29-30方位安装标定角度
    int16_t fy_calib_angle;     //31-32俯仰安装标定角度
    int16_t roll_calib_angle;   //33-34滚筒安装标定角度
    uint8_t res[3];							//35-37预留
    uint16_t sum;								//38-39校验和 0到37累加
}MASTER_CALIBRATION_RSP_T;
#define MASTER_CALIBRATION_RSP_LEN  sizeof(MASTER_CALIBRATION_RSP_T)
#pragma pack()

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:常规通信应答帧
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MASTER_Normal_Rsp_Send(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:版本应答帧
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MASTER_Version_Rsp_Send(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:标定应答帧
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MASTER_Calibration_Rsp_Send(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MASTER_API_Period_Handle(void);
#endif
#endif
