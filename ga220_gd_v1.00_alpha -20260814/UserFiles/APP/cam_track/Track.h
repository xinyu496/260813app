#ifndef __TRACK_H
#define __TRACK_H
#include "Common/base_inc.h"
#include "Common/config.h"

/*
跟踪器控制帧
接收到该帧，解析需要的方位数据，之后执行相应的动作。
*/
__packed typedef struct
{
    uint8_t Header1; 	/*[0]帧头*/
    uint8_t Header2;	/*[1]帧头*/
    uint16_t data_len;	/*[2-3]长度*/
    uint8_t ctrl_object;/*[4]控制对象*/
    uint8_t ctrl_cmd;/*[5]控制指令*/
    uint8_t zh_para1;/*[6]*/
    uint8_t zh_para2;/*[7]*/
    uint8_t zh_para3;/*[8]*/
    uint8_t zh_para4;//[9]载荷控制数据
    uint8_t	res0;	/*[10]预留*/
    int16_t FW_para1;	/*[11-12]*/
    int16_t FW_para2;	/*[13-14]*/
    int16_t FW_para3;	/*[15-16]*/
    int16_t FW_para4;	/*[17-18]*/
    int16_t FY_para1;	/*[19-20]*/
    int16_t FY_para2;	/*[21-22]*/
    int16_t FY_para3;	/*[23-24]*/
    int16_t FY_para4;	/*[25-26]*/
    int16_t HG_para1;	/*[27-28]*/
    int16_t HG_para2;	/*[29-30]*/
    int16_t HG_para3;	/*[31-32]*/
    int16_t HG_para4;	/*[33-34]*/
    int16_t in_gd_heading;//[35-36]内惯导航向角
    int16_t in_gd_pitch;  //[37-38]内惯导俯仰角
    int16_t in_gd_roll;   //[39-40]内惯导横滚角
    int16_t geo_heading;	//[41-42]地理跟踪航向角
    int16_t geo_pitch;		//[43-44]地理跟踪俯仰角
    int16_t geo_roll;		//[45-46]地理跟踪横滚角
	int16_t res1;  			//预留方位角
	int16_t res2;			//预留俯仰角
	uint8_t main_video;	//0x01-可见光；0x02-红外；0x3-可见光2；0x4-红外2；
	uint16_t vl_focus;
	uint16_t ir_focus;
	uint16_t vl_focus_2;	//[56-57]当前可见光2焦距
    uint16_t ir_focus_2; 	//[58-59]当前红外2焦距
	uint8_t track_sta; //当前跟踪状态
	uint8_t ele_zoom;  //电子变倍
	int16_t FW_miss;   //方位脱靶量
	int16_t FY_miss;	//俯仰脱靶量
	int32_t imu_x;
	int32_t imu_y;
	int32_t imu_z;
	int16_t cur_fy_data;
	uint8_t res3[2];//预留字节
    uint16_t add_check;
} TRACK_CTRL_DATA_T;

//跟踪器接收的状态帧
__packed typedef struct
{
    uint8_t Header1;   		/*[0] 0xA8*/
    uint8_t Header2;		/*[1] 0xA4*/
    uint16_t data_len;		/*[2-3]帧长度*/
    uint8_t sf_sys_sta;		/*[4]伺服系统状态 伺服模式*/	
    int16_t amuzith;		/*[5-6]方位角*/
    int16_t pitch;			/*[7-8]俯仰角*/
    int16_t roll;			/*[9-10]横滚角*/
    int16_t amuzith_speed;	/*[11-12]*/
    int16_t pitch_speed;	/*[13-14]*/
    int16_t roll_speed;		/*[15-16]*/
    int32_t amuzith_imu_speed; 	/*[17-20]方位陀螺速度*/
    int32_t pitch_imu_speed;	/*[21-24]俯仰陀螺速度*/
    int32_t roll_imu_speed;		/*[25-28]横滚陀螺速度*/
    uint8_t fw_self_check;		/*[29]方位自检状态*/
    uint8_t fy_self_check;		/*[30]俯仰自检状态*/
    uint8_t hg_self_check;		/*[31]横滚自检状态*/
    uint8_t gyro_sel_check;		/*[32]陀螺自检状态*/
    uint8_t res;				/*[33]版本号板卡标记*/
    VERSION_T sf_version;		/*[34-44]版本号*/
    uint8_t zero_find_sta;		/*[45]寻零状态*/
    uint8_t zero_find_gain;		/*[46]寻零增益*/
    int16_t amuzith_motor_a;/*[47-48]方位电机电流 0.05A/ bit*/
    int16_t pitch_motor_a;		/*[49-50]俯仰电机电流*/
    int16_t roll_motor_a;		/*[51-52]横滚电机电流*/
	IR_REPORT_INFO_T	ir_info;	/*[53-62]红外信息(预留)*/
	LASER_REPORT_INFO_T laser_info;	/*[63-69]激光信息(预留)*/
	int8_t  pod_temp;			/*[70]载荷舱温度*/
    uint8_t res1[8];		/*[71-78]*/
    uint16_t add_check;		/*[79-80]*/
} TRACK_TO_SF_STATUS_DATA_T;

/*跟踪器上报的状态帧*/
typedef struct
{
    uint8_t head1; // [0-1]
    uint8_t head2; // [0-1]
    uint16_t data_len;
    uint8_t track_sta;
    int16_t fw_miss;			// 3 水平脱靶量低位
    int16_t fy_miss;			// 5 高低脱靶量低位
    uint8_t track_selfcheck;			// 7 自检状态
    uint8_t storage;    // 8 剩余存储容量
    uint8_t recognize_sta;//  目标识别状态
    struct
    {
        uint8_t Port1 : 1;
        uint8_t Port2 : 1;
        uint8_t Dual : 1;
        uint8_t rev : 4;
        uint8_t LanMode : 1;
    } video_port_sta; //1- 端口输出开 0-端口输出关
    struct
    {
        uint8_t res2 : 7;
        uint8_t track_type : 1;
    } track_type;
    uint8_t video_biterate;// 14 当前视频压缩码率
    uint8_t vl_frames;		//可见原始输出帧数
    uint8_t ir_frames;		//红外原始输出帧数
    uint8_t wave_gate_w;	//当前跟踪波门宽
    uint8_t wave_gate_h;	//当前跟踪波门高
    int8_t vl_Fw_cross;				//可见方位十字丝偏移量
    int8_t vl_Fy_cross;				//可见高低十字丝偏移量
    int8_t ir_Fw_cross;				//红外方位十字丝偏移量
    int8_t ir_Fy_cross;				//红外高低十字丝偏移量
    int16_t eccentric_fw_offset;//偏心跟踪水平偏移量
    int16_t eccentric_fy_offset;//偏心跟踪竖直偏移量
    uint8_t ele_zoom;
    uint8_t stable_sta;//主视频稳像状态
    VERSION_T track_version;
    uint8_t res[50];//预留字节
    uint8_t sum;		   // 11 校验和 2~10字节累加和低8位
} CAM_VIDEO_RECV_DATA_T; // 接收视频跟踪器数据帧

typedef enum
{
    CAM_CROSSHAIR_FAST_TRACK = 0X95,/*十字中心快速跟踪*/
    CAM_PIXEL_FAST_TRACK = 0X9F,	/*像素位置快速跟踪*/
    CAM_AUTO_DOOR = 0X10,			/*自适应波门*/
    CAM_BIG_DOOR = 0X13,			/*大波门*/
    CAM_MID_DOOR = 0X14,			/*中波门*/
    CAM_SMALL_DOOR = 0X15,			/*小波门*/
    CAM_ADJUST_SLIGHT_UP = 0XC2,	/*波门微调上*/
    CAM_ADJUST_SLIGHT_DOWN = 0XC4,	/*波门微调下*/
    CAM_ADJUST_SLIGHT_LEFT = 0XC7,	/*波门微调左*/
    CAM_ADJUST_SLIGHT_REIGHT = 0XC8,/*波门微调右*/
    CAM_MAIN_CROSSHAIR_UP = 0XC3,	/*主视频十字丝上移*/
    CAM_MAIN_CROSSHAIR_DOWN = 0XC5,	/*主视频十字丝下移*/
    CAM_MAIN_CROSSHAIR_LEFT = 0XC9,	/*主视频十字丝左移*/
    CAM_MAIN_CROSSHAIR_REIGHT = 0XCA,/*主视频十字丝右移*/
    CAM_MAIN_CROSSHAIR_MID = 0XCB,	/*主视频十字丝归中*/
    CAM_MAIN_CROSSHAIR_SAVE = 0X8C,	/*主视频十字丝保存*/
    // 视频模式
    CAM_MODE_KJG_ONLY = 0X17,		/**/
    CAM_MODE_COMBINE_IR_Q = 0X1D,	/**/
    CAM_MODE_COMBINE_IR2X = 0X1E,	/**/
    CAM_MODE_COMBINE_KJG_MAIN = 0X1F,/**/
    CAM_MODE_COMBINE_KJ2X = 0X1B,	/**/
    CAM_MODE_IR_ONLY = 0X23,		/**/
    CAM_MODE_COMBINE_KJG_Q = 0X24,	/**/
    CAM_MODE_COMBINE_KJG2X = 0X25,	/**/
    CAM_MODE_COMBINE_IR_MAIN = 0X26,/**/
    CAM_MODE_COMBINE_IR_MAIN3X = 0X27,/**/

    CAM_OSD_CORLOR = 0X8,		/**/
} CAM_CMD_INFO_TYPE;


#define TRACK_MOVE_STEP  0x10
#define	TRACK_OFF_STEP 0x10

#if UNITY_TRACK

/*OSD状态帧*/
typedef struct
{
    uint8_t dataLen;   // [14] 总OSD数据长度
    uint8_t Devi_ID;   // [15] 设备id
    uint8_t VideoMode; // [16] 0 -- 主副模式 1-- 独立
    __packed struct
    {

        uint8_t Cross : 1;		  // 十字丝
        uint8_t TraGate : 1;	  // 跟踪框
        uint8_t IdenGate : 1;	  // 识别框
        uint8_t Fw_Ang : 1;		  // 方位角度
        uint8_t FY_Ang : 1;		  // 俯仰角度
        uint8_t sysmode : 1;	  // 系统工作模式
        uint8_t sensor_chose : 1; // 传感器选择
        uint8_t Time : 1;		  // 时间
        uint8_t MianView : 1;	  // 主视场角
        uint8_t VLView : 1;		  // 可见视场
        uint8_t IRView : 1;		  // 红外视场
        uint8_t PP : 1;			  // PP  当前载机位置
        uint8_t LP : 1;			  // LP
        uint8_t WP : 1;			  // WP
        uint8_t LazSta : 1;		  // 激光状态
        uint8_t LazCode : 1;	  // 激光编码
        uint8_t LazTemp : 1;	  // 激光温度
        uint8_t LMC : 1;		  // LMC 显示
        uint8_t Compass : 1;	  // 指北针显示
        uint8_t PTRA : 1;		  // 0-PTRA / 1-STRA显示
        uint8_t sensor_hzh : 1;	  // 画中画sensor
        uint8_t Zbx_show : 1;	  // 坐标系显示
        uint8_t SRDis : 1;		  // SR定义暂不知
        uint8_t MainDis : 1;	  // 暂不知
        uint8_t SubDis : 1;		  // 暂不知
        uint8_t MINS_VRTK_xy : 1; // 暂不知
        uint8_t Sz_pos_show : 1;  // 十字丝偏移显示
        uint8_t rev : 5;		  // 待填充
    } Ctrl;						  // [17-20] 32位，osd显示控制   0 -- 显示  1 -- 不显示
    uint8_t SysMode;			  // [21] 系统模式，具体含义待定
    int32_t EO_FWAngle;			  // [22-25] 方位角 分辨率0.01
    int16_t EO_FYAngle;			  // [26-27] 俯仰角 分辨率0.01
    __packed struct
    {
        uint8_t LMCState : 1; // LMC状态 1:开启LMC   0：关闭lmc
        uint8_t TrakMode : 1; // 备用
        uint8_t resv : 6;	  // 备用
    } dc_state;				  // [28] 吊舱的一些状态
    uint8_t Compass;		// [29] 指北针指向 0~22	0档指北，顺时针旋转，15°递增
    uint16_t MainViewRange; // [30-31] 主视频视场角 分辨率0.01
    uint16_t VLViewRange;	// [32-33] 可见光视场角 分辨率0.01
    uint16_t IRViewRange;	// [34-35] 红外视场角 分辨率0.01
    __packed struct
    {
        uint8_t enable : 1; // 激光使能
        uint8_t power : 1;	// 电源开启
        uint8_t resv : 6;
    } LazSta;		  // [36] 激光状态  0 - dis 1 -- enable
    uint8_t LazMode;  // [37] 激光工作模式，待定
    uint8_t LazTemp;  // [38] 激光器温度
    uint16_t LazDis;  // [39-40] 激光器距离
    uint16_t LazCode; // [41-42] 1~65535 激光编码
    __packed struct
    {
        uint8_t IM : 1; // 可见图像增强
        uint8_t resv : 1;
        uint8_t dzbb : 4; // 可见电子变倍
        uint8_t resv2 : 2;
    } VLMode; // [43] 可见光状态
    __packed struct
    {
        uint8_t IM : 1; // 图像增强
        uint8_t Color : 1;
        uint8_t ditBB : 4;
        uint8_t rev2 : 2;
    } IRMode; // [44] 可见光状态

    __packed struct
    {
        uint8_t Year;		  // UTC年 - 2000
        uint8_t Month;		  // UTC月
        uint8_t Day;		  // UTC日
        uint8_t Hour;		  // UTC时
        uint8_t Minute;		  // UTC分
        uint8_t Second;		  // UTC秒
        uint16_t Millisecond; // UTC毫秒低位
    } Time;					  // [45 - 52] 系统时间

    POSITION_INFO_T ZJ_Pos;		// [53-61] 载机当前位置
    POSITION_INFO_T ZJ_NextPos;	// [62-70] 载机下一点位置
    POSITION_INFO_T TarPos;		// [71-79] 目标点位置

    uint16_t SRDis;		//[80-81] 1m SR定义暂不知
    uint16_t MainDis_H; //[82-83] 1m 暂不知
    uint16_t MainDis_V; //[84-85] 1m 暂不知
    uint16_t SubDis_H;	//[86-87] 1m 暂不知
    uint16_t SubDis_V;	//[88-89] 1m 暂不知

    uint8_t rev[21]; // [90-110]
} CAM_VIDEO_OSDDATA_T;

//跟踪器接收的控制帧
typedef struct // 新协议
{
    // [2] 跟踪器相关指令码。
    uint8_t header1;
    uint8_t header2;
    uint16_t data_len;
    int16_t PositionCoor; // [3-4] 点选目标方位坐标。
    int16_t PitchCoor;	  // [5-6] 点选目标俯仰坐标。
    uint8_t Para;		  // [7] 参数   // 波门微调步长，编号跟踪的编号，十字丝挪动步长...
    uint8_t Video_Main;	  // 当前主视频
    uint8_t Ctrlrev[5];	  // [8-13] 备用 识别类型选择，根据实际情况定义

    /*OSD相关数据*/
    CAM_VIDEO_OSDDATA_T Osd;

    uint16_t chk_sum;	 // [111] 校验和  2~110 字节累加和低8位
} CAM_VIDEO_STATUS_DATA_T; // 新协议
#endif


#pragma pack()

/*跟踪器显示系统模式*/
typedef enum
{
    POD_STO = 0x13,	  /*收藏*/
    POD_TRCK = 0x15,  /*跟踪*/
    POD_LOCK = 0x16,  /*锁定*/
    POD_SCAN = 0x19,  /*扫描*/
    POD_MANU = 0x1A,  /*手动*/
    POD_STR = 0x1C,	  /*搜索跟踪*/
    POD_SPOT = 0x23,  /*小区搜索*/
    POD_SLA = 0x25,	  /*随动*/
    POD_RST = 0x26,	  /*随动*/
    POD_GEO = 0x1D,	  /*地理跟踪*/
    POD_GUIDE = 0x2A, /*漫游引导*/

    TRA_SHOW_MODE_END,
} TRA_SHOW_MODE_IN_TYPE_INFO;

/*跟踪器指令集*/
typedef enum
{
    TRACK_CMD_INVALID = 0x00,	// 无效指令
    TRACK_POINT_SEL = 0x9E,	 	// 跟踪点选择，先内部跟踪，等收到【跟踪确认】指令后，发出脱靶量实现闭环跟踪。
    TRACK_ENABLE = 0x92,	 	// 跟踪确认
    TRACK_IDENTIFY = 0x93,	 	// 识别跟踪，进入捕获状态，一旦识别到指定类别的目标以后，选择离与3~6字节代表的点最近的目标进行跟踪
    TRACK_WITH_CROSSHAIR = 0x95,// 十字中心快速跟踪
    TRACK_WITH_PIXEL = 0x9F, 	// 像素位置快速跟踪，不用等待【跟踪确认】指令，直接发出脱靶量实现闭环跟踪。
    TRACK_WITH_DICENTER = 0x91,	// 偏心跟踪
    TRACK_DISABLE = 0x45,	 	// 退出跟踪
    TRACK_WAVE_GATE_AUTO = 0x10,// 自动波门
    TRACK_WAVE_GATE_LAR = 0x13,	 // 大波门
    TRACK_WAVE_GATE_MID = 0x15,	 // 中波门
    TRACK_WAVE_GATE_SMA = 0x16,	 // 小波门
    TRACK_TARGET_DETECT_ON = 0xAC,  // 目标检测开，仅识别框、无编号，配合65字节
//	TRACK_TARGET_DETECT_OFF = 0xAD, // 目标检测关，LWB自行增加的指令，仅测试使用。
    TRACK_TARGET_RECOGNIZE_ON = 0xAD,  // 目标识别开，稳定编号，配合65字节
    TRACK_TARGET_RECOGNIZE_OFF = 0xAE, // 目标识别关
    TRACK_WITH_ID = 0xAF, // 编号跟踪，无特殊操作，只是将给定编号的识别目标的脱靶量按照20ms周期上报。
    TRACK_WAVE_GATE_UP = 0xC2,	   // 波门微调上
    TRACK_WAVE_GATE_DOWN = 0xC4,  // 波门微调下
    TRACK_WAVE_GATE_LEFT = 0xC7,  // 波门微调左
    TRACK_WAVE_GATE_RIGHT = 0xC8, // 波门微调右
    TRACK_CROSSHAIR_UP = 0xC3, // 主视频十字丝上移
    TRACK_CROSSHAIR_DOWN = 0xC5,	  // 主视频十字丝下移
    TRACK_CROSSHAIR_LEFT = 0xC9,	  // 主视频十字丝左移
    TRACK_CROSSHAIR_RIGHT = 0xCA,	// 主视频十字丝右移
    TRACK_CROSSHAIR_RESET = 0xCB,	  // 主视频十字丝归中
    TRACK_CROSSHAIR_SAVE = 0x8C,	  // 主视频十字丝存储

    TRACK_MODE_VL = 0x17,		 // 视频显示模式――可见光单显
    TRACK_MODE_VL_IR = 0x1D,	 // 视频显示模式――可见光主/原始红外全局子
    TRACK_MODE_VL_IR2X = 0x1E, // 视频显示模式――可见光主/原始红外2x子
    TRACK_MODE_VL_VL1X = 0x1F, // 视频显示模式――可见光主/可见光中心子
    TRACK_MODE_VL_VL2X = 0x1B, // 视频显示模式――可见光主/可见光中心2x变倍子
    TRACK_MODE_IR = 0x23,		 // 视频显示模式――红外单显
    TRACK_MODE_IR_VL = 0x24,	 // 视频显示模式――红外主/可见全局子
    TRACK_MODE_IR_VL2X = 0x25, // 视频显示模式――红外主/可见2x子
    TRACK_MODE_IR_IR1X = 0x26, // 视频显示模式――红外主/红外中心子
    TRACK_MODE_IR_IR2X = 0x27, // 视频显示模式――红外主/红外中心2x
    TRACK_VIDEO_ON = 0xAA,	// 录像开
    TRACK_VIDEO_OFF = 0xAB, // 录像关，默认
    TRACK_ALL_BLANKING = 0x5A, // 全消隐，视频上不叠加字符。
    TRACK_ALL_DISPLAY = 0x5B, // 全显示，视频上显示所有字符，默认。

    TRACK_SDI_OSD_OFF = 0xD0, // SDI视频OSD关，默认
    TRACK_SDI_OSD_ON = 0xD1,	 // SDI视频OSD开

    TRACK_ETH_OSD_OFF = 0xD2, // 网口视频OSD关
    TRACK_ETH_OSD_ON = 0xD3,	 // 网口视频OSD开，默认

    TRACK_ETH_VIDEOPORT1_ON = 0xF2,	 // UDP网口视频端口1开，端口号10301
    TRACK_ETH_VIDEOPORT2_ON = 0xF4,	 // UDP网口视频端口2开，端口号10302
    TRACK_ETH_VIDEO_OFF = 0xF3,	 // UDP网口视频接口两路同时输出模式关
    TRACK_RTSP_VIDEO_OFF = 0xF6, // RTSP网口视频接口关，默认
    TRACK_RTSP_VIDEO_ON = 0xF8,  // RTSP网口视频接口开

    TRACK_SDI_VIDEO_OFF = 0xD6,	 // SDI视频开，默认
    TRACK_SDI_VIDEO_ON = 0xD4, // SDI视频关

    TRACK_ELE_STABLE_ON = 0x8A,//电子稳像开
    TRACK_ELE_STABLE_OFF = 0x8B,//电子稳像关

    TRACK_TARGET_TEMPLATE_STAPLE = 0xD9,		// 目标模板装订，装订过程中跟踪器暂停其它操作，完成后恢复
    TRACK_TARGET_TEMPLATE_OUTPUT = 0xDC,	// 目标模板输出开，1）	仅在目标跟踪模式下有效，截取视场中心一定大小的图像并压缩输出；2）上述操作不影响正常跟踪。
    TRACK_TARGET_TEMPLATE_NO_OUTPUT = 0xE3, // 目标模板输出关

    TRACK_AREA_OF_INTEREST_10X = 0xA1, // 感兴趣区域――1倍模型尺寸
    TRACK_AREA_OF_INTEREST_15X = 0xA2, // 感兴趣区域――1.5倍模型尺寸
    TRACK_AREA_OF_INTEREST_20X = 0xA3, // 感兴趣区域――2倍模型尺寸
    TRACK_AREA_OF_INTEREST_FULL = 0xA4, // 感兴趣区域――全画幅(默认)
    TRACK_VERSION_REQ = 0x22, // 版本号查询
    TRACK_CODE_OUTPUT_MODE = 0xE8, // 压缩视频码率设置，码值参数位于64字节：分辨率：0.1M/bit，范围：0.5~6M
    TRACK_OSD_COLOR = 0x8,

    TRACK_CMD_INFO_END,
} TRACK_CTRL_CMD_IN_TYPE_INFO;

/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
void TRACK_Recv_Ctrl_Data_Process(uint8_t *data_in, uint8_t lengh);
/*==============================================================
 *FUNCTION NAME:
 *DISCRIPTION:
 *PARAMETERS:
 *RETURN:
 *N/A
 *NOTES:
 *HISTORY:
 *==============================================================*/
uint8_t TRACK_API_Ctrl_SendHandle(uint8_t send_type, uint8_t *send_data);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口接收数据初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TRACK_API_Serial_Data_Init(void);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t TRACK_API_Period_Handle(void);
#endif

