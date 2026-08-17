#ifndef __OPT_CMD_H
#define __OPT_CMD_H

#define	IRFOCUS_STEP 		2
#define	IRFZOOM_STEP 		2
#define	IRFOCUS_STEP_STEP 	1
#define	IRZOOM_STEP_STEP 	1

#define	IRLIGHT_RESET 		100
#define	IRCONTRST_RESET 	100
#define	IRDDE_RESET 		100

#define	IRDETIAL_RESET 		35
#define	IRLIGHT_STEP 		7
#define	IRCONTRST_STEP 		7
#define	IRDDE_STEP 			1
#define	IRDETIAL_STEP 		5

/*上位机下发的指令分类，各个状态位各占一位，可以同时开启。
同一个指令（上位机下发的），可能需要同时控制跟踪器和伺服*/
typedef enum
{
	MASTER_CTRL_NULL = 0x00,
	MASTER_CTRL_SYS = 0x01,
	MASTER_CTRL_TRACK = 0x02,
	MASTER_CTRL_SF = 0x04,
	MASTER_CTRL_VL = 0x08,
    MASTER_CTRL_IR = 0x10,
    MASTER_CTRL_LASER = 0x20,
   	MASTER_CTRL_VL_2 = 0x40,
    MASTER_CTRL_IR_2 = 0x80,
    MASTER_CTRL_END,
} MASTER_CTRL_TYPE_ENUM;

/**************************两个枚举一一对应***********************/
/*上位机控制指令参数
与MASTER_CTRL_TYPE_ENUM枚举一一对应，因此如有更改，请加在MASTER_PARA_END前*/
typedef enum
{
	MASTER_PARA_NULL,
    MASTER_SYS_PARA,
    MASTER_TRACK_PARA,
    MASTER_SF_PARA,
    MASTER_VL_PARA,
    MASTER_IR_PARA,
	MASTER_LASER_PARA,
	MASTER_FWSF_PARA,
	MASTER_FYSF_PARA,
    MASTER_PARA_END,
} MASTER_PARA_ENUM;

//当前主视频枚举
typedef enum
{
	MAIN_VIDEO_NULL,
	MAIN_VIDEO_VL_1,
	MAIN_VIDEO_IR_1,
	MAIN_VIDEO_VL_2,
	MAIN_VIDEO_IR_2,
}MAIN_VIDEO_INFO_E;

//伺服模式枚举
typedef enum
{
	SF_SELF_CHECK,	/*自检*/
	SF_BRAKE,		/*刹车*/	
	SF_WITH_DRAW,	/*收藏*/
	SF_ZEORO,		/*归零（自检）*/
	SF_MIDDLE,		/*归中*/
	SF_TRACK,		/*跟踪*/
	SF_LOCKED_BMQ,	/*锁定当前*/	
//	SF_LOCKED,		/*锁定*/		
//	SF_SCAN,		/*扫描*/		
	SF_SPEED_FOLLOW,/*速率（手动）*/		
	SF_TRACK_GEO,	/*地理跟踪(内惯导)*/		
	SF_DETECT_GEO, 	/*搜索跟踪（（图像跟踪））*/		
	SF_FW_SCAN,		/*方位扫描*/	
	SF_FY_SCAN,		/*俯仰扫描*/	
	SF_HG_SCAN,		/*横滚扫描*/	
	SF_TD_MODE,		/*零偏修正*/
	SF_DEBUG_MODE,	/*维修模式*/
	SF_TRACK_GEO_E,	/*地理跟踪(外惯导)*/		
	SF_GUIDE,		/*漫游引导*/	
	SF_TD_RESET,	/*漂移补偿自动/恢复出厂设置*/
}SF_MODE_E;

//系统控制指令枚举
typedef enum
{
	SYS_BREAK	= 0x41,  //刹车
	SYS_LOCK_CUR = 0x42, //锁定当前
    SYS_WITH_DRAW =0x43, //收藏
	SYS_ZERO	= 0x44, //归零（手动自检）
	SYS_SERCH   = 0x45, //搜索/跟踪
    SYS_TRACK	= 0x46, //跟踪/图像跟踪
	SYS_CENTER_TRACK = 0x47,
	SYS_TRACK_SERCH = 0x48,
	SYS_TRACK_GEO = 0x49,
	SYS_LOCK = 0x4A,//定角度锁定
    SYS_HANDLE	= 0x4B, //手动模式
    SYS_SCAN_FW	= 0x4C, //方位扫描
    SYS_SCAN_FY	= 0x4D, //俯仰扫描
	SYS_SCAN_HG	= 0x4E, //速率
	SYS_EXIT = 0x4F,//退出跟踪
	DEBUG_MODE,
	COMBYPASS_ENABLE = 0xC7,
	SYS_CMD_END,
}SYS_CTRL_MODE_E;//系统控制指令枚举
//红外控制指令
typedef enum
{
	/*参数1：cmd-指令码*/
	IR_SYS_STA_REQ = 0,	/*系统状态查询 兼容NX30的实时反馈功能（1-开；0-关）;
												参数1：1-版本查询；2-自检结果查询；3-温度查询；4-对比度查询；5-亮度查询
												用户根据需求增加查询枚举*/
	IR_BLACK_WHITE = 3,	/*黑白热 0-白，1-黑*/
	IR_ZOOM_CTRL = 4,	/*参数1：1-变倍加；2-变倍减；3-变倍停；4-变倍步进设置；5-变倍值设置；0x0F-变倍查询；（步进默认为1）； 
						参数2：参数1为4时，该值为步进值，参数1为5时，该值为变倍值；其余情况下默认为0；*/
	IR_FOCUS_CTRL,
	IR_FOCUS_AUTO,			/*自动聚焦*/
	IR_FOCUS_ONE_SET,		/*一键聚焦*/
	IR_FOCUS_COMPENSATE,	/*聚焦补偿 NX30*/
	IR_LIGHT_CTRL,      	/*参数1：1-亮度加；2-亮度减；3-亮度自动；4-亮度手动；5-亮度设置；0x0f-亮度读取；（步进默认为1）
							参数2：参数1为5时，该值为亮度值；
							注意：根据不同的外设，加减设置需要先切换到手动模式。*/
	IR_CONTRAST_CTRL,		/*对比度控制*/
	IR_GAIN_CTRL,			/*增益控制*/
	IR_SHUT_CTRL,			/*快门控制*/
	
	IR_CH_MLARGE = 0x11,	/*极大视场*/
	IR_CH_LARGE,			/*大视场*/
	IR_CH_MI,				/*中视场*/
	IR_CH_SMA,				/*小视场*/
	IR_CH_MIN,				/*极小视场*/
	IR_ITR_TIME_SET,		/*积分时间设置*/
	IR_ZOOM_DIT,         	/*电子变倍 1,2,4 100-800*/
	IR_VIEW_ANGLE,    		/*视场角设置*/
	
	IR_CFG_X_POSITION = 0x20,/*横坐标设置*/
	IR_CFG_Y_POSITION,		/*纵坐标设置*/
	IR_IMAGE_ENHANCE,		/*图像增强 0-关，1-开,2-加，3-减*/
	IR_CALIBRATION, 		/*图像校正 参数1：bit1-背景矫正；bit2-虚焦校正，bit3-档板校正，bit4-快门校正，bit5-自动矫正设置；
								（0-关；1-开）*/
	IR_DDE_RANGE,			/*DDE档位*/
	IR_CFG_CROSSHAIR_CTRL,/*参数1：1-十字开；2-十字关；
													参数2：1-十字上移；2-十字下移；3-十字左移；4-十字右移；5-十字复位*/
	IR_CFG_DEAD_PIXEL,		/*去盲元:参数1：0-自动；1-手动*/
	IR_CFG_FILTER,			/*滤波*/
	IR_DETAIL_XJH,			/*细节和发送*/
	IR_RENOISE,				/*降噪 data[0]:0-时域降噪，1-空域降噪 data[1]:0-off,1-on*/
	IR_SET_MIRROR,			/*镜像 0-off;1-垂直;2-水平；3-对角*/
	IR_TRACK_FOCUS,			/*跟焦 0-off;1-on*/
	IR_COLOR_DEEP,			/*色深 0-8bit,1-14bit,2-8/14bit,3-25bit,4-30bit*/
	IR_SLIGHTLY_ZOOM,		/*微动变倍*/
	IR_SLIGHTLY_FOCUS,		/*微动聚焦*/
	IR_SATURATION_CTRL,		/*饱和度控制*/
	IR_SHAR_CTRL,			/*锐度控制*/
#if (IR_CTRL_INCLUDE&IR_NX30_150)
	IR_SLIGHT_ZOOM,				/*参数1：1-变倍加；2-变倍减；*/
	IR_SLIGHT_FOCUS,			/*参数1：1-调焦加；2-调焦减；*/
#endif
	IR_SYS_RES = 0xFC,    /*系统复位*/
	
	IR_CMD_END,
	IR_IMAGE_TONE,			/*图像色调 0-暖色，1-冷色*/
}SYS_IR_CMD_CTRL_E;

//可见控制指令
typedef enum
{
	/*参数1：cmd-指令码*/
	VL_STATUS_ASK, 			/*系统状态查询 1-系统状态查询，2-焦距查询，3-变倍查询*/
	VL_ZOOM_CTRL = 4,		/*参数1：1-变倍加；2-变倍减；3-变倍停；4-变倍步进设置；5-变倍值设置；0x0F-变倍查询；（步进默认为1）； 
							 参数2：参数1为4时，该值为步进值，参数1为5时，该值为变倍值；其余情况下默认为0；*/
	VL_FOCUS_CTRL = 5,			/*调焦控制--同上*/
	VL_FOCUS_AUTO,			/*自动聚焦*/
	VL_FOCUS_ONE_SET,		/*一键聚焦*/
	VL_LIGHT_CTRL = 8,     		/*参数1：1-亮度加；2-亮度减；3-亮度自动；4-亮度手动；5-亮度设置；0x0f-亮度读取；（步进默认为1）
							参数2：参数1为5时，该值为亮度值；
							注意：根据不同的外设，加减设置需要先切换到手动模式。*/
	VL_CONTRAST_CTRL = 9,	/*对比度控制 -- 同上*/
	VL_GAIN_CTRL,			/*增益控制 -- 同上*/
	VL_SATURATION_CTRL,		/*饱和度控制 -- 同上*/
	VL_SHAR_CTRL,			/*锐度控制 -- 同上*/
	VL_EXPOSURE_CTRL, 		/*曝光控制：参数1：1-曝光开；2-曝光关；3-曝光重置；*/
	VL_EXPOSURE,			/*曝光补偿 vs2030:CMD_DISABLE为-；CMD_ENABLE为+*/
	
	VL_CH_MLARGE = 0x10,	/*极大视场*/
	VL_CH_LARGE ,			/*大视场*/
	VL_CH_MI,				/*中视场*/
	VL_CH_SMA,				/*小视场*/
	VL_CH_MIN,				/*极小视场*/
	VL_ELE_ZOOM,        	/*电子变倍 0-关，1-开*/
	VL_VIEW_ANGLE,    		/*视场角设置*/
	
	VL_IMAGE_ENHANCE = 0x20,/*图像增强 0-关，1-开,2-加，3-减*/
	VL_CALI, 				/*图像校正*/
	VL_DEFOG_RANK,  		/*透雾档位设置*/
	VL_FRAME_RATE,			/*帧频设置 参数1：0-24fps;1-25fps;2-30fps*/
	VL_VIEW_RANK,			/*视场档位设置*/
	VL_LIGHT_INHIBITE,		/*强光抑制 1-开,2-关*/
	
	VL_CMD_END,
}SYS_VL_CMD_CTRL_E;

/*激光指令集*/
/*控制参数*/
#define LAZ_5HZTIME_SET 15
#define LAZ_CODENUM_SET (00)
/*激光串口波特率设置*/
typedef enum
{
	BAUD_9600,
	BAUD_19200,
	BAUD_115200,
	BAUD_256000,
	BAUD_460800,
	BAUD_921600,
	BAUD_57600,
}COM_BAUD_TYPE;


//激光指令
typedef enum
{
	LZ_LIGHT_START,
	/*参数1：cmd-指令码*/
	LZ_STATUS_ASK, 			/*参数1：1-版本查询；2-自检结果查询；3-出光次数查询*/
	LZ_DETEC_STOP = 0x2,	/*测距停止*/
	LZ_SINGLE_DETECT,		/*单次测距*/
	LZ_DETECT_FREQ_1,		/*1hz测距*/
	LZ_DETECT_FREQ_5,		/*5hz测距*/
	LZ_DETECT_FREQ_10,		/*10hz测距*/
	LZ_DETECT_FREQ_20,		/*5hz测距*/
	LZ_DETECT_MARK,	
	LZ_DETECT_FREQ_SET,		/*测距频率设置 参数1：测距频率；1Hz/bit*/
	LZ_CODE_WRITE,			/*编码装订*/
	LZ_TX_STA,				/*SYS_LASER_ENABLE_MODE tx使能 1：开,2:关(apd电源设置)*/
	LZ_RX_STA,				/*rx使能 1：开*/
	
	LZ_DETECT_DISTANCE_REQ,		/*测距距离查询*/
	LZ_LIGHT_FAR_DISTANCE_REQ,	/*设置的最远距离查询*/
	LZ_DETECT_CODE_REQ,			/*模组序列号读取*/
	LZ_DETECT_SELFCHECK,		/*自检*/
	LZ_DETECT_INFO_REQ,			/*（LASER_LRS_0610A）测距参数查询*/
	LZ_DETECT_ARM_ID,			/*� ��LASER_LRS_0610A）电路板编号查询*/

	/*参数1：cmd-指令码
	  参数2：选通值/能量控制模式
	  参数3：额定能量的百分比*/
	LZ_DYNAMI = 0x40,			/*动静态设置 1：动；2：静*/
	LZ_RANGE_GATE_VALUE,   		/*选通值设置-参数1：cmd-指令码；参数2：选通值*/
	LZ_POWER_CFG,				/*能量设置*/
	LZ_LIGHT_DELAY,				/*照射时延设置*/
	LZ_LIGHT_FAR_DISTANCE,  	/*最远距离设置*/
	LZ_LIGHT_TIMEOUT_VALUE, 	/*激光连续工作超时时间设置*/
	LZ_SELF_DIS,				/*编码自毁 1-自毁失败，2-自毁成功*/
	/*------------------以上位协议规定--------------------*/
	/*参数1：cmd-指令码
	  参数3：序列码*/
	LZ_DETECT_PRECISION,	  	/*精频码照射*/
	LZ_DETECT_CHANGE_INTERVAL,	/*变间隔码照射*/
	LZ_DETECT_PLUSE,		  	/*脉冲序列码照射*/
	LZ_LIGHT_EXT,			  	/*外触发照射*/
	LZ_LIGHT,					/*激光照射*/
	LZ_CTRL_CMD_END,//功能指令分隔符
	
	/*参数1：cmd-指令码
	  参数2：data-使能信息，CMD_ENABLE；CMD_DISABLE*/
	LZ_CODE_STA,				/*编码状态 1：启动*/			
	LZ_GOAL,					/*（LSP_LD_0820）目标状态 1-末  ；2-首
								（LASER_LRS_0610A|LASER_LSP_0410）目标模式 1-单目标，2-三目标，3-首末目标*/
	LZ_VERSION,					/*版本查询*/
	LZ_LIGHT_CNT,				/*出光次数查询*/
	LZ_RESET,					/*恢复出厂设置(兼容lrs_0610a的出光次数清0)*/
	LZ_COM_BAUD_CFG,			/*串口波特率设置: COM_BAUD_TYPE(枚举)*/
	LZ_DETECT_FREQ_CONTINUOUS,	/*连续测距*/
	LZ_GOAL_FIRST_TARGET,		/*测距首目标设置*/
	LZ_GOAL_LAST_TARGET,		/*测距末目标设置*/
	LZ_GOAL_MULTI_TARGET,		/*测距多目标设置*/
	LZ_RANGE_GATE_VALUE_MIN,	/*最小选通值设置*/
	LZ_RANGE_GATE_VALUE_MAX,	/*最大选通值设置*/
	LZ_DETECT_DISTANCE_REQ_MIN,	/*最小选通值查询*/
	LZ_DETECT_DISTANCE_REQ_MAX,	/*最大选通值查询*/
	LZ_DETECT_END,
}SYS_LASER_DETECT_MODE_E;


/*跟踪器指令集*/
#if 0
typedef enum
{
	TRA_CMD_INVALID = 0x00,	 // 无效指令
	TRA_POINT_SEL,	 	// 跟踪点选择，(先内部跟踪，等收到【跟踪确认】指令后，发出脱靶量实现闭环跟踪。)
	TRA_ENABLE ,	 	// 跟踪确认
	TRA_DISABLE,		// 退出跟踪
	TRA_IDENTIFY,		// 识别跟踪，进入捕获状态，一旦识别到指定类别的目标以后，选择离与3~6字节代表的点最近的目标进行跟踪
	TRA_WITH_CROSSHAIR, // 十字中心快速跟踪
	TRA_WITH_PIXEL, 	// 像素位置快速跟踪，不用等待【跟踪确认】指令，直接发出脱靶量实现闭环跟踪。
	TRA_WITH_PIXEL_2,	// 像素跟踪2，不用等待【跟踪确认】指令，直接发出脱靶量实现闭环跟踪。
	TRA_LARGE_SCOPE,	// 20241221  大范围跟踪

	TRA_WAVE_GATE_AUTO = 0x10, // 自动波门
	TRA_WAVE_GATE_LAR ,	 // 大波门
	TRA_WAVE_GATE_MID ,	 // 中波门
	TRA_WAVE_GATE_SMA ,	 // 小波门

	TRA_TARGET_DETECT_ON = 0x20,// 目标检测开，仅识别框、无编号，配合65字节
	TRA_TARGET_DETECT_OFF, 		// 目标检测关，LWB自行增加的指令，仅测试使用。
	TRA_TARGET_RECOGNIZE_ON ,   // 目标识别开，稳定编号，配合65字节
	TRA_TARGET_RECOGNIZE_OFF,	// 目标识别关

	TRA_WITH_ID, 		// 编号跟踪，无特殊操作，只是将给定编号的识别目标的脱靶量按照20ms周期上报。

	TRA_WAVE_GATE_UP,	// 波门微调上
	TRA_WAVE_GATE_DOWN,  // 波门微调下
	TRA_WAVE_GATE_LEFT,  // 波门微调左
	TRA_WAVE_GATE_RIGHT, // 波门微调右

	TRA_CROSSHAIR_UP,	 // 主视频十字丝上移
	TRA_CROSSHAIR_DOWN,	 // 主视频十字丝下移
	TRA_CROSSHAIR_LEFT,	 // 主视频十字丝左移
	TRA_CROSSHAIR_RIGHT, // 主视频十字丝右移
	TRA_CROSSHAIR_RESET,// 主视频十字丝归中
	TRA_CROSSHAIR_SAVE,	// 主视频十字丝存储
	TRA_VIDEO_MODE,		//视频模式选择

	TRA_VIDEO_ON,		// 录像开
	TRA_VIDEO_OFF, 		// 录像关，默认

	TRA_ALL_BLANKING, 	// 全消隐，视频上不叠加字符。
	TRA_ALL_DISPLAY, 	// 全显示，视频上显示所有字符，默认。

	TRA_ETH_VIDEO1_ON = 0x40,// UDP网口视频端口1开，端口号10301
	TRA_ETH_VIDEO2_ON,		// UDP网口视频端口2开，端口号10302
	TRA_ETH_TWO_VIDEO_ON,	// UDP网口视频接口两路同时输出模式开，当两个端口同时输出，分别输出两个成像设备的单显画面(这个模式不会进行画中画操作)。
	TRA_ETH_TWO_VIDEO_OFF, 	// UDP网口视频接口两路同时输出模式关，关闭时，两路端口输出同一视频源。
	TRA_ETH_VIDEO_OFF,		// UDP网口视频接口关，默认
	
	TRA_SDI_OSD_OFF, // SDI视频OSD关，默认
	TRA_SDI_OSD_ON,	 // SDI视频OSD开

	TRA_ETH_OSD_OFF, // 网口视频OSD关
	TRA_ETH_OSD_ON,	 // 网口视频OSD开，默认

	TRA_RTSP_VIDEO_OFF, // RTSP网口视频接口关，默认
	TRA_RTSP_VIDEO_ON,  // RTSP网口视频接口开
	TRA_SDI_VIDEO_OFF,	 // SDI视频开，默认
	TRA_SDI_VIDEO_ON, // SDI视频关
	TRA_TARGET_TEMPLATE_STAPLE,	// 目标模板装订，装订过程中跟踪器暂停其它操作，完成后恢复
	TRA_TARGET_TEMPLATE_OUTPUT,	// 目标模板输出开，1）	仅在目标跟踪模式下有效，截取视场中心一定大小的图像并压缩输出；2）上述操作不影响正常跟踪。
	TRA_TARGET_TEMPLATE_NO_OUTPUT,// 目标模板输出关

	TRA_RECOGNIZE, // 跟踪、识别同时工作。
	TRA_RECOGNIZE_SEPARATE, // 跟踪、识别独立工作。

	TRA_AREA_OF_INTEREST_10X, // 感兴趣区域――1倍模型尺寸
	TRA_AREA_OF_INTEREST_15X, // 感兴趣区域――1.5倍模型尺寸
	TRA_AREA_OF_INTEREST_20X, // 感兴趣区域――2倍模型尺寸
	TRA_AREA_OF_INTEREST_FULL, // 感兴趣区域――全画幅

	TRA_CODE_RATE, // 压缩视频码率设置，码值参数位于64字节：分辨率：0.1M/bit，范围：0.5~6M

	TRA_VL_OUTPUT_FPS, // 可见光原始图像每秒输出帧数，参数位于64字节：分辨率：1帧/bit，范围：0~5帧(参数为0时即不输出)
	TRA_IR_OUTPUT_FPS,	 // 红外原始图像每秒输出帧数，参数位于64字节：分辨率：1帧/bit，范围：0~5帧(参数为0时即不输出)

	TRA_VIDEO_OUT_TS,  // 切换为TS流输出，默认。
	TRA_VIDEO_OUT_UDP, // 切换为UDP组播输出。

	TRA_VERSION_REQ, // 版本号查询
//	TRA_VIDEO_OUT_TS = 0xBA, // 红外图像增强
	TRA_ELE_STABLE_ON,	 // 电子稳像开
	TRA_ELE_STABLE_OFF, // 电子稳像关

	TRA_OSD_COLOR_WHITE, // 字符白色
	TRA_OSD_COLOR_RED,	// 字符红色
	TRA_OSD_COLOR_BLUE,	// 字符蓝色

	TRA_OSD_SHOE_DMS , // 度分秒
	TRA_OSD_SHOE_D ,	 // 度（带小数）显示

	TRA_MODE_PTRA, // 跟踪方式-自动
	TRA_MODE_STRA, // 跟踪方式-场景

	TRA_DECENTER_UP,	 // 偏心微调上
	TRA_DECENTER_DOWN,	 // 偏心微调下
	TRA_DECENTER_LEFT,	 // 偏心微调左
	TRA_DECENTER_RIGHT, // 偏心微调右
	TRA_DECENTER_CLEAR , // 偏心跟踪清零

	TRA_PUSH_PACK_ON = 0xDD,  // 推流数据打包开
	TRA_PUSH_PACK_OFF = 0xDE, // 推流数据打包关

	TRA_CMD_END,
} TRACK_CTRL_CMD_TYPE_INFO;
#else
typedef enum
{
	TRA_CMD_INVALID = 0x00,	 // 无效指令
	TRA_WAVE_GATE_CTRL = 0xAB, /*参数1：
								0x10 – 自适应波门
								0x13 – 大波门
								0x14 – 中波门
								0x15– 小波门
								0x16 – 波门减*/
	TRA_ELE_STABLE = 0xAC,	/*电子稳像*/
	TRA_RECORD_VIDEO = 0xAD,/*录像*/
	TRA_CODE_RATE = 0xB1,	/*压缩视频控制*/
	TRA_CROSSHAIR_CTRL,		/*十字丝控制*/
	TRA_TARGET_DETECT,		/*目标检测，仅识别框、无编号，配合65字节*/
	TRA_TARGET_RECOGNIZE,	/*目标识别开，稳定编号，配合65字节*/
	TRA_OSD_CTRL,			/*OSD控制*/
	TRA_CATCH_IMAGE,		/*抓图控制*/
	TRA_COG_TO_TRACK,		/*识别转跟踪*/
}TRACK_CTRL_CMD_TYPE_E;
#endif
	
	
	
#endif
