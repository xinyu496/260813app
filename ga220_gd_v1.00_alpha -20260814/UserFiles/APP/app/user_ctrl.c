#include "Common/base_inc.h"
extern TIM_HandleTypeDef htim6;

#if LASER_LRD_0301
#include "../laser/laser_lrd_0301.h"
#endif

#if VIDEO_TRACK_INCLUDE
#include "../cam_track/Track.h"
#endif
#include "Bsp/bsp_timer.h"
extern USER_CNT_T User_Tick;


#include "tim.h"

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:开始加热
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void start_heat(void)
{
	HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_1 );
	HAL_TIM_PWM_Start(&htim3 ,TIM_CHANNEL_4 );	
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:停止加热
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/

void stop_heat(void)
{
	HAL_TIM_PWM_Stop(&htim1 ,TIM_CHANNEL_1 );
	HAL_TIM_PWM_Stop(&htim3 ,TIM_CHANNEL_4 );	
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:同步信号触发回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t gpio_int_flag;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == PPS_EXIT_Pin)
	{
		gpio_int_flag = 1;
	}
}
void Track_Loop_Handle(void)
{
	if(gpio_int_flag == 1)
	{
		gpio_int_flag = 0;
		
		TRACK_API_Ctrl_SendHandle(0,0);
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光轮询发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LaserCtrl_Loop_Handle(void)
{
    if (User_Tick.LaserCtrl >= 100)
    {
        /*每隔1s查询激光的版本号，以确定激光正常运行*/
        User_Tick.LaserCtrl = 0;
#if LASER_INCLUDE
        Laser_Ctrl_SendHandle(LZ_SINGLE_DETECT, 0, 0);
#endif
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光周期性控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Laser_Handle(void)
{
    uint8_t lz_cmd_type;
    uint8_t lz_ctrl_data;
    uint8_t lz_code;
    //通过上位机下发的指令，切换激光相关参数
    SYS_LASER_STA_T laser_info_sta;

#if 0
    //激光控制指令执行条件
    /*地面状态，激光不工作*/
    if (EO_Status.eo_para.wheel_sta == PLANEGROUND)
    {
        return;
    }

    /*在非跟踪状态时，不打激光*/
    if ((main_info.sys_mode != Sys_Track) && (main_info.sys_mode != Sys_Serch))
    {
        return;
    }

    /******激光在内框架0.8°内停止工作 Start******/
    if (User_Cmd.MC_TO_LZCtrl)
    {
        EO_Para.LZ_ProtectSta = 0;
        User_Cmd.LZOld_WorkSta = 0;
        User_Tick.LZ_ProtectTime = HAL_GetTick();
    }
#endif
    /*周期性的激光控制的处理*/
    LaserCtrl_Loop_Handle();

    //该函数周期性执行，检测到Master_Ctrl发生变化则立即执行
    if (CONFIG_Get_Master_Ctrl_Sta() != MASTER_CTRL_LASER)
    {
        return;
    }

    switch(CONFIG_Get_Master_Ctrl_Cmd(MASTER_LASER_PARA))
    {
#if 0
		case MASTER_LASER_POWER_ON: /*激光器上电*/
			//LaserPower_On;
			laser_info_sta.LZ_Powersta = 1;
			break;
		case MASTER_LASER_POWER_OFF: /*激光器下电*/
			//LaserPower_Off;
			laser_info_sta.LZ_Powersta = 0;
			break;
#endif
		case LZ_SINGLE_DETECT: /*激光单次测距*/
			lz_cmd_type = LZ_SINGLE_DETECT;
			break;

		case LZ_DETECT_FREQ_5: /*激光连续测距（5HZ，时间待定）*/
			lz_cmd_type = LZ_DETECT_FREQ_5;
			break;

		case LZ_DETECT_PRECISION: /*激光照射*/
			lz_cmd_type = LZ_DETECT_PRECISION;
			//main_info.LZCtrl_Para2 = main_info;//.LazNum;
			//User_Cmd.LZCtrl_Para1 = 20;
			break;
		case LZ_DETEC_STOP: /*激光测距/照射停*/
			lz_cmd_type = LZ_DETEC_STOP;
			break;
		case LZ_RX_STA: /*激光使能开(接收)*/
			lz_cmd_type = LZ_RX_STA;
			lz_ctrl_data = CMD_ENABLE;
			break;
//		case MASTER_LASER_DISABLE: /*激光使能关(接收)*/
//			lz_cmd_type = LZ_RX_STA;
//			lz_ctrl_data = CMD_DISABLE;
//			break;

		default:
			//都不符合，则状态清空,清空后返回
			CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
			return;
		}
    //cmd_para1:使能或者选通值等通用参数 cmd_para2:仅用于放照射序列码
#if LASER_INCLUDE
    Laser_Ctrl_SendHandle((SYS_LASER_DETECT_MODE)lz_cmd_type, lz_ctrl_data, lz_code);
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外热像仪初始化配置
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
/*初始化状态机*/
typedef enum
{
    PERIGHERALS_INIT_WAIT,
    PERIGHERALS_INIT_STEP1,
    PERIGHERALS_INIT_STEP2,
    PERIGHERALS_INIT_STEP3,
    PERIGHERALS_INIT_STEP4,
    PERIGHERALS_INIT_END,
} PERIGHERALS_INIT_STEP_ENUM;
uint8_t IR_Init_Step_By_step(void)
{
    //上电后初始化配置
    static uint8_t ir_init_step = PERIGHERALS_INIT_WAIT;
    uint8_t send_type;
    uint8_t temp_buf;
    switch (ir_init_step)
    {
    case PERIGHERALS_INIT_WAIT: /*对比度设置*/
        send_type = IR_CONTRAST_CTRL;
        temp_buf = 20;
        ir_init_step = PERIGHERALS_INIT_STEP1;
        break;
    case PERIGHERALS_INIT_STEP1: /*亮度设置*/
        send_type = IR_LIGHT_CTRL;
        temp_buf = 76;
        ir_init_step = PERIGHERALS_INIT_STEP2;
        break;
    case PERIGHERALS_INIT_STEP2: /*进入图像增强*/
        send_type = IR_IMAGE_ENHANCE;
        temp_buf = CMD_ENABLE;
        ir_init_step = PERIGHERALS_INIT_END;
        break;
    default :
        ir_init_step = PERIGHERALS_INIT_END;
        break;
    }
    Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, send_type, (uint8_t *)&temp_buf);
    return ir_init_step;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外热像仪周期性发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IRCtrl_Loop_Handle(void)
{
    if (User_Tick.IRCtrl >= 1000)
    {
        /*每隔1s查询红外的系统状态，以确定红外正常运行*/
        User_Tick.IRCtrl = 0;
        Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, IR_SYS_STA_REQ, 0);
        //如过存在另一款型号的红外，直接将发送函数加载下面即可
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外周期性控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_IR_Handle(void)
{
    SYS_IR_CMD_CTRL send_type = IR_SYS_STA_REQ;
    SYS_IR_STA_T main_ir_info;
    uint32_t temp_buf;
    float cali_irfocus[VIEW_FOCUS_END];

    //获取标定的焦距值
    CONFIG_Get_Cali_irFocus(cali_irfocus);

    //上电初始化函数
    if (PERIGHERALS_INIT_END != IR_Init_Step_By_step())
    {
        return;
    }
    //周期轮询函数
    IRCtrl_Loop_Handle();

    //如果不是红外相机的处理，则直接退出该函数
    if (CONFIG_Get_Master_Ctrl_Sta() != MASTER_CTRL_IR)
    {
        return;
    }
    main_ir_info = CONFIG_Get_Ir_Info();
    switch (CONFIG_Get_Master_Ctrl_Cmd(MASTER_CTRL_IR))
    {
#if 0
    case MASTER_FOCUS_ADD: /*调焦加*/
        send_type = IR_FOCUS_STEP_ADD;
        break;
    case MASTER_FOCUS_MINUS: /*调焦减*/
        send_type = IR_FOCUS_STEP_MINUS;
        break;
    case MASTER_FOCUS_AUTO: /*变焦*/
        send_type = IR_ZOOM_POSITION;
        if (main_ir_info.ir_focus_value != 0)
        {
            temp_buf = IRMinZoom + ((main_ir_info.ir_focus_value * (IRMaxZoom - IRMinZoom)) / 255);
        }
        break;
    case MASTER_VIEW_LARGE: /*视场切换-大视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[LARGE_VIEW_FOCUS];
        break;
    case MASTER_VIEW_MID: /*视场切换-中视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[MID_VIEW_FOCUS];
        break;
    case MASTER_VIEW_SMALL: /*视场切换-小视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
        break;
    case MASTER_VIEW_SSMALL: /*视场切换-极小视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[MIN_VIEW_FOCUS];
        break;
    case MASTER_ZOOM_IN:			/*ZOOM IN*/
        switch (main_ir_info.irview_Now) // 放大
        {
        case 1:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MID_VIEW_FOCUS];
            break;
        case 2:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
            break;
        case 3:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MIN_VIEW_FOCUS];
            break;
        default:
            break;
        }
        break;
    case MASTER_ZOOM_OUT:			/*ZOOM OUT*/
        switch (main_ir_info.irview_Now) // 放大
        {
        case 4:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
            break;
        case 3:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MID_VIEW_FOCUS];
            break;
        case 2:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[LARGE_VIEW_FOCUS];
            break;
        default:
            break;
        }
        break;
    case MASTER_VIEW_ELECTRIC: /*电子变倍*/
        send_type = IR_ZOOM_DIT;
        switch (main_ir_info.elezoom)
        {
        case 0:
        case 1:
            temp_buf = 2;
            break;
        case 2:
        case 3:
            temp_buf = 4;
            break;
        case 4:
            temp_buf = 0;
            break;
        default:
            break;
        }
        break;
    case MASTER_PICTURE_AUTO: /*图像自动调节*/
        // User_Cmd.IRCtrl_type = MASTER_PICTURE_AUTO;
        //ir_init_flag = 1;
        break;

    case MASTER_GAIN_ADD: /*增益＋*/
        send_type = IR_SET_CONTRAST;
        if (main_ir_info.contrast_level < 255 - main_ir_info.contrast_step)
        {
            temp_buf = main_ir_info.contrast_level + main_ir_info.contrast_step;
        }
        else
        {
            temp_buf = 255;
        }
        break;
    case MASTER_GAIN_MINUS: /*增益-*/
        send_type = IR_SET_CONTRAST;
        if (main_ir_info.contrast_level > IRCONTRST_STEP)
            temp_buf = main_ir_info.contrast_level - IRCONTRST_STEP;
        else
            temp_buf = 0;
        break;
    case MASTER_LIGHT_ADD: /*亮度＋*/
        send_type = IR_SET_LIGHT;
        if (main_ir_info.light_level < 255 - IRLIGHT_STEP)
            temp_buf = main_ir_info.light_level + IRLIGHT_STEP;
        else
            temp_buf = 255;
        break;
    case MASTER_LIGHT_MINUS: /*亮度-*/
        send_type = IR_SET_LIGHT;
        if (main_ir_info.light_level > IRLIGHT_STEP)
            temp_buf = main_ir_info.light_level - IRLIGHT_STEP;
        else
            temp_buf= 0;
        break;
    case MASTER_FOCUS_ON_AUTO: /*自动聚焦*/
        send_type = IR_FOCUS_AUTO;
        break;
#if 0
    case MASTER_PICTURE_ENHANCE_ON: /*进入图象增强*/
        User_Cmd.IRCtrl_Cmd = IR_IMAGE_ENHANCE;
        User_Cmd.IRCtrl_Para1 = CMD_ENABLE;
        break;
    case MASTER_PICTURE_ENHANCE_OFF: /*退出图象增强*/
        User_Cmd.IRCtrl_Cmd = IR_IMAGE_ENHANCE;
        User_Cmd.IRCtrl_Para1 = CMD_DISABLE;
        break;
#endif
    case MASTER_PICTURE_ENHANCE_ON: /*进入图象增强 实质是DDE调整*/
        send_type = IR_DDE_RANGE;
        if (main_ir_info.DDE_level == 30)
            temp_buf = 100;
        else
            temp_buf = main_ir_info.DDE_level + IRDDE_STEP;
        //temp_buf = CHECK_RESET(temp_buf,int32_t);
        CHECK_MAX(temp_buf,uint32_t);
        break;
    case MASTER_PICTURE_ENHANCE_OFF: /*退出图象增强 实质是DDE给30*/
        send_type = IR_DDE_RANGE;
        temp_buf = 30;
        break;
    case MASTER_HW_POWER_ON: /*红外上电*/
        IRPower_On;
        main_ir_info.IR_PowerSta = 1;
        break;
    case MASTER_HW_POWER_OFF: /*红外下电*/
        IRPower_Off;
        main_ir_info.IR_PowerSta = 0;
        break;
    case MASTER_HW_PICTURE_CALIBRATE: /*红外图像校准 虚焦校正*/
        send_type = IR_CALIBRATION;
        temp_buf = 3;
        break;
    case MASTER_HW_PICTURE_CHANGE: /*红外正负像切换*/
        send_type = IR_BLACK_WHITE;
        main_ir_info.ir_status.black_white = ~(main_ir_info.ir_status.black_white) & 0x01;
        temp_buf = main_ir_info.ir_status.black_white;
        break;

    case MASTER_VIEW_MATCH_ON:/*视场匹配*/
        if(CONFIG_Get_Main_Video() == MainVideo_VL)
        {
            send_type = IR_ZOOM_POSITION;
            temp_buf = (int32_t)USER_Ctrl_View_Match(main_ir_info.IRRange_H);
        }
        else
        {
            return;
        }
        break;

        /*内部测试用*/
#if 0
    case MASTER_IR_TESTCTRL: /*红外透传*/
        User_Cmd.IRCtrl_Cmd = IR_MAINTIAN_SET;
        memcpy(User_Cmd.IR_CtrlData, User_Cmd.Dev_CtrlData, 10);
        if (Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, User_Cmd.IRCtrl_Cmd, (uint8_t *)&User_Cmd.IR_CtrlData) == HAL_OK)
        {
            User_Cmd.IRCtrl_Cmd = NULL;
            User_Cmd.IRCtrl_Para1 = NULL;
        }
        break;
#endif
    default:
        //都不符合，则状态清空,清空后返回
        CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
        return;
#endif
    }
    Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, send_type, (uint8_t *)&temp_buf);
    CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
}

#if 0
User_TickTypeDef User_Tick;		  /*用户定时计数*/
#define SF_SLEW_MAX (32000)
#define SF_SLEW_MIN (-32000)
/*==============================================================
*FUNCTION NAME:系统硬件初始化
*DISCRIPTION:激光、红外等上电
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Hw_Power_init(void)
{
    LaserPower_On;
    IRPower_On;
    FAN_On;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:地理跟踪、定位解算
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
extern float LaserDistance;
void USER_Ctrl_GEO_process(void)
{
    EXT_GD_DATA_T ext_gd = {0};
    INSIDE_GD_DATA_T inner_gd = {0};
    GEOTrack_DataDef aim_info = {0};
    SYS_SF_DATA_T sf_info = {0};
    SYS_LASER_STA_T laser_info = {0};

    if ( User_Tick.GEO_tick >= 40) /*20ms间隔*/
    {
        User_Tick.GEO_tick = 0;
        /*********************参数获取Start******************/
        /*INS为FJ惯导数据*/
        ext_gd = CONFIG_Get_Ext_position();//获取最新的外惯导信息
        INS.Pitch = ext_gd.pitch;
        INS.Roll = ext_gd.roll;
        INS.Yaw = ext_gd.heading;
        INS.Yaw = UTL_Normalize_Angle(INS.Yaw);
        INS.longitude = ext_gd.longtitude * degree_to_rad;
        INS.lattitude = ext_gd.latitude * degree_to_rad;
        INS.height = ext_gd.alt;
        INS.Vel_E = ext_gd.speed_east;
        INS.Vel_N = ext_gd.speed_north;
        INS.Vel_S = ext_gd.speed_sky;

        /*INS_Inner为内惯导数据*/
        inner_gd = CONFIG_Get_Internal_position();//获取最新的内惯导信息
        INS_Inner.Pitch = inner_gd.pitch;
        INS_Inner.Roll = inner_gd.roll;
        INS_Inner.Yaw = inner_gd.heading;
        INS_Inner.Yaw = UTL_Normalize_Angle(INS.Yaw);
        INS_Inner.longitude = inner_gd.longtitude * degree_to_rad;
        INS_Inner.lattitude = inner_gd.latitude * degree_to_rad;
        INS_Inner.height = inner_gd.alt;

        /*PT_Angle为吊舱框架角*/
        sf_info = CONFIG_Get_Sf_Info();//获取最新的伺服信息
        PT_Angle.Fw = sf_info.FW_Combine;
        PT_Angle.Fy = sf_info.FY_Combine;

        /*Target为地理跟踪目标经纬高：外部输入的目标点经纬高*/
        aim_info = CONFIG_Get_GEO_Aim_Info();
        Target.longitude = (double)aim_info.TargetLon * degree_to_rad;
        Target.lattitude = (double)aim_info.TargetLat * degree_to_rad;
        Target.height = aim_info.TargetAlt;
        /*********************参数获取End******************/

        /***********************目标距离计算***********************/
        aim_info.TargetDis_Calu = GEO_Target_Get_Dis();
        /***********************目标距离计算***********************/

        /***********************地理跟踪***********************/
        if (CONFIG_Get_SF_Mode() == P_GEO)
        {
            GEO();
            aim_info.Heading_FrPT = OrientLoad_PT.Orient_Load_FW;
            aim_info.Pitch_FrPT = OrientLoad_PT.Orient_Load_FY;

            aim_info.Heading_FrInner = OrientLoad_Inner.Orient_Load_FW;
            aim_info.Pitch_FrInner = OrientLoad_Inner.Orient_Load_FY;
        }
        /***********************LMC***********************/
        else if (CONFIG_Get_SF_Mode() == P_Vfollow)
        {
            if (CONFIG_Get_LMC_Sta() == 0x01) // LMC开
            {
                /* TODO 参数没传 */
//				LMC_Vel_Cal();
                if (sf_info.FY_Combine < -75.0)
                {
                    LMC.Fw = 0;
                }
            }
            else
            {
                LMC.Fw = 0;
                LMC.Fy = 0;
            }
            aim_info.LMC_Fw = LMC.Fw;
            aim_info.LMC_Fy = LMC.Fy;
        }
        else
        {
            GPS_Orient_Result.height = 0;
            GPS_Orient_Result.lattitude = 0;
            GPS_Orient_Result.longitude = 0;
            Target_Motion.Vel = 0;
            Target_Motion.Yaw = 0;

            OrientLoad_PT.Orient_Load_FW = 0;
            OrientLoad_PT.Orient_Load_FY = 0;
            OrientLoad_Inner.Orient_Load_FW = 0;
            OrientLoad_Inner.Orient_Load_FY = 0;

            LMC.Fw = 0;
            LMC.Fy = 0;
        }

        /*************************单目标定位***************************/
#if GEOPOSTION
        /*激光正常工作的时候进行计算*/
        static uint8_t LZDis_effect = 0;
        static float Laz_oldDis = 0;
        if ((laser_info.laser_sta.apd_lock == 0x00) && (laser_info.sys_sta != LASER_WORK_STAY))
        {
            if((laser_info.distance[0]) && (laser_info.distance[0] != 65535))
                aim_info.Slant_Dis = laser_info.distance[0];

            if(Laz_oldDis != aim_info.Slant_Dis)
                LZDis_effect = 1;

            Laz_oldDis = aim_info.Slant_Dis;
        }
        else
        {
            aim_info.Slant_Dis = 0;
            Laz_oldDis = 0;
        }

        if ((LZDis_effect)&&(aim_info.Slant_Dis))
        {
            LZDis_effect = 0;

            LaserDistance = aim_info.Slant_Dis;

            /***********基于外惯导**********/
            Target_Orient_Fun();

            /***********基于内惯导**********/
            Orient.Orient_Yaw = INS_Inner.Yaw;
            Orient.Orient_Pitch = INS_Inner.Pitch;
            Orient.Orient_Roll = INS_Inner.Roll;

            aim_info.CalcLon = GPS_Orient_Result.longitude*rad_to_degree;
            aim_info.CalcLat = GPS_Orient_Result.lattitude*rad_to_degree;
            aim_info.CalcAlt = GPS_Orient_Result.height;
        }
#endif
    }
    CONFIG_Set_GEO_Aim_Info(aim_info);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:校靶
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Bore_Zero(uint8_t *geo_data)
{
    //校靶，上位机中填好方位俯仰数据之后下发解析。
    //上电读过一次了，此时还读吗？
    //处理校靶的方位数据
    SFSaveAngle_TypeDef sf_geo_info = {0};

    sf_geo_info = CONFIG_Get_Geo_Zero();

    sf_geo_info.geo_fw_zero = Query_Data(STM32FALSH_Servo_ADDR, 0);
    if (geo_data[0] == 1)
    {
        //取高16位的数据-传进来的数据，等于新的0位修正的值
        sf_geo_info.geo_fw_zero = ((sf_geo_info.geo_fw_zero << 16) >> 16) - (((int16_t)geo_data[1]));
    }
    else
    {
        sf_geo_info.geo_fw_zero = ((sf_geo_info.geo_fw_zero << 16) >> 16) + (((int16_t)geo_data[1]));
    }
    Record_Data(STM32FALSH_Servo_ADDR, (int32_t)(sf_geo_info.geo_fw_zero), 0);
    HAL_Delay(100);
    //处理校靶的俯仰数据
    sf_geo_info.geo_fy_zero = Query_Data(STM32FALSH_Servo_ADDR, 1);
    if (geo_data[2] == 1)
    {
        sf_geo_info.geo_fy_zero = ((sf_geo_info.geo_fy_zero << 16) >> 16) - (((int16_t)geo_data[3]));
    }
    else
    {
        sf_geo_info.geo_fy_zero = ((sf_geo_info.geo_fy_zero << 16) >> 16) + (((int16_t)geo_data[3]));
    }
    Record_Data(STM32FALSH_Servo_ADDR, (int32_t)(sf_geo_info.geo_fy_zero), 1);
    HAL_Delay(100);
}
/*==============================================================
*FUNCTION NAME:视场等比例匹配
*DISCRIPTION:根据当前视场角和需要改变的视场角算出当前需要设置的值
*PARAMETERS:in-当前焦距值
*RETURN:需要控制的焦距变值
*N/A：
*NOTES:
*HISTORY:
*==============================================================*/
float USER_Ctrl_View_Match(float curr_view)
{
//视场角等比例匹配，相当于红外的和可见光的视场角等比例匹配
    float CalcFocus = 0.0;
    float AimIRRange = 0.0, AimVLRange = 0.0;
    float cur_ir_range = 0.0,cur_vl_range = 0.0;
    float cali_vlfocus[VIEW_FOCUS_END];
    float cali_irfocus[VIEW_FOCUS_END];
    //获取标定的焦距值
    CONFIG_Get_Cali_vlFocus(cali_vlfocus);
    CONFIG_Get_Cali_irFocus(cali_irfocus);
    //当前计算出的视场角
    //当前主视频为可见光时，需要红外视场角和可见光视场角出来的效果保持一致，
    if (CONFIG_Get_Main_Video() == MainVideo_VL)
    {
        AimIRRange = ((curr_view - VLSetMinView) / (VLSetLarView - VLSetMinView)) * (IRSetLarView - IRSetMinView) + IRSetMinView; // 互换
        if (AimIRRange > IRSetLarView)
        {
            AimIRRange = IRSetLarView;
        }
        else if (AimIRRange < IRSetMinView)
        {
            AimIRRange = IRSetMinView;
        }

        if (fabs(AimIRRange - cur_ir_range) >= 0.5)
        {
            CalcFocus = UTL_Focus_Calc(VLPixel,VLPNum_H,AimIRRange);
            if (CalcFocus <= cali_irfocus[LARGE_VIEW_FOCUS])
            {
                CalcFocus = cali_irfocus[LARGE_VIEW_FOCUS];
            }
            else if (CalcFocus >= cali_irfocus[MIN_VIEW_FOCUS])
            {
                CalcFocus = cali_irfocus[MIN_VIEW_FOCUS];
            }
        }
    }
    else
    {
        AimVLRange = ((curr_view - IRSetMinView) / (IRSetLarView - IRSetMinView)) * (VLSetLarView - VLSetMinView) + VLSetMinView; // 互换

        if (AimVLRange > VLSetLarView)
        {
            AimVLRange = VLSetLarView;
        }
        else if (AimVLRange < (float)VLSetMinView)
        {
            AimVLRange = VLSetMinView;
        }

        if (fabs(AimVLRange - cur_vl_range) >= 0.5)
        {
            CalcFocus = UTL_Focus_Calc(IRPixel,IRPNum_H,AimVLRange);
            if (CalcFocus <= cali_vlfocus[LARGE_VIEW_FOCUS])
            {
                CalcFocus = cali_vlfocus[LARGE_VIEW_FOCUS];
            }
            else if (CalcFocus >= cali_vlfocus[MIN_VIEW_FOCUS])
            {
                CalcFocus = cali_vlfocus[MIN_VIEW_FOCUS];
            }
        }
    }
    //算完之后，就要返回，去控制红外和可见光了。
    return CalcFocus;
}
/*==============================================================
*FUNCTION NAME:系统性周期性处理
*DISCRIPTION:对于需要一直监控状态变化，并控制相关变量的处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_System_Handle(void)
{
    int16_t slew_data_ori;
    int16_t slew_data_pitch;
    bool VFollowMove_sta = false;
    bool GEOMove_sta = false;
//	SF_SYS_MODE_ENUM sf_mode;
    ATTITUDE_PAPA_T slew_data = {0};
    SFSaveAngle_TypeDef sf_cfg_info = {0};
    MASTER_CONFIG_INFO_T m_sys_info;

    slew_data = CONFIG_Get_Sf_Attitude();
#if 0
    /*当前主视频视场角，用于控制(直接用可见光或者红外的视场角即可)*/
    if (CONFIG_Get_Main_Video() == MainVideo_VL)
    {
        EO_Para.MianView_H = EO_state.vl_info.VLRange_H;
        EO_Para.MianView_V = EO_state.vl_info.VLRange_V;
    }
    else
    {
        EO_Para.MianView_H = EO_state.ir_info.IRRange_H;
        EO_Para.MianView_V = EO_state.ir_info.IRRange_V;
    }
#endif
    /*视场匹配*/


    /*微调的逻辑实现*/
    if (CONFIG_Get_Adjust_Sta() != ADJUST_NULL)
    {
        if ((m_sys_info.sys_work_mode == Sys_Track)
                || (m_sys_info.sys_work_mode == Sys_Serch)
                || (CONFIG_Get_Axis_Sta())
                || (CONFIG_Get_Decenter_Sta())) /*跟踪调波门/十字/偏心*/
        {
            CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_TRACK);//调整跟踪器
        }
        else if (m_sys_info.sys_work_mode == Sys_Vfollow) // 速率调位置
        {
            VFollowMove_sta = 1;
            CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_SF);
        }
        else if (m_sys_info.sys_work_mode == Sys_SLA) // 随动调位置
        {
            GEOMove_sta = 1;
            CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_SF);
        }
    }

    /*单杆微调*/
    if (VFollowMove_sta)
    {
        switch(CONFIG_Get_Adjust_Sta())
        {
        case ADJUST_UP:
            slew_data.azimuth_para = VFollowMove_STEP;
            break;
        case ADJUST_DOWN:
            slew_data.azimuth_para = 0 - VFollowMove_STEP;
            break;
        case ADJUST_RIGHT:
            slew_data.pitch_para = VFollowMove_STEP;
            break;
        case ADJUST_LEFT:
            slew_data.pitch_para = 0 - VFollowMove_STEP;
            break;
        default:
            break;
        }
    }

    /*随动微调*/
    if (GEOMove_sta)
    {
        switch (CONFIG_Get_Adjust_Sta())
        {
        case ADJUST_UP:
            sf_cfg_info.geo_fy_adj = (int16_t)sf_cfg_info.geo_fy_adj + m_sys_info.GEOMove_STEP * 100;
            break;
        case ADJUST_DOWN:
            sf_cfg_info.geo_fy_adj = (int16_t)sf_cfg_info.geo_fy_adj - m_sys_info.GEOMove_STEP * 100;
            break;
        case ADJUST_RIGHT:
            sf_cfg_info.geo_fw_adj = (int16_t)sf_cfg_info.geo_fw_adj + m_sys_info.GEOMove_STEP * 100;
            break;
        case ADJUST_LEFT:
            sf_cfg_info.geo_fw_adj = (int16_t)sf_cfg_info.geo_fw_adj - m_sys_info.GEOMove_STEP * 100;
            break;
        default:
            break;
        }
    }
    /*杆量根据传感器和视场角计算出目的速度*/
    //User_Cmd.SlewData_A:视场角对应的弧度，算出伺服转向，round:四舍五入函数
    if (m_sys_info.main_video == MainVideo_VL)
    {
        slew_data_ori = round(1.0f * m_sys_info.vl_info.VLRange_H); /*数值即为目标速度 °/s*/
        slew_data_pitch = round(1.0f * m_sys_info.vl_info.VLRange_H);
    }
    else
    {
        slew_data_ori = round(1.0f * m_sys_info.ir_info.IRRange_H); /*数值即为目标速度 °/s*/
        slew_data_pitch = round(1.0f * m_sys_info.ir_info.IRRange_V);
    }

    /*比例转换 比例从何而来待明确*/
    slew_data_ori = slew_data_ori * (slew_data.azimuth_para * 0.008f);
    slew_data_pitch = slew_data_pitch * (slew_data.pitch_para * 0.008f);

    //极值限定
    if (slew_data_ori > SF_SLEW_MAX)
    {
        slew_data_ori = SF_SLEW_MAX;
    }
    else if (slew_data_ori < SF_SLEW_MIN)
    {
        slew_data_ori =SF_SLEW_MIN;
    }

    if (slew_data_pitch > SF_SLEW_MAX)
    {
        slew_data_pitch = SF_SLEW_MAX;
    }
    else if (slew_data_pitch < SF_SLEW_MIN)
    {
        slew_data_pitch = SF_SLEW_MIN;
    }

//速率模式，单杆控制
//扫描模式控制俯仰
    if (m_sys_info.sys_work_mode == Sys_Scan)
    {
        //线性补偿开
        if (CONFIG_Get_TDC_Sta())
        {
            if (!CONFIG_Get_Plane_Data_Sta())
            {
                LMC.Fw = 0;
                LMC.Fy = 0;
            }
            slew_data_ori = (int16_t)(slew_data_ori * 1.0 + LMC.Fy * 336.8421f);
        }
    }
    else if (m_sys_info.sys_work_mode == Sys_Vfollow)
    {
        //温漂补偿打开
        if (CONFIG_Get_TDC_Sta())
        {
            slew_data_ori = 0;
            slew_data_pitch = 0;
        }
        //线性运动补偿打开
        else if (m_sys_info.LMCSta == 1)
        {
            if (!CONFIG_Get_Plane_Data_Sta())
            {
                LMC.Fw = 0;
                LMC.Fy = 0;
            }
            slew_data_ori = (int16_t)(slew_data_ori * 1.0 + LMC.Fw * 256.0f);
            slew_data_pitch = ( int16_t)(slew_data_pitch * 1.0 + LMC.Fy * 336.8421f);
        }
        //得到传给伺服的两个参数
        //SFCtrl_Data.FW_para1 = main_info.sf_slew_a;
        //SFCtrl_Data.FY_para1 = main_info.sf_slew_a;
        //该数据直接影响到方位控制参数，俯仰控制参数
//		main_info.sf_attitude.pitch_para = main_info.sf_slew_e;
//		main_info.sf_attitude.azimuth_para = main_info.sf_slew_a;
    }
    slew_data.azimuth_para = slew_data_ori;
    slew_data.pitch_para = slew_data_pitch;
    CONFIG_Set_Sf_Attitude(slew_data);

}
/*==============================================================
*FUNCTION NAME:伺服周期控制
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_SF_Handle(void)
{
    uint8_t sf_send_cmd = 0;
    SYS_SF_DATA_T sys_sf_info = {0};
    ATTITUDE_PAPA_T sf_data = {0};
    uint8_t recv_cmd = 0;
    MASTER_CTRL_TYPE_ENUM set_ctrl_type = MASTER_CTRL_NULL;
    uint8_t sf_mode = {0};
#if 0
    /*伺服上电自检*/
    if ((User_Cmd.SF_Initial) && (HAL_GetTick() > 3000))
    {
        User_Cmd.SF_Initial = 0;
        User_Cmd.SF_CtrlCmd = SF_Zero;
        User_Cmd.SF_CtrlSta = P_Zero;
    }
#endif
    /*5ms发送一次*/
    if (User_Tick.SF < 5)
    {
        return;
    }
    else
    {
        User_Tick.SF = 0;
    }
    recv_cmd = CONFIG_Get_Master_Ctrl_Cmd();
    if (!APP_IS_BIT_SET(CONFIG_Get_Master_Ctrl_Sta(),MASTER_SF_PARA))//判断该位是否置高，不置高则不是跟踪器指令，就周期性发送无指令的报文
    {
        //如果没有控制指令，则发空指令
        recv_cmd = MASTER_CMD_NULL;
    }
    switch (recv_cmd)
    {
    case MASTER_CMD_NULL:

        break;
    case MASTER_SELF_CHECK: /*自检*/
        sf_send_cmd = SF_WITH_DRAW;
        break;
    case MASTER_PITCH_COMPENSATE_ADD: /*俯仰漂移补偿 +*/
        sf_send_cmd = SF_TEMP_DRIFT_UP;
        break;
    case MASTER_PITCH_COMPENSATE_MINUS: /*俯仰漂移补偿－*/
        sf_send_cmd = SF_TEMP_DRIFT_DOWN;
        break;
    case MASTER_YAW_COMPENSATE_ADD: /*方位漂移补偿 +*/
        sf_send_cmd = SF_TEMP_DRIFT_RIGHT;
        break;
    case MASTER_YAW_COMPENSATE_MINUS: /*方位漂移补偿－*/
        sf_send_cmd = SF_TEMP_DRIFT_LEFT;
        break;
    case MASTER_COMPENSATE_AUTO_RESET: /*漂移补偿自动/恢复出厂设置*/
        sf_send_cmd = SF_TEMP_DRIFT_RESET;
        break;
    case MASTER_COLLECT: /*收藏*/
        sf_send_cmd = SF_WITH_DRAW;
        //User_Cmd.SF_CtrlSta = P_Withdraw;
        break;
    case MASTER_TRACE: /*跟踪*/
        sf_send_cmd = SF_TRACK_MODE;
        //User_Cmd.SF_CtrlSta = P_Track;
        //User_Cmd.MC_TO_TraCtrl = FKCtrl_Cmd;
        //EO_Para.SysTrack_mode = 0;
        break;
    case MASTER_LOCKED_CUR: /*锁定当前*/
        sf_send_cmd = SF_LOCK_BMQ;
        //User_Cmd.SF_CtrlSta = P_LockBMQ;
        sys_sf_info.FW_Combine = UTL_Normalize_Angle(sys_sf_info.FW_Combine);
        sf_data.azimuth_para = (int16_t)(sys_sf_info.FW_Combine * 100);
        sf_data.pitch_para = (int16_t)(sys_sf_info.FY_Combine * 100);
        CONFIG_Set_Sf_Attitude(sf_data);
        break;
    case MASTER_LOCKED: /*锁定*/
        sf_send_cmd = SF_LOCK_BMQ;
        //User_Cmd.SF_CtrlSta = P_LockBMQ;
        //TempData = (int16_t)(User_Cmd.SF_CtrlData[0] | User_Cmd.SF_CtrlData[1] << 8);
        //if (TempData > 180)
        //	TempData = TempData - 360;
        //SFCtrl_Data.FW_para1 = (int16_t)(TempData * 100);

        //TempData = (int16_t)(User_Cmd.SF_CtrlData[2] | User_Cmd.SF_CtrlData[3] << 8);
        //SFCtrl_Data.FY_para1 = (int16_t)(TempData * 100);
        break;
    case MASTER_SCANNING: /*扫描*/
        sf_send_cmd = SF_SCAN;
        //		User_Cmd.SF_CtrlSta = P_Scan;
        //		TempData = (int16_t)(User_Cmd.SF_CtrlData[0] | User_Cmd.SF_CtrlData[1] << 8);
        //		if (TempData > 180)
        //			TempData = TempData - 360;
        //		SFCtrl_Data.FW_para1 = (int16_t)(TempData * 100);
        //		TempData = (uint16_t)((User_Cmd.SF_CtrlData[2] | User_Cmd.SF_CtrlData[3] << 8) * 0.006);
        //		SFCtrl_Data.FW_para2 = (uint16_t)(TempData * 100);
        break;
    case MASTER_SERVO: /*随动*/
        sf_send_cmd = SF_V_FOLLOW;
        //		if (EO_state.plane_Data.destnation != DEST_EO)
        {
            //			EO_Para.GEO_origin_flag = 1;
            //			sf_send_cmd = SF_GEO_FK;
            //			User_Cmd.SF_CtrlSta = P_GEO_FK;
        }
        break;
    case MASTER_PLOT_DETECT: /*小区搜索*/

        break;
    case MASTER_SPEED_HANDLE: /*速率（手动）*/
        sf_send_cmd = SF_V_FOLLOW;
        //		User_Cmd.SF_CtrlSta = P_Vfollow;
        break;
    case MASTER_TRACE_GEOGRAPHY: /*地理跟踪*/
        sf_send_cmd = SF_GEO;
        //		EO_Para.GEO_origin_flag = 0;
        //		User_Cmd.SF_CtrlSta = P_GEO_FK;
        break;
    case MASTER_DETECT_TRACE: /*搜索跟踪*/
        sf_send_cmd = SF_TRACK_MODE;
        //		User_Cmd.SF_CtrlSta = P_Track;
        break;
    case MASTER_ROAM: /*漫游引导*/
        sf_send_cmd = SF_GUIDE;
        CONFIG_Set_LMC_Sta(true);
        //		User_Cmd.RoamGuide_sta = 1;
        //		User_Cmd.SF_CtrlSta = P_Guide;
        //		TempData = (EOCtrl_Data.slew_data_A * 0.00003125) * EO_Para.MianView_H;
        //		TempData = TempData * 0.5f + EO_state.sf_info.FW_Combine;
        //		if (TempData > 180)
        //			TempData = TempData - 360;
        //		SFCtrl_Data.FW_para1 = (int16_t)(TempData * 100);
        //		TempData = (EOCtrl_Data.slew_data_E * 0.00003125) * EO_Para.MianView_V;
        //		TempData = TempData * 0.5f + EO_state.sf_info.FY_Combine;
        //		SFCtrl_Data.FY_para1 = (int16_t)(TempData * 100);
        break;

    /*内部调试指令*/
    case MASTER_BREAK: /*刹车*/
        //		sf_send_cmd = SF_Brake;
        //		User_Cmd.SF_CtrlSta = P_Brake;
        break;
    default:
        break;
    }
    CONFIG_Set_SF_Mode((SF_SYS_MODE_ENUM)sf_mode);
    SF_API_Ctrl_SendHandle(sf_send_cmd,0);
    //发完报文后置低位
    APP_CLEAR_BIT(set_ctrl_type,MASTER_TRACK_PARA);
    CONFIG_Set_Master_Ctrl_Sta(set_ctrl_type);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:跟踪器周期性控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Track_Handle(void)
{
    uint8_t track_cmd = 0;
    SYS_TRACK_DATA_T track_parse_info;
    uint8_t send_buf = 0;
    uint8_t recv_cmd = 0;
    MASTER_CTRL_TYPE_ENUM master_ctrl_type = MASTER_CTRL_NULL;
    /*40ms发送一次*/
    if (User_Tick.TrackCtrl < 40)
    {
        return;
    }
    else
    {
        User_Tick.TrackCtrl = 0;
    }
    recv_cmd = CONFIG_Get_Master_Ctrl_Cmd();
    master_ctrl_type = CONFIG_Get_Master_Ctrl_Sta();
    if (!APP_IS_BIT_SET(master_ctrl_type,3))//判断该位是否置高，不置高则不是跟踪器指令，就周期性发送无指令的报文
    {
        recv_cmd = MASTER_CMD_NULL;
    }
    switch (recv_cmd)
    {
    case MASTER_CMD_NULL: /*无动作*/
        track_cmd = TRA_CMD_INVALID;
#if 0
        if (User_Cmd.TraSerch_flg)
        {
            User_Cmd.TraCtrl_Cmd = Tra_gzqr;//跟踪确认
            User_Cmd.TraSerch_flg = 0;
        }
#endif
        break;

    case MASTER_SELF_CHECK: /*自检*/
        track_cmd = TRA_ETH_VIDEO1_ON;
        break;
    case MASTER_TRACE: /*跟踪*/
        track_cmd = TRA_WITH_CROSSHAIR;
        break;
    case MASTER_SENSOR_TV: /*传感器选择-TV*/
        //选主视频
        if ((track_parse_info.GZQStatus.VideoDisplayMode != TRA_MODE_VL)
                || (track_parse_info.GZQStatus.VideoDisplayMode != TRA_MODE_IR))
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_VL_IR;
        }
        else
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_VL;
        }
        break;
    case MASTER_SENSOR_TR: /*传感器选择-IR*/
        if ((track_parse_info.GZQStatus.VideoDisplayMode != TRA_MODE_VL)
                || (track_parse_info.GZQStatus.VideoDisplayMode != TRA_MODE_IR))
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_VL_IR;
        }
        else
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_IR;
        }
        break;
#if 1
    case MASTER_AXIS_ADJUST_UP: /*微调上*/
        if (CONFIG_Get_Decenter_Sta())
        {
            track_cmd = TRA_DECENTER_UP;
        }
        else if (CONFIG_Get_Axis_Sta())
        {
            track_cmd = TRA_CROSSHAIR_UP;
        }
        else
        {
            track_cmd = TRA_WAVE_GATE_UP;
        }
        break;
    case MASTER_AXIS_ADJUST_DOWN: /*微调下*/
        if (CONFIG_Get_Decenter_Sta())
        {
            track_cmd =  TRA_DECENTER_DOWN;
        }
        else if (CONFIG_Get_Axis_Sta())
        {
            track_cmd = TRA_CROSSHAIR_DOWN;
        }
        else
        {
            track_cmd = TRA_WAVE_GATE_DOWN;
        }
        break;
    case MASTER_AXIS_ADJUST_LEFT: /*微调左*/
        if (CONFIG_Get_Decenter_Sta())
        {
            track_cmd =  TRA_DECENTER_LEFT;
        }
        else if (CONFIG_Get_Axis_Sta())
        {
            track_cmd = TRA_CROSSHAIR_LEFT;
        }
        else
        {
            track_cmd = TRA_WAVE_GATE_LEFT;
        }
        break;
    case MASTER_AXIS_ADJUST_RIGHT: /*微调右*/
        if (CONFIG_Get_Decenter_Sta())
        {
            track_cmd =  TRA_DECENTER_RIGHT;
        }
        else if (CONFIG_Get_Axis_Sta())
        {
            track_cmd = TRA_CROSSHAIR_RIGHT;
        }
        else
        {
            track_cmd = TRA_WAVE_GATE_RIGHT;
        }
        break;
    case MASTER_BORESIGHT_ON: /*空中校轴开boresight*/

//		EO_Para.Axis_sta = 1;
        break;

    case MASTER_AXIS_WITH_SAVE: /*退出校轴并保存*/
        track_cmd = TRA_CROSSHAIR_SAVE;
        break;

    case MASTER_AXIS_WITHOUT_SAVE: /*退出校轴不保存*/

        track_cmd = TRA_CROSSHAIR_RESET;
        break;
#if 0
    case MASTER_FANGYONG_LEVEL_1: /*一级防拥*/
#if UNITY_TRACK
        TraCtrl_data.Osd.Ctrl.PP = 1;
        TraCtrl_data.Osd.Ctrl.WP = 1;
        TraCtrl_data.Osd.Ctrl.LP = 1;
#endif
        break;
    case MASTER_FANGYONG_LEVEL_2: /*二级防拥*/
#if UNITY_TRACK
        *(uint32_t *)&TraCtrl_data.Osd.Ctrl = 0xFFFFFFFF;
        TraCtrl_data.Osd.Ctrl.Cross = 0;
        TraCtrl_data.Osd.Ctrl.TraGate = 0;
        TraCtrl_data.Osd.Ctrl.IdenGate = 0;
#endif
        break;
    case MASTER_FANGYONG_OFF: /*防拥关闭*/
#if UNITY_TRACK
        *(uint32_t *)&TraCtrl_data.Osd.Ctrl = 0x00000000;
#endif
        break;
    case MASTER_FANGYONG_ALL: /*全防拥*/
#if UNITY_TRACK
        *(uint32_t *)&TraCtrl_data.Osd.Ctrl = 0xFFFFFFFF;
#endif
        break;
#endif
    case MASTER_P_IN_P_ON: /*画中画开启*/
        if (CONFIG_Get_Main_Video() == MainVideo_VL)
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_VL_IR;
        }
        else if ((CONFIG_Get_Main_Video() == MainVideo_MWIR) || (CONFIG_Get_Main_Video() == MainVideo_LWIR))
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_IR_VL;
        }
        break;
    case MASTER_P_IN_P_OFF: /*画中画关闭*/
        if (CONFIG_Get_Main_Video() == MainVideo_VL)
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_VL;
        }
        else if ((CONFIG_Get_Main_Video() == MainVideo_MWIR) || (CONFIG_Get_Main_Video() == MainVideo_LWIR))
        {
            track_cmd = TRA_VIDEO_MODE;
            send_buf = TRA_MODE_IR;
        }
        break;
    case MASTER_MULTI_DEST_ON:		/*多目标跟踪开启*/
//		User_Cmd.TraCtrl_Cmd = Tra_dzstable_on; // 临时
        break;
    case MASTER_MULTI_DEST_OFF:		/*多目标跟踪关闭*/
//		User_Cmd.TraCtrl_Cmd = Tra_dzstable_off; // 临时
        break;
    case MASTER_MULTI_DEST_ID: /*多目标编号选择*/
        track_cmd = TRA_WITH_ID;
        switch (track_parse_info.track_id)
        {
        case 0x13:
            send_buf = 1;
            break;
        case 0x15:
            send_buf = 2;
            break;
        case 0x16:
            send_buf = 3;
            break;
        case 0x19:
            send_buf = 4;
            break;
        case 0x1A:
            send_buf = 5;
            break;
        case 0x1C:
            send_buf = 6;
            break;
        case 0x23:
            send_buf = 7;
            break;
        case 0x25:
            send_buf = 8;
            break;
        case 0x27:
            send_buf = 9;
            break;
        case 0x2A:
            send_buf = 10;
            break;
        default:
            break;
        }
        break;
    case MASTER_EO_TXTP: /*EO视频字符颜色/显示单位切换（TXTP）*/
        switch (track_parse_info.video_color)
        {
        case 0x13:
            send_buf = TRA_OSD_COLOR_WHITE;
            break;
        case 0x19:
            send_buf = TRA_OSD_COLOR_RED;
            break;
        case 0x1A:
            send_buf = TRA_OSD_COLOR_BLUE;
            break;
        case 0x23: /*度分秒显示*/
            send_buf = TRA_OSD_SHOE_DMS;
            break;
        case 0x25: /*度显示*/
            send_buf = TRA_OSD_SHOE_D;
            break;
        default:
            break;
        }
        break;
    case MASTER_OFFSET_AIMING_ON: /*偏置瞄准开*/
//		EO_Para.Traoffcenter_flg = 1;
        break;
    case MASTER_OFFSET_AIMING_OFF: /*偏置瞄准关*/
//		EO_Para.Traoffcenter_flg = 0;
//		track_cmd = TRA_WITH_ID; = Traoff_clear;
        break;
    case MASTER_TRACE_METHOD_AUTO: /*跟踪方式-自动*/
        track_cmd = TRA_MODE_PTRA;
        break;
    case MASTER_TRACE_METHOD: /*跟踪方式-场景*/
        track_cmd = TRA_MODE_STRA;
        break;
    case MASTER_DETECT_TRACE: /*搜索跟踪*/
//		User_Cmd.TraCtrl_Cmd = Tra_gzdxz;
        if (CONFIG_Get_Main_Video() == MainVideo_VL)
        {
//			User_Cmd.TraCtrl_Para1 = (((int16_t)(User_Cmd.MC_TO_TraPara[0] | User_Cmd.MC_TO_TraPara[1] << 8)) * 0.000015625) * VLPNum_H;
//			User_Cmd.TraCtrl_Para2 = (((int16_t)(User_Cmd.MC_TO_TraPara[2] | User_Cmd.MC_TO_TraPara[3] << 8)) * 0.000015625) * VLPNum_V;
        }
        else if ((CONFIG_Get_Main_Video()== MainVideo_MWIR) || (CONFIG_Get_Main_Video() == MainVideo_LWIR))
        {
//			User_Cmd.TraCtrl_Para1 = (((int16_t)(User_Cmd.MC_TO_TraPara[0] | User_Cmd.MC_TO_TraPara[1] << 8)) * 0.000015625) * VLPNum_H; //* IRPNum_H;
//			User_Cmd.TraCtrl_Para2 = (((int16_t)(User_Cmd.MC_TO_TraPara[2] | User_Cmd.MC_TO_TraPara[3] << 8)) * 0.000015625) * VLPNum_V; //* IRPNum_V;
        }
        break;
    default:
        CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
        return;
    }
#endif
    //隔40ms的周期发送报文
    TRACK_API_Ctrl_SendHandle(track_cmd,(uint8_t *)&send_buf);
    //发完报文后置低位
    APP_CLEAR_BIT(master_ctrl_type,MASTER_TRACK_PARA);
    CONFIG_Set_Master_Ctrl_Sta(master_ctrl_type);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光轮询发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void LaserCtrl_Loop_Handle(void)
{
    if (User_Tick.LaserCtrl >= 100)
    {
        /*每隔1s查询激光的版本号，以确定激光正常运行*/
        User_Tick.LaserCtrl = 0;
#if LASER_INCLUDE
        Laser_Ctrl_SendHandle(LZ_SINGLE_DETECT, 0, 0);
#endif
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:激光周期性控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Laser_Handle(void)
{
    uint8_t lz_cmd_type;
    uint8_t lz_ctrl_data;
    uint8_t lz_code;
    //通过上位机下发的指令，切换激光相关参数
    SYS_LASER_STA_T laser_info_sta;

#if 0
    //激光控制指令执行条件
    /*地面状态，激光不工作*/
    if (EO_Status.eo_para.wheel_sta == PLANEGROUND)
    {
        return;
    }

    /*在非跟踪状态时，不打激光*/
    if ((main_info.sys_mode != Sys_Track) && (main_info.sys_mode != Sys_Serch))
    {
        return;
    }

    /******激光在内框架0.8°内停止工作 Start******/
    if (User_Cmd.MC_TO_LZCtrl)
    {
        EO_Para.LZ_ProtectSta = 0;
        User_Cmd.LZOld_WorkSta = 0;
        User_Tick.LZ_ProtectTime = HAL_GetTick();
    }
#endif
    /*周期性的激光控制的处理*/
    LaserCtrl_Loop_Handle();

    //该函数周期性执行，检测到Master_Ctrl发生变化则立即执行
    if (CONFIG_Get_Master_Ctrl_Sta() != MASTER_CTRL_LASER)
    {
        return;
    }

    switch(CONFIG_Get_Master_Ctrl_Cmd())
    {
    case MASTER_LASER_POWER_ON: /*激光器上电*/
        LaserPower_On;
        laser_info_sta.LZ_Powersta = 1;
        break;
    case MASTER_LASER_POWER_OFF: /*激光器下电*/
        LaserPower_Off;
        laser_info_sta.LZ_Powersta = 0;
        break;

    case MASTER_LASER_DETECT_SIGLE: /*激光单次测距*/
        lz_cmd_type = LZ_SINGLE_DETECT;
        break;

    case MASTER_LASER_DETECT_5HZ: /*激光连续测距（5HZ，时间待定）*/
        lz_cmd_type = LZ_DETECT_FREQ_5;
        break;

    case MASTER_LASER_LIGHT: /*激光照射*/
        lz_cmd_type = LZ_DETECT_PRECISION;
        //main_info.LZCtrl_Para2 = main_info;//.LazNum;
        //User_Cmd.LZCtrl_Para1 = 20;
        break;
    case MASTER_LASER_STOP: /*激光测距/照射停*/
        lz_cmd_type = LZ_DETEC_STOP;
        break;
    case MASTER_LASER_ENABLE: /*激光使能开(接收)*/
        lz_cmd_type = LZ_RX_STA;
        lz_ctrl_data = CMD_ENABLE;
        break;
    case MASTER_LASER_DISABLE: /*激光使能关(接收)*/
        lz_cmd_type = LZ_RX_STA;
        lz_ctrl_data = CMD_DISABLE;
        break;

    case MASTER_CODE_CHOOSE: /*激光编码选择*/
        lz_cmd_type = LZ_DETECT_PRECISION;
        lz_ctrl_data = laser_info_sta.codenum;//激光编码赋值
        switch (lz_ctrl_data)
        {
        case 0x13:
            lz_code = 1;
            break;
        case 0x15:
            lz_code = 2;
            break;
        case 0x16:
            lz_code = 3;
            break;
        case 0x19:
            lz_code = 4;
            break;
        case 0x1A:
            lz_code = 5;
            break;
        case 0x1C:
            lz_code = 6;
            break;
        case 0x23:
            lz_code = 7;
            break;
        case 0x25:
            lz_code = 8;
            break;
        case 0x26:
            lz_code = 9;
            break;
        case 0x29:
            lz_code = 10;
            break;
        case 0x2A:
            lz_code = 11;
            break;
        case 0x2C:
            lz_code = 12;
            break;
        case 0x31:
            lz_code= 13;
            break;
        case 0x32:
            lz_code = 14;
            break;
        case 0x34:
            lz_code = 15;
            break;
        case 0x37:
            lz_code = 16;
            break;
        default:
            break;
        }
        break;
    default:
        //都不符合，则状态清空,清空后返回
        CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
        return;
    }
    //cmd_para1:使能或者选通值等通用参数 cmd_para2:仅用于放照射序列码
#if LASER_INCLUDE
    Laser_Ctrl_SendHandle((SYS_LASER_DETECT_MODE)lz_cmd_type, lz_ctrl_data, lz_code);
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外热像仪初始化配置
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Init_Step_By_step(void)
{
    //上电后初始化配置
    static uint8_t ir_init_step = PERIGHERALS_INIT_WAIT;
    uint8_t send_type;
    uint8_t temp_buf;
    switch (ir_init_step)
    {
    case PERIGHERALS_INIT_WAIT: /*对比度设置*/
        send_type = IR_SET_CONTRAST;
        temp_buf = 20;
        ir_init_step = PERIGHERALS_INIT_STEP1;
        break;
    case PERIGHERALS_INIT_STEP1: /*亮度设置*/
        send_type = IR_SET_LIGHT;
        temp_buf = 76;
        ir_init_step = PERIGHERALS_INIT_STEP2;
        break;
    case PERIGHERALS_INIT_STEP2: /*进入图像增强*/
        send_type = IR_IMAGE_ENHANCE;
        temp_buf = CMD_ENABLE;
        ir_init_step = PERIGHERALS_INIT_END;
        break;
    default :
        ir_init_step = PERIGHERALS_INIT_END;
        break;
    }
    Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, send_type, (uint8_t *)&temp_buf);
    return ir_init_step;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外热像仪周期性发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IRCtrl_Loop_Handle(void)
{
    if (User_Tick.IRCtrl >= 1000)
    {
        /*每隔1s查询红外的系统状态，以确定红外正常运行*/
        User_Tick.IRCtrl = 0;
        Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, IR_SYS_STA_REQ, 0);
        //如过存在另一款型号的红外，直接将发送函数加载下面即可
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外热像仪周期性发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_Inner_Handle(void)
{
    uint8_t gd_ctrl_cmd = 0;
    if (User_Tick.InnerGD >= 20)
    {
        /*定周期发送报文*/
        User_Tick.InnerGD = 0;
        /*内惯导对准*/
        gd_ctrl_cmd = GD_Inner_Calibrate();
        Inside_gd_process_data_send(gd_ctrl_cmd);
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:红外周期性控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_IR_Handle(void)
{
    SYS_IR_CMD_CTRL send_type = IR_SYS_STA_REQ;
    SYS_IR_STA_T main_ir_info;
    uint32_t temp_buf;
    float cali_irfocus[VIEW_FOCUS_END];

    //获取标定的焦距值
    CONFIG_Get_Cali_irFocus(cali_irfocus);

    //上电初始化函数
    if (PERIGHERALS_INIT_END != IR_Init_Step_By_step())
    {
        return;
    }
    //周期轮询函数
    IRCtrl_Loop_Handle();

    //如果不是红外相机的处理，则直接退出该函数
    if (CONFIG_Get_Master_Ctrl_Sta() != MASTER_CTRL_IR)
    {
        return;
    }
    main_ir_info = CONFIG_Get_Ir_Info();
    switch (CONFIG_Get_Master_Ctrl_Cmd())
    {
    case MASTER_FOCUS_ADD: /*调焦加*/
        send_type = IR_FOCUS_STEP_ADD;
        break;
    case MASTER_FOCUS_MINUS: /*调焦减*/
        send_type = IR_FOCUS_STEP_MINUS;
        break;
    case MASTER_FOCUS_AUTO: /*变焦*/
        send_type = IR_ZOOM_POSITION;
        if (main_ir_info.ir_focus_value != 0)
        {
            temp_buf = IRMinZoom + ((main_ir_info.ir_focus_value * (IRMaxZoom - IRMinZoom)) / 255);
        }
        break;
    case MASTER_VIEW_LARGE: /*视场切换-大视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[LARGE_VIEW_FOCUS];
        break;
    case MASTER_VIEW_MID: /*视场切换-中视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[MID_VIEW_FOCUS];
        break;
    case MASTER_VIEW_SMALL: /*视场切换-小视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
        break;
    case MASTER_VIEW_SSMALL: /*视场切换-极小视场*/
        send_type = IR_ZOOM_POSITION;
        temp_buf = cali_irfocus[MIN_VIEW_FOCUS];
        break;
    case MASTER_ZOOM_IN:			/*ZOOM IN*/
        switch (main_ir_info.irview_Now) // 放大
        {
        case 1:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MID_VIEW_FOCUS];
            break;
        case 2:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
            break;
        case 3:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MIN_VIEW_FOCUS];
            break;
        default:
            break;
        }
        break;
    case MASTER_ZOOM_OUT:			/*ZOOM OUT*/
        switch (main_ir_info.irview_Now) // 放大
        {
        case 4:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[SMA_VIEW_FOCUS];
            break;
        case 3:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[MID_VIEW_FOCUS];
            break;
        case 2:
            send_type = IR_ZOOM_POSITION;
            temp_buf = cali_irfocus[LARGE_VIEW_FOCUS];
            break;
        default:
            break;
        }
        break;
    case MASTER_VIEW_ELECTRIC: /*电子变倍*/
        send_type = IR_ZOOM_DIT;
        switch (main_ir_info.elezoom)
        {
        case 0:
        case 1:
            temp_buf = 2;
            break;
        case 2:
        case 3:
            temp_buf = 4;
            break;
        case 4:
            temp_buf = 0;
            break;
        default:
            break;
        }
        break;
    case MASTER_PICTURE_AUTO: /*图像自动调节*/
        // User_Cmd.IRCtrl_type = MASTER_PICTURE_AUTO;
        //ir_init_flag = 1;
        break;

    case MASTER_GAIN_ADD: /*增益＋*/
        send_type = IR_SET_CONTRAST;
        if (main_ir_info.contrast_level < 255 - main_ir_info.contrast_step)
        {
            temp_buf = main_ir_info.contrast_level + main_ir_info.contrast_step;
        }
        else
        {
            temp_buf = 255;
        }
        break;
    case MASTER_GAIN_MINUS: /*增益-*/
        send_type = IR_SET_CONTRAST;
        if (main_ir_info.contrast_level > IRCONTRST_STEP)
            temp_buf = main_ir_info.contrast_level - IRCONTRST_STEP;
        else
            temp_buf = 0;
        break;
    case MASTER_LIGHT_ADD: /*亮度＋*/
        send_type = IR_SET_LIGHT;
        if (main_ir_info.light_level < 255 - IRLIGHT_STEP)
            temp_buf = main_ir_info.light_level + IRLIGHT_STEP;
        else
            temp_buf = 255;
        break;
    case MASTER_LIGHT_MINUS: /*亮度-*/
        send_type = IR_SET_LIGHT;
        if (main_ir_info.light_level > IRLIGHT_STEP)
            temp_buf = main_ir_info.light_level - IRLIGHT_STEP;
        else
            temp_buf= 0;
        break;
    case MASTER_FOCUS_ON_AUTO: /*自动聚焦*/
        send_type = IR_FOCUS_AUTO;
        break;
#if 0
    case MASTER_PICTURE_ENHANCE_ON: /*进入图象增强*/
        User_Cmd.IRCtrl_Cmd = IR_IMAGE_ENHANCE;
        User_Cmd.IRCtrl_Para1 = CMD_ENABLE;
        break;
    case MASTER_PICTURE_ENHANCE_OFF: /*退出图象增强*/
        User_Cmd.IRCtrl_Cmd = IR_IMAGE_ENHANCE;
        User_Cmd.IRCtrl_Para1 = CMD_DISABLE;
        break;
#endif
    case MASTER_PICTURE_ENHANCE_ON: /*进入图象增强 实质是DDE调整*/
        send_type = IR_DDE_RANGE;
        if (main_ir_info.DDE_level == 30)
            temp_buf = 100;
        else
            temp_buf = main_ir_info.DDE_level + IRDDE_STEP;
        //temp_buf = CHECK_RESET(temp_buf,int32_t);
        CHECK_MAX(temp_buf,uint32_t);
        break;
    case MASTER_PICTURE_ENHANCE_OFF: /*退出图象增强 实质是DDE给30*/
        send_type = IR_DDE_RANGE;
        temp_buf = 30;
        break;
    case MASTER_HW_POWER_ON: /*红外上电*/
        IRPower_On;
        main_ir_info.IR_PowerSta = 1;
        break;
    case MASTER_HW_POWER_OFF: /*红外下电*/
        IRPower_Off;
        main_ir_info.IR_PowerSta = 0;
        break;
    case MASTER_HW_PICTURE_CALIBRATE: /*红外图像校准 虚焦校正*/
        send_type = IR_CALIBRATION;
        temp_buf = 3;
        break;
    case MASTER_HW_PICTURE_CHANGE: /*红外正负像切换*/
        send_type = IR_BLACK_WHITE;
        main_ir_info.ir_status.black_white = ~(main_ir_info.ir_status.black_white) & 0x01;
        temp_buf = main_ir_info.ir_status.black_white;
        break;

    case MASTER_VIEW_MATCH_ON:/*视场匹配*/
        if(CONFIG_Get_Main_Video() == MainVideo_VL)
        {
            send_type = IR_ZOOM_POSITION;
            temp_buf = (int32_t)USER_Ctrl_View_Match(main_ir_info.IRRange_H);
        }
        else
        {
            return;
        }
        break;

        /*内部测试用*/
#if 0
    case MASTER_IR_TESTCTRL: /*红外透传*/
        User_Cmd.IRCtrl_Cmd = IR_MAINTIAN_SET;
        memcpy(User_Cmd.IR_CtrlData, User_Cmd.Dev_CtrlData, 10);
        if (Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, User_Cmd.IRCtrl_Cmd, (uint8_t *)&User_Cmd.IR_CtrlData) == HAL_OK)
        {
            User_Cmd.IRCtrl_Cmd = NULL;
            User_Cmd.IRCtrl_Para1 = NULL;
        }
        break;
#endif
    default:
        //都不符合，则状态清空,清空后返回
        CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
        return;
    }
    Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A, send_type, (uint8_t *)&temp_buf);
    CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:可见光按步初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t VL_Init_Step_By_Step(void)
{
    static uint8_t vl_init_step = PERIGHERALS_INIT_WAIT;
    uint8_t send_type = 0;
    uint8_t send_para = 0;
    float cali_vlfocus[VIEW_FOCUS_END];

    //获取标定的焦距值
    CONFIG_Get_Cali_vlFocus(cali_vlfocus);

    switch (vl_init_step)
    {
    case PERIGHERALS_INIT_WAIT: /*亮度手动*/
        send_type = VL_LIGHT_HANDIE;
        vl_init_step = PERIGHERALS_INIT_STEP1;
        break;
    case PERIGHERALS_INIT_STEP1: /*大视场*/
        send_type = VL_ZOOM_POSITION;
        send_para = cali_vlfocus[LARGE_VIEW_FOCUS];//切换到大视场
        vl_init_step = PERIGHERALS_INIT_STEP1;
        break;
    case PERIGHERALS_INIT_STEP2: /*亮度设置*/
        send_type = VL_LIGHT_POSITION;
        send_para = 100;
        vl_init_step = PERIGHERALS_INIT_STEP1;
        break;
    case PERIGHERALS_INIT_STEP3: /*透雾一档*/
        send_type = VL_IMAGE_ENHANCE;
        send_para = 2;
        vl_init_step = PERIGHERALS_INIT_END;
        break;
    default:
        vl_init_step = PERIGHERALS_INIT_END;
        break;
    }
    Visible_API_Ctrl_SendHandle(VL_F15_300,send_type,(uint8_t *)&send_para);

    return vl_init_step;
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:可见光周期处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VLCtrl_Loop_Handle(void)
{
    if (User_Tick.VLCtrl >= 40)
    {
        /*每隔40ms查询可见光的版本号，以确定可见光正常运行*/
        User_Tick.VLCtrl = 0;
#if (VISIBLE_INCLUDE&VL_F15_300)
        Visible_API_Ctrl_SendHandle(VL_F15_300,VL_FOCUS_REQ,0);
#endif
#if (VISIBLE_INCLUDE&VL_VS2030)
        Visible_API_Ctrl_SendHandle(VL_VS2030,VL_FOCUS_REQ,0);
#endif
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:可见光循环控制
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void USER_Ctrl_VL_Handle(void)
{
    uint8_t vl_send_type;
    uint32_t vl_send_buf = 0;
    uint8_t opt_ctrl_type = 0;
    SYS_VISIBLE_DATA_T  sys_vl_info;
    float cali_vlfocus[VIEW_FOCUS_END];

    //获取标定的焦距值
    CONFIG_Get_Cali_vlFocus(cali_vlfocus);
    //上电初始化
    if (PERIGHERALS_INIT_END != VL_Init_Step_By_Step())
    {
        return;
    }
    //初始化之后，周期发送查询报文，检查可将光相机的状态
    VLCtrl_Loop_Handle();
    //如果不是可见光相机的处理，则直接退出该函数
    if (CONFIG_Get_Master_Ctrl_Sta() != MASTER_CTRL_VL)
    {
        return;
    }
    //获取当前可见光参数
    sys_vl_info = CONFIG_Get_Visible_Info();

    if (opt_ctrl_type == MASTER_CTRL_VL)
    {
        switch (CONFIG_Get_Master_Ctrl_Cmd())
        {
        case MASTER_FOCUS_ADD: /*调焦-焦距加*/
            vl_send_type = VL_FOCUS_ADD;//步进在实现文件中已经明确，此处无需再赋值
            break;
        case MASTER_FOCUS_MINUS: /*调焦-焦距减*/
            vl_send_type = VL_FOCUS_MINUS;
            break;
        case MASTER_FOCUS_AUTO: /*变焦*/
            vl_send_type = VL_ZOOM_POSITION;
            if (sys_vl_info.focus_value)
            {
                vl_send_buf = (VLMinFocus + ((sys_vl_info.focus_value * (VLMaxFocus - VLMinFocus)) / 255));
            }
            break;
        case MASTER_VIEW_MATCH_ON:/*视场匹配*/
            if(CONFIG_Get_Main_Video() == MainVideo_MWIR)
            {
                vl_send_type = IR_ZOOM_POSITION;
                vl_send_buf = (int32_t)USER_Ctrl_View_Match(sys_vl_info.VLRange_H);
            }
            else
            {
                //如果当前主视频是可见光，则不调整可见光视场。
                return;
            }
            break;

        case MASTER_VIEW_LARGE: /*视场切换-大视场*/
            vl_send_type = VL_ZOOM_POSITION;
            vl_send_buf = cali_vlfocus[LARGE_VIEW_FOCUS];
            break;
        case MASTER_VIEW_MID: /*视场切换-中视场*/
            vl_send_type = VL_ZOOM_POSITION;
            vl_send_buf = cali_vlfocus[MID_VIEW_FOCUS];
            break;
        case MASTER_VIEW_SMALL: /*视场切换-小视场*/
            vl_send_type = VL_ZOOM_POSITION;
            vl_send_buf = cali_vlfocus[SMA_VIEW_FOCUS];
            break;
        case MASTER_VIEW_SSMALL: /*视场切换-极小视场*/
            vl_send_type = VL_ZOOM_POSITION;
            vl_send_buf = cali_vlfocus[MIN_VIEW_FOCUS];
            break;
        case MASTER_ZOOM_IN:			/*ZOOM IN*/
            switch (sys_vl_info.vlview_Now) // 放大
            {
            case 1:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[LARGE_VIEW_FOCUS];
                break;
            case 2:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[MID_VIEW_FOCUS];
                break;
            case 3:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[SMA_VIEW_FOCUS];
                break;
            default:
                break;
            }
            break;
        case MASTER_ZOOM_OUT:			/*ZOOM OUT*/
            switch (sys_vl_info.vlview_Now) // 放大
            {
            case 4:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[SMA_VIEW_FOCUS];
                break;
            case 3:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[MID_VIEW_FOCUS];
                break;
            case 2:
                vl_send_type = VL_ZOOM_POSITION;
                vl_send_buf = cali_vlfocus[LARGE_VIEW_FOCUS];
                break;
            default:
                break;
            }
            break;
        case MASTER_VIEW_ELECTRIC: /*电子变倍 1 2 4*/
            vl_send_type = VL_ELE_ZOOM;
            if ((sys_vl_info.ele_zoom == 0)||(sys_vl_info.ele_zoom == 1))
            {
                vl_send_buf = 2;
            }
            else if ((sys_vl_info.ele_zoom == 2)||(sys_vl_info.ele_zoom == 3))
            {
                vl_send_buf = 2;
            }
            else if (vl_send_buf == 4)
            {
                vl_send_buf = 0;
            }
            break;
        case MASTER_PICTURE_AUTO: /*图像自动调节*/
            vl_send_type = VL_CALI;//图像校正
            break;

        case MASTER_GAIN_ADD: /*增益＋*/
            //手动模式才可加减，否则执行不成功
            if (sys_vl_info.vis_sta_2.gain_sta == 0)
            {
                vl_send_type = VL_GAIN_ADD;
            }
            else
            {
                //如果当前位自动模式，则返回。
                return;
            }
            break;
        case MASTER_GAIN_MINUS: /*增益-*/
            if (sys_vl_info.vis_sta_2.gain_sta == 0)
            {
                vl_send_type = VL_GAIN_MINUS;
            }
            else
            {
                //如果当前位自动模式，则返回。
                return;
            }
            break;

        case MASTER_LIGHT_ADD:/*亮度＋*/
            if (sys_vl_info.vis_sta_1.light_sta == 0) /*手动*/
            {
                vl_send_type = VL_LIGHT_ADD;
            }
            else
            {
                return;
            }
            break;
        case MASTER_LIGHT_MINUS:/*亮度-*/
            if (sys_vl_info.vis_sta_1.light_sta == 0) /*手动*/
            {
                vl_send_type = VL_LIGHT_MINUS;
            }
            else
            {
                return;
            }
            break;
        case MASTER_FOCUS_ON_AUTO: /*自动聚焦*/
            vl_send_type = VL_FOCUS_AUTO;
            break;
        case MASTER_PICTURE_ENHANCE_ON: /*进入图象增强*/
            vl_send_type = VL_IMAGE_ENHANCE;
            vl_send_buf = 3;
            break;
        case MASTER_PICTURE_ENHANCE_OFF: /*退出图象增强*/
            vl_send_type = VL_IMAGE_ENHANCE;
            vl_send_buf = 2;
            break;

        default:
            //都不符合，则状态清空,清空后返回
            CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
            return;
        }
        //User_Cmd.MC_TO_VLCtrl = NULL;
        //User_Cmd.MC_TO_VLPara = NULL;
    }
#if 0
    /*图像自动调节、亮度手动调节、增强手动调节*/
    //由于部分功能没有，比如图像自动调节，需要手动写调节逻辑
    if ((!User_Cmd.VLCtrl_flg) && (User_Cmd.VLCtrl_type))
    {
        if (User_Cmd.VLCtrl_type == MASTER_PICTURE_AUTO) /*图像自动调节*/
        {
            if (!User_Cmd.VLCtrl_step)
                User_Cmd.VLCtrl_step = 2;

            if (User_Cmd.VLCtrl_step == 2)
            {
                User_Cmd.VLCtrl_Cmd = VL_LIGHT_AUTO;
            }
            else if (User_Cmd.VLCtrl_step == 1)
            {
                User_Cmd.VLCtrl_Cmd = VL_GAIN_AUTO;
                User_Cmd.VLCtrl_type = NULL;
            }
            User_Cmd.VLCtrl_step--;
        }
    }
#endif
    /*指令发送 -选择控制的可见光函数，并执行控制函数的发送*/
#if (VISIBLE_INCLUDE&VL_F15_300)
    Visible_API_Ctrl_SendHandle(VL_F15_300,vl_send_type, (uint8_t *)&vl_send_buf);
#endif
#if (VISIBLE_INCLUDE&VL_VS2030)
    Visible_API_Ctrl_SendHandle(VL_VS2030,vl_send_type, (uint8_t *)&vl_send_buf);
#endif
    CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_NULL);
}

/***********************************定时器****************************************************/

void IWDG_Refresh_Period_Handle(void)
{
    /*看门狗喂狗*/
    if (HAL_GetTick() >= User_Tick.iwdg_tick + 20) // 喂狗时间20ms
    {
        User_Tick.iwdg_tick = HAL_GetTick();
        HAL_IWDG_Refresh(&hiwdg); // 看门狗喂狗
    }
}
#endif