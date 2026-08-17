
#include "Driver/drv_uart.h"

#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
#include "../ir/ir_ctrl_dybmc_l640.h"
#include "Common/utl_check.h"
#include "Common/utl_math.h"
#include "Common/config.h"

#define IR_DATA_SEND_HEADER_1 0xAA
#define IR_DATA_SEND_HEADER_2 0x01
#define IR_DATA_RECV_HEADER_2 0x56
#define IR_SEND_DATA_TAIL (0xCC)

static COM_RECV_INFO_T ir_l640_rcv_buf;
IR_DATA_CTRL_T ir_send_buf;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_Dybmc_Serial_Data_Init(void)
{
	ir_l640_rcv_buf.header1 = IR_DATA_SEND_HEADER_1;
	ir_l640_rcv_buf.header2 = IR_DATA_SEND_HEADER_2;
	ir_l640_rcv_buf.tail1 = IR_SEND_DATA_TAIL;
	ir_l640_rcv_buf.data_recv_len = 36;
	COM_Rcv_SerialPort_Init(COM_IR_DYBMC, ir_l640_rcv_buf.header1,ir_l640_rcv_buf.tail1,ir_l640_rcv_buf.data_recv_len);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Send_Without_Para(uint8_t cmd)
{
    SYS_IR_STA_T sys_ir_info;
    sys_ir_info = CONFIG_Get_Ir_Info();

    memset(&ir_send_buf,0x0,IR_CTRL_DATA_LEN);

    ir_send_buf.Header1 = IR_DATA_SEND_HEADER_1;
    ir_send_buf.Header2 = IR_DATA_SEND_HEADER_2;
    switch(cmd)
    {
    case IR_ZOOM_ADD:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_ADD;
        break;

    case IR_ZOOM_MINUS:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_MINUS;
        break;

    case IR_ZOOM_STOP:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_STOP;
        break;

    case IR_ZOOM_STEP_ADD:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_STEP_ADD;
        ir_send_buf.ctrl_data1 = IRZOOM_STEP_STEP;
        break;

    case IR_ZOOM_STEP_MINUS:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_STEP_MINUS;
        ir_send_buf.ctrl_data1 = IRZOOM_STEP_STEP;
        break;

    case IR_FOCUS_ADD:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_ADD;
        break;

    case IR_FOCUS_MINUS:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_MINUS;
        break;

    case IR_FOCUS_STOP:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_STOP;
        break;

    case IR_FOCUS_AUTO: /*自动聚焦*/
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_AUTO;
        break;

    case IR_FOCUS_STEP_ADD:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_STEP_ADD;
        ir_send_buf.ctrl_data1 = IRFOCUS_STEP_STEP;
        break;

    case IR_FOCUS_STEP_MINUS:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_STEP_MINUS;
        ir_send_buf.ctrl_data1 = IRFOCUS_STEP_STEP;
        break;

    case IR_CFG_CROSSHAIR_RIGHT:	/*十字右移*/
        ir_send_buf.ctrl_cmd = IR_SET_CROSSHAIR_RIGHT;
        break;
    case IR_CFG_CROSSHAIR_LEFT: /*十字左移*/
        ir_send_buf.ctrl_cmd = IR_SET_CROSSHAIR_LEFT;
        break;

    case IR_CFG_CROSSHAIR_DOWN:/*十字下移*/
        ir_send_buf.ctrl_cmd = IR_SET_CROSSHAIR_DOWN;
        break;

    case IR_CFG_CROSSHAIR_UP:	/*十字上移*/
        ir_send_buf.ctrl_cmd = IR_SET_CROSSHAIR_UP;
        break;

    case IR_CFG_CROSSHAIR_RES:	/*十字复位*/
        ir_send_buf.ctrl_cmd = IR_SET_DEAD_PIXEL_HAND;
        break;

    case IR_FOCUS_WITH_ONE_STEP:/*一键聚焦*/
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_ONE_STEP;
        break;


    case IR_LIGHT_ADD:
    case IR_LIGHT_MINUS:
        ir_send_buf.ctrl_cmd = IR_SET_LIGHT_LEVEL;
        if (cmd == IR_LIGHT_ADD)
        {
            sys_ir_info.light_level++;
        }
        else if (cmd == IR_LIGHT_MINUS)
        {
            sys_ir_info.light_level--;
        }
        ir_send_buf.ctrl_data1 = sys_ir_info.light_level;
        break;
    default:
        //return CMD_ERR;
        break;
    }
    ir_send_buf.tail_1 = IR_SEND_DATA_TAIL;
    ir_send_buf.tail_2 = 0;

    COM_API_Send_Data(COM_IR_DYBMC,(uint8_t *)&ir_send_buf,sizeof(ir_send_buf));
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Send_With_Other_Para(uint8_t cmd,uint8_t *data)
{
    SYS_IR_STA_T sys_ir_info;
    sys_ir_info = CONFIG_Get_Ir_Info();
    uint16_t data_para  = 0;

    memset(&ir_send_buf,0x0,IR_CTRL_DATA_LEN);

    ir_send_buf.Header1 = IR_DATA_SEND_HEADER_1;
    ir_send_buf.Header2 = IR_DATA_SEND_HEADER_2;
    switch(cmd)
    {

    case IR_ZOOM_POSITION:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_POSITION;
        memcpy(&data_para,data,sizeof(data_para));
        if ((data_para > IRMaxZoom) || (data_para < IRMinZoom))
        {
            return PARA_ERR;
        }
        else
        {
            data_para = UTL_Htons(data_para);
            memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        }
        break;

    case IR_ZOOM_DIT:
        ir_send_buf.ctrl_cmd = IR_SET_ZOOM_ELECTRONIC;
        if ((data[0] >4))
        {
            return PARA_ERR;
        }
        else
        {
            ir_send_buf.ctrl_data1 = data[0];
        }
        break;
    case IR_FOCUS_POSITION:
    case IR_CH_MLARGE:			/*极大视场*/
    case IR_CH_LARGE:
    case IR_CH_SMA:
    case IR_CH_MIN:
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_POSITION;
        memcpy(&data_para,data,sizeof(data_para));
        data_para = UTL_Htons(data_para);
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    case IR_BLACK_WHITE://0-白，1黑
        ir_send_buf.ctrl_cmd = IR_SET_BLACK_WHITE_CHOOSE;
        ir_send_buf.ctrl_data1 = data[0];
        break;

    case IR_CALIBRATION:
        ir_send_buf.ctrl_cmd = IR_SET_CALIBRATION;
        ir_send_buf.ctrl_data1 = data[0];//1:背景校正，2:虚焦校正，3:档板校正
        break;

    case IR_ITR_TIME_SET:/*积分时间设置*/
        ir_send_buf.ctrl_cmd = IR_SET_FOCUS_POSITION;
        memcpy(&data_para,data,sizeof(data_para));
        data_para = UTL_Htons(data_para);
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));

        break;
    case IR_CFG_X_POSITION:
        ir_send_buf.ctrl_cmd = IR_SET_X_POSITION;
        memcpy(&data_para,data,sizeof(data_para));
        data_para = UTL_Htons(data_para);
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    case IR_CFG_Y_POSITION:
        ir_send_buf.ctrl_cmd = IR_SET_Y_POSITION;
        memcpy(&data_para,data,sizeof(data_para));
        data_para = UTL_Htons(data_para);
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    case IR_CFG_DEAD_PIXEL_AUTO:	/*自动去盲元*/
        ir_send_buf.ctrl_cmd = IR_SET_DEAD_PIXEL_AUTO;
        ir_send_buf.ctrl_data1 = data[0];
        ir_send_buf.ctrl_data2 = data[1];
        break;

    case IR_CFG_DEAD_PIXEL_HAND:	/*手动去盲元*/
        ir_send_buf.ctrl_cmd = IR_SET_DEAD_PIXEL_HAND;
        ir_send_buf.ctrl_data1 = data[0];
        ir_send_buf.ctrl_data2 = data[1];
        break;

    case IR_SET_CONTRAST:
        ir_send_buf.ctrl_cmd = IR_SET_CONTRAET_LEVEL;
        memcpy(&data_para,data,sizeof(data_para));
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    case IR_SET_LIGHT:
        ir_send_buf.ctrl_cmd = IR_SET_LIGHT_LEVEL;
        memcpy(&data_para,data,sizeof(data_para));
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    case IR_DDE_RANGE:
        ir_send_buf.ctrl_cmd = IR_SET_DDE_LEVEL;
        memcpy(&data_para,data,sizeof(data_para));
        memcpy(&ir_send_buf.ctrl_data1,&data_para,sizeof(data_para));
        break;

    default:
        return CMD_ERR;
        break;
    }
    ir_send_buf.tail_1 = IR_SEND_DATA_TAIL;
    ir_send_buf.tail_2 = 0;

   COM_API_Send_Data(COM_IR_DYBMC,(uint8_t *)&ir_send_buf,sizeof(ir_send_buf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Send_Enable(uint8_t cmd,uint8_t enable)
{
//	SYS_IR_STA_T sys_ir_info;
//	sys_ir_info = CONFIG_Get_Ir_Info();

    memset(&ir_send_buf,0x0,IR_CTRL_DATA_LEN);

    ir_send_buf.Header1 = IR_DATA_SEND_HEADER_1;
    ir_send_buf.Header2 = IR_DATA_SEND_HEADER_2;
    switch(cmd)
    {

    case IR_IMAGE_ENHANCE:		/*图像增强开关 dde*/
        ir_send_buf.ctrl_cmd = IR_SET_DDE_ENHANCE;
        break;
    case IR_CFG_FILTER:			/*滤波*/
        ir_send_buf.ctrl_cmd = IR_SET_FILTER;
        break;
    case IR_CROSS_CTRL:			/*十字开关*/
        ir_send_buf.ctrl_cmd = IR_SET_CROSSHAIR_CHOOSE;
        break;

    default:
        return CMD_ERR;
        break;
    }

    if (enable == CMD_ENABLE)
    {
        ir_send_buf.ctrl_data1 = 1;
    }
    else if(enable == CMD_DISABLE)
    {
        ir_send_buf.ctrl_data1 = 0;
    }
    ir_send_buf.tail_1 = IR_SEND_DATA_TAIL;
    ir_send_buf.tail_2 = 0;

    COM_API_Send_Data(COM_IR_DYBMC,(uint8_t *)&ir_send_buf,sizeof(ir_send_buf));
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Ctrl_Dybmc_SendHandle(uint8_t send_type,uint8_t *data)
{
    uint16_t data_para = 0;
    SYS_IR_STA_T sys_ir_info;
    sys_ir_info = CONFIG_Get_Ir_Info();

    if ((send_type == IR_ZOOM_ADD)
            ||(send_type == IR_ZOOM_MINUS))
    {
        if ((sys_ir_info.zoom_position > IRMaxZoom)
                ||(sys_ir_info.zoom_position < IRMinZoom))
        {
            return PARA_ERR;
        }
    }
    switch(send_type)
    {
    case IR_ZOOM_ADD:
    case IR_ZOOM_MINUS:
    case IR_ZOOM_STOP:
    case IR_ZOOM_STEP_ADD:
    case IR_ZOOM_STEP_MINUS:
    case IR_FOCUS_ADD:
    case IR_FOCUS_MINUS:
    case IR_FOCUS_STOP:
    case IR_FOCUS_AUTO:
    case IR_FOCUS_WITH_ONE_STEP:	/*一键聚焦*/
    case IR_FOCUS_STEP_ADD:
    case IR_FOCUS_STEP_MINUS:
    case IR_LIGHT_ADD:
    case IR_LIGHT_MINUS:
    case IR_CFG_CROSSHAIR_RIGHT:/*滤波*/
    case IR_CFG_CROSSHAIR_LEFT:/*十字右移*/
    case IR_CFG_CROSSHAIR_DOWN:/*十字左移*/
    case IR_CFG_CROSSHAIR_UP:/*十字下移*/
    case IR_CFG_CROSSHAIR_RES:	/*十字上移*/
        IR_Ctrl_Send_Without_Para(send_type);
        break;

    case IR_IMAGE_ENHANCE:		/*图像增强开关 dde*/
    case IR_CFG_FILTER:			/*滤波*/
    case IR_CROSS_CTRL:			/*十字开关*/
        IR_Ctrl_Send_Enable(send_type,data[0]);
        break;

    case IR_ZOOM_POSITION:
    case IR_ZOOM_DIT:
    case IR_FOCUS_POSITION:
    case IR_CH_MLARGE:			/*极大视场*/
    case IR_CH_LARGE:
    case IR_CH_SMA:
    case IR_CH_MIN:
    case IR_BLACK_WHITE://0-白，1黑
    case IR_CALIBRATION:
    case IR_ITR_TIME_SET:/*积分时间设置*/
    case IR_CFG_X_POSITION:
    case IR_CFG_Y_POSITION:
    case IR_CFG_DEAD_PIXEL_AUTO:	/*自动去盲元*/
    case IR_CFG_DEAD_PIXEL_HAND:	/*手动去盲元*/
    case IR_SET_CONTRAST:
    case IR_DDE_RANGE:
    case IR_SET_LIGHT:
        IR_Ctrl_Send_With_Other_Para(send_type,data);
        break;
    default:
        return CMD_ERR;
        break;
    }
    return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:姿态通信协议
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_PROCESS_Get(uint8_t *data,uint32_t length)
{
    IR_DATA_RECV_T rcv_ir_info;
    SYS_IR_STA_T main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    //长度判断
    if (length != IR_RECV_DATA_LEN)
    {
        //printf("\r\n len_err  %d  %d",length,IR_RECV_DATA_LEN);
        return RECV_DATA_LEN_ERR;
    }

    //帧尾判断
    if(data[length-1] != IR_SEND_DATA_TAIL)
    {
        //printf("\r\n tail_err  %d  %d",data[length-1],IR_SEND_DATA_TAIL);
        return RECV_DATA_XOR_ERR;
    }
    memcpy(&rcv_ir_info,data,IR_RECV_DATA_LEN);
#if 0
    //测试验证程序
    //memcpy(&main_ir_info.ir_status,&rcv_ir_info.ir_status,1);
    //Infrared_Ctrl_SendHandle(data[3],(uint8_t *)&rcv_ir_info.integration_time);
#endif
    memcpy(&main_ir_info.ir_status,&rcv_ir_info.ir_status,1);
    main_ir_info.light_level = rcv_ir_info.light_level;
    main_ir_info.contrast_level = rcv_ir_info.contrast_level;
    main_ir_info.elezoom = rcv_ir_info.elezoom;
    main_ir_info.DDE_level = rcv_ir_info.enhancement;
    main_ir_info.itr_range = rcv_ir_info.itr_range;
    main_ir_info.integration_time = UTL_Htons(rcv_ir_info.integration_time);
    main_ir_info.crosshair_x = UTL_Htons(rcv_ir_info.crosshair_x);
    main_ir_info.crosshair_y = UTL_Htons(rcv_ir_info.crosshair_y);
    main_ir_info.work_time = UTL_Htonl(rcv_ir_info.work_time);
    main_ir_info.startup_cnt = UTL_Htons(rcv_ir_info.startup_cnt);
    main_ir_info.detect_temp = UTL_Htons(rcv_ir_info.detect_temp);
    main_ir_info.ambient_temp = UTL_Htons(rcv_ir_info.ambient_temp);
    main_ir_info.grayscale = UTL_Htons(rcv_ir_info.grayscale);
    main_ir_info.zoom_position = UTL_Htons(rcv_ir_info.zoom_position) * 0.1;
    main_ir_info.ir_focus_value = UTL_Htons(rcv_ir_info.focus_position);
    main_ir_info.lens_temp = rcv_ir_info.lens_temp;

    /*计算数据*/
    main_ir_info.IRRange_H = 57.3*(2*(atan((IRPixel*IRPNum_H*0.001)/((uint16_t)main_ir_info.zoom_position))));
    main_ir_info.IRRange_V = 57.3*(2*(atan((IRPixel*IRPNum_V*0.001)/((uint16_t)main_ir_info.zoom_position))));
    /*当前档位判断*/
#if 0
    if (main_ir_info.zoom_position <= (IR_LAR_FOCUS + IR_MID_FOCUS) / 2)
        main_ir_info.irview_Now = 1;
    else if (main_ir_info.zoom_position <= (IR_MID_FOCUS + IR_SMA_FOCUS) / 2)
        main_ir_info.irview_Now = 2;
    else if (main_ir_info.zoom_position <= (IR_SMA_FOCUS + IR_MIN_FOCUS) / 2)
        main_ir_info.irview_Now = 3;
    else
        main_ir_info.irview_Now = 4;
#endif
#if 0
    DEBUG_LASER_PRINT("=============================");
    DEBUG_LASER_PRINT("\r\n积分时间挡位[%d]\r\n积分时间[%d]\r\nX轴坐标[%d]\r\nY轴坐标[%d]\r\n十字[%s]\r\n滤波[%s]",
                      main_ir_info.itr_range,
                      main_ir_info.integration_time,
                      main_ir_info.crosshair_x,
                      main_ir_info.crosshair_y,
                      //							main_ir_info.ir_status.image_change,
                      main_ir_info.ir_status.crosshair_on? "on":"off",
                      //							main_ir_info.ir_status.video_output ? 8:14,
                      main_ir_info.ir_status.filter_sta ? "on":"off");
    DEBUG_LASER_PRINT ("=============================");

    DEBUG_LASER_PRINT("\r\n亮度[%d]\r\n对比度[%d]\r\nDDE[%d]\r\n灰度[%d]\r\n变倍[%d]\r\n调焦[%d]\r\n极性[%c]\r\n探测器温度[%dK]\r\n环境温度[%d℃]\r\n镜头温度[%d℃]",
                      main_ir_info.light_level,
                      main_ir_info.contrast_level,
                      main_ir_info.DDE_level,
                      main_ir_info.grayscale,
                      main_ir_info.zoom_position,
                      main_ir_info.ir_focus_value,
                      main_ir_info.ir_status.black_white ? 'B':'W',
                      main_ir_info.detect_temp,
                      main_ir_info.ambient_temp,
                      main_ir_info.lens_temp);
#endif
    CONFIG_Set_Ir_Info(main_ir_info);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收后的数据处理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_Dybmc_Period_Handle(void)
{
    uint8_t recv_len = 0;
	recv_len = COM_REC_DataAnalysis_nocheck(COM_IR_DYBMC,ir_l640_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
	
	IR_PROCESS_Get(ir_l640_rcv_buf.recv_buf,recv_len);
	
	return RECV_DATA_SUC;

}
#endif
