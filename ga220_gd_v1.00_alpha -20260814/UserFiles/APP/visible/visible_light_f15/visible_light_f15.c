#include "Common/base_inc.h"
#include "Common/utl_math.h"
#include "../visible/visible_light_f15.h"
#include "Common/config.h"

#include "../cli/cli_cmd_line.h"
#include "Driver/drv_uart.h"
#include "COMMON/utl_check.h"

#if (VISIBLE_INCLUDE&VL_F15_300)

#define VISIBLE_DATA_SEND_HEADER_2 0xAA
#define VISIBLE_DATA_SEND_HEADER_1 0x55
#define VISIBLE_DATA_RECV_HEADER_2 0x55
#define VISIBLE_DATA_RECV_HEADER_1 0xAA

#define VISIBLE_SEND_DATA_TAIL 0xFE
#define VISIBLE_DATA_MAX_INDEX   5

static COM_RECV_INFO_T visible_f15_rcv_buf;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_F15_Serial_Data_Init(void)
{
	visible_f15_rcv_buf.header1 = VISIBLE_DATA_RECV_HEADER_1;
	//尾使用和校验
	visible_f15_rcv_buf.tail1 = 0;
	visible_f15_rcv_buf.data_recv_len = 12;
	COM_Rcv_SerialPort_Init(COM_KJG_IN, visible_f15_rcv_buf.header1,visible_f15_rcv_buf.tail1,visible_f15_rcv_buf.data_recv_len);

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:控制指令；
			ctrl_type:指令类型；
			ctrl_data:数据内容；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#define VL_ZOOM_MAX (300)
#define VL_ZOOM_MIN (15)
#define VL_FOCUS_STEP_MAX (255)
#define VL_FOCUS_STEP_MIN (50)
#define VL_LIGHT_MAX (0xA0)
#define VL_LIGHT_MIN (0x20)
#define VL_GAIN_MAX (240)
uint8_t Visible_F15_Ctrl_SendHandle(uint8_t send_type,uint8_t *ctrl_data)
{

    VISIBLE_F15_SEND_T visible_send_buf;

    uint16_t data_temp_buf = 0;
    CONFIG_Get_Visible_Info();
    memset(&visible_send_buf,0x0,VISIBLE_SEND_DATA_LEN);

    visible_send_buf.Header1 = VISIBLE_DATA_SEND_HEADER_1;
    visible_send_buf.Header2 = VISIBLE_DATA_SEND_HEADER_2;
    visible_send_buf.connect_method = 0;
    visible_send_buf.ctrl_cmd = 2;
    switch (send_type)
    {
//    case VL_ZOOM_ADD:
//        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
//        visible_send_buf.ctrl_cmd2 = 3;
//        visible_send_buf.ctrl_para[0] = VL_ZOOM_STEP_LEN;
//        break;

//    case VL_ZOOM_MINUS:
//        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
//        visible_send_buf.ctrl_cmd2 = 4;
//        visible_send_buf.ctrl_para[0] = VL_ZOOM_STEP_LEN;
//        //if ((sys_visiable.zoom_value > VL_ZOOM_MAX)||(sys_visiable.zoom_value < VL_ZOOM_MIN))
//        {
//            //	return PARA_ERR;
//        }
//        break;

//    case VL_ZOOM_POSITION:
//        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
//        visible_send_buf.ctrl_cmd2 = 0x33;
//        memcpy(&data_temp_buf,ctrl_data,sizeof(data_temp_buf));
//        if (data_temp_buf > VL_ZOOM_MAX)
//        {
//            data_temp_buf = VL_ZOOM_MAX;
//        }
//        else if (data_temp_buf < VL_ZOOM_MIN)
//        {
//            data_temp_buf = VL_ZOOM_MIN;
//        }
//        memcpy(visible_send_buf.ctrl_para,&data_temp_buf,sizeof(data_temp_buf));
//        break;

//    case VL_ELE_ZOOM://1-2-4
//        visible_send_buf.ctrl_cmd1 = VL_F15_ELE_ZOOM;
//        visible_send_buf.ctrl_cmd2 = 0x33;
//        if (ctrl_data[0] > 4)
//        {
//            return PARA_ERR;
//        }
//        else
//        {
//            visible_send_buf.ctrl_para[0] = ctrl_data[0];
//        }
//        break;

//    case VL_FOCUS_ADD:
//        visible_send_buf.ctrl_cmd1 = VL_F15_SET_FOCUS;
//        visible_send_buf.ctrl_cmd2 = 1;
//        visible_send_buf.ctrl_para[0] = VL_FOCUS_STEP_LEN;
//        break;

//    case VL_FOCUS_MINUS:
//        visible_send_buf.ctrl_cmd1 = VL_F15_SET_FOCUS;
//        visible_send_buf.ctrl_cmd2 = 2;
//        visible_send_buf.ctrl_para[0] = VL_FOCUS_STEP_LEN;
//        break;

    case VL_FOCUS_AUTO:
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_FOCUS;
        visible_send_buf.ctrl_cmd2 = 3;
        break;

    case VL_FOCUS_ONE_SET:
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_FOCUS;
        visible_send_buf.ctrl_cmd2 = 4;;
        break;

//    case VL_GAIN_ADD:
//        visible_send_buf.ctrl_cmd1 = VL_F15_ENHANCE;
//        visible_send_buf.ctrl_cmd2 = 1;
//        visible_send_buf.ctrl_para[0] = VL_GAIN_STEP_LEN;
//        break;

//    case VL_GAIN_MINUS:
//        visible_send_buf.ctrl_cmd1 = VL_F15_ENHANCE;
//        visible_send_buf.ctrl_cmd2 = 2;
//        visible_send_buf.ctrl_para[0] = VL_GAIN_STEP_LEN;
//        break;

//    case VL_GAIN_SET://0-240
//        visible_send_buf.ctrl_cmd1 = VL_F15_ENHANCE;
//        visible_send_buf.ctrl_cmd2 = 0x33;
//        if (ctrl_data[0] > VL_GAIN_MAX)
//        {
//            return PARA_ERR;
//        }
//        else
//        {
//            visible_send_buf.ctrl_para[0] = ctrl_data[0];
//        }
//        break;
//    case VL_GAIN_AUTO:
//        visible_send_buf.ctrl_cmd1 = VL_F15_ENHANCE;
//        visible_send_buf.ctrl_cmd2 = 3;
//        break;

//    case VL_GAIN_HANDLE:
//        visible_send_buf.ctrl_cmd1 = VL_F15_ENHANCE;
//        visible_send_buf.ctrl_cmd2 = 0;
//        break;

//    case VL_LIGHT_HANDIE:/*亮度手动*/
//        visible_send_buf.ctrl_cmd1 = VL_F15_LIGHT_CTRL;
//        visible_send_buf.ctrl_cmd2 = 0;
//        break;

//    case VL_LIGHT_AUTO:/*亮度自动*/
//        visible_send_buf.ctrl_cmd1 = VL_F15_LIGHT_CTRL;
//        visible_send_buf.ctrl_cmd2 = 3;
//        break;

//    case VL_LIGHT_ADD:/*亮度加*/
//        visible_send_buf.ctrl_cmd1 = VL_F15_LIGHT_CTRL;
//        visible_send_buf.ctrl_cmd2 = 1;
//        break;

//    case VL_LIGHT_MINUS:/*亮度减*/
//        visible_send_buf.ctrl_cmd1 = VL_F15_LIGHT_CTRL;
//        visible_send_buf.ctrl_cmd2 = 2;
//        break;

//    case VL_LIGHT_POSITION://0x20-0xA0
//        visible_send_buf.ctrl_cmd1 = VL_F15_LIGHT_CTRL;
//        visible_send_buf.ctrl_cmd2 = 0x33;
//        if (ctrl_data[0] > VL_LIGHT_MIN)
//        {
//            ctrl_data[0] = VL_LIGHT_MIN;
//        }
//        else if (ctrl_data[0] < VL_LIGHT_MAX)
//        {
//            ctrl_data[0] = VL_LIGHT_MAX;
//        }
//        visible_send_buf.ctrl_para[0] = ctrl_data[0];
//        break;


#if 1
    case VL_CH_MLARGE:		/*极大视场*/
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
        visible_send_buf.ctrl_cmd2 = 1;
        visible_send_buf.ctrl_para[0] = 5;
        break;
    case VL_CH_LARGE:			/*大视场*/
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
        visible_send_buf.ctrl_cmd2 = 1;
        visible_send_buf.ctrl_para[0] = 4;
        break;
    case VL_CH_MI:				/*中视场*/
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
        visible_send_buf.ctrl_cmd2 = 1;
        visible_send_buf.ctrl_para[0] = 3;
        break;
    case VL_CH_SMA:				/*小视场*/
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
        visible_send_buf.ctrl_cmd2 = 1;
        visible_send_buf.ctrl_para[0] = 2;
        break;
    case VL_CH_MIN:				/*极小视场*/
        visible_send_buf.ctrl_cmd1 = VL_F15_SET_ZOOM;
        visible_send_buf.ctrl_cmd2 = 1;
        visible_send_buf.ctrl_para[0] = 1;
        break;
#endif

    default:
        return CMD_ERR;
    }
    visible_send_buf.add_check = UTL_ADD_CHECK((uint8_t *)&visible_send_buf.ctrl_cmd,(VISIBLE_SEND_DATA_LEN-4));
    visible_send_buf.tail_1 = VISIBLE_SEND_DATA_TAIL;
    visible_send_buf.tail_2 = VISIBLE_SEND_DATA_TAIL;

    COM_API_Send_Data(COM_KJG_IN,(uint8_t *)&visible_send_buf,sizeof(visible_send_buf));
#if CLI_INCLUDE
    DEBUG_VL_TX_PRINT("\r\n VL_SEND:");
    DEBUG_VL_TX_PRINT("%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                      visible_send_buf.Header1,
                      visible_send_buf.Header1,
                      visible_send_buf.connect_method,
                      visible_send_buf.ctrl_cmd,
                      visible_send_buf.ctrl_cmd1,
                      visible_send_buf.ctrl_cmd2,
                      visible_send_buf.ctrl_para[0],
                      visible_send_buf.ctrl_para[1],
                      visible_send_buf.ctrl_para[2],
                      visible_send_buf.ctrl_para[3],
                      visible_send_buf.add_check,
                      visible_send_buf.tail_1,
                      visible_send_buf.tail_2);
#endif
    return RECV_DATA_SUC;

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
//static uint8_t enter_cnt = 0;
uint8_t visiable_process_data_in(uint8_t *data,uint32_t length)
{
    VISIBLE_F15_RECV_T rcv_visible_info;
    SYS_VISIBLE_DATA_T  sys_vl_info;
    uint8_t sum_check = 0;
    float cali_vlfocus[VIEW_FOCUS_END];

    //获取标定的焦距值
    CONFIG_Get_Cali_vlFocus(cali_vlfocus);
	
    //校验计算
    sum_check = UTL_ADD_CHECK(&data[2],(length-5));
    if (data[length-3] != sum_check)
    {
#if CLI_INCLUDE
        DEBUG_VL_INFO_PRINT("\r\n check %d %d ",sum_check,data[length-3]);
#endif
        return RECV_DATA_XOR_ERR;
    }

    memcpy(&rcv_visible_info,data,VISIBLE_RECV_DATA_LEN);
    //将输入的报文copy到实际里面去
    memcpy(&sys_vl_info.vis_sta_1,&rcv_visible_info.video_sta,sizeof(rcv_visible_info.video_sta));
    memcpy(&sys_vl_info.vis_sta_2,&rcv_visible_info.cmd_sta,sizeof(rcv_visible_info.cmd_sta));
    sys_vl_info.vis_sta_2.vis_hlc = rcv_visible_info.light_sta.view_hlc;
    sys_vl_info.vis_sta_2.vis_blc = rcv_visible_info.light_sta.view_blc;
    memcpy(&sys_vl_info.focus_limit,&rcv_visible_info.focus_sta,sizeof(rcv_visible_info.focus_sta));
    sys_vl_info.enhance_degree = rcv_visible_info.enhance_degree;

    sys_vl_info.zoom_value = rcv_visible_info.zoom_position;
    sys_vl_info.ele_zoom = rcv_visible_info.zoom_ele;
    sys_vl_info.visible_fps = rcv_visible_info.video_fps;
    sys_vl_info.light_degree = rcv_visible_info.light_degree;
    sys_vl_info.gain_degree = rcv_visible_info.gain_degree;
    sys_vl_info.contrast_degree = rcv_visible_info.contrast_degree;
    sys_vl_info.view_ch = rcv_visible_info.view_sta.view_ch;
    if (APP_IS_BIT_SET(rcv_visible_info.video_alarm, 0))
    {
        sys_vl_info.core_err = 1;
    }
    else
    {
        sys_vl_info.core_err = 0;
    }
    /*水平视场角-度*/
    sys_vl_info.VLRange_H = 57.3*(2*( atan((VLPixel*VLPNum_H*0.001*0.5)/((uint16_t)rcv_visible_info.zoom_position))));
    /*垂直视场角-度*/
    sys_vl_info.VLRange_V = 57.3*(2*( atan((VLPixel*VLPNum_V*0.001*0.5)/((uint16_t)rcv_visible_info.zoom_position))));

    /*判断当前视场档位*/
    if (sys_vl_info.zoom_value <= ((cali_vlfocus[LARGE_VIEW_FOCUS] + cali_vlfocus[MID_VIEW_FOCUS]) / 2))
    {
        sys_vl_info.vlview_Now = 1;
    }
    else if (sys_vl_info.zoom_value <= (cali_vlfocus[MID_VIEW_FOCUS] + cali_vlfocus[SMA_VIEW_FOCUS]) / 2)
    {
        sys_vl_info.vlview_Now = 2;
    }
    else if (sys_vl_info.zoom_value <= (cali_vlfocus[SMA_VIEW_FOCUS] + cali_vlfocus[MIN_VIEW_FOCUS]) / 2)
    {
        sys_vl_info.vlview_Now = 3;
    }
    else
    {
        sys_vl_info.vlview_Now = 4;
    }
#if CLI_INCLUDE
    DEBUG_VL_INFO_PRINT("\r\n %d %d-%d %d",sys_vl_info.zoom_value,sys_vl_info.ele_zoom,sys_vl_info.visible_fps,sys_vl_info.light_degree);
    DEBUG_VL_INFO_PRINT("\r\n=============================");
#if 0
    DEBUG_VL_INFO_PRINT("\r\n%8s：%dmm \r\n%8s：%d\r\n %8s：%dfps \r\n%8s：%d\r\n %8s：%d \r\n%8s：%d \r\n%8s：%d \r\n%8s：%s",
                        "焦距",sys_vl_info.focus_value,
                        "电子变倍",sys_vl_info.ele_zoom,
                        "帧频",sys_vl_info.visible_fps,
                        "亮度",sys_vl_info.light_degree,
                        "增益",sys_vl_info.gain_degree,
                        "对比度",sys_vl_info.contrast_degree,
                        "机芯故障",sys_vl_info.core_err//?"故障":"正常"
                       );
#endif
    DEBUG_VL_INFO_PRINT("\r\n=============================");
#endif
    CONFIG_Set_Visible_Info(sys_vl_info);
    
    return true;
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
uint8_t VISIABLE_F15_Period_Handle(void)
{
    uint8_t recv_len = 0;
    
	recv_len = COM_REC_DataAnalysis_nocheck(COM_KJG_IN,visible_f15_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
#if CLI_INCLUDE
	DEBUG_VL_INFO_PRINT("\r\n LASER_RCV:");
	for(uint8_t i = 0; i < recv_len; i++)
	{
		DEBUG_VL_INFO_PRINT("%02x ",visible_f15_rcv_buf.recv_buf[i]);
	}
#endif
	visiable_process_data_in(visible_f15_rcv_buf.recv_buf,recv_len);
    
	return RECV_DATA_SUC;
}
#endif
