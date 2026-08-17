#include "stm32f4xx_hal.h"
#include "string.h"
#include "Common/utl_math.h"
//#include "Common/com_api.h"
#include "Common/config.h"
#include "../ir/ir_ctrl_s640a_c1.h"
#include "Common/opt_cmd.h"


#if (IR_CTRL_INCLUDE&IR_S640A_C1)

#define IR_S640A_HEADER 0xF0
#define IR_S640A_LOCAL_ID 0x26
#define IR_S640_DATA_TAIL 0xFF


static uint8_t ir_s640a_send_buf[12];
static COM_BUF_INFO_T ir_s640a_rcv_buf;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:清除接收buf
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_S640A_RECBUFF_Clear(void)
{
    ir_s640a_rcv_buf.buf_rcved = false;
    ir_s640a_rcv_buf.length = 0;
    memset(ir_s640a_rcv_buf.com_data_buf,0,sizeof(ir_s640a_rcv_buf.com_data_buf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接受数据头判断，状态切换
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool IR_S640a_Data_in_rcv_cb(uint8_t *data,uint16_t data_len)
{
    if ((data[0] != IR_S640A_HEADER)&&(data[1] != IR_S640A_LOCAL_ID))
    {
        return false;
    }
    memcpy(ir_s640a_rcv_buf.com_data_buf,data,data_len);
    ir_s640a_rcv_buf.buf_rcved = true;
    ir_s640a_rcv_buf.length = data_len;

    return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口初始化，注册串口回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void IR_S640A_API_Serial_Data_Init(void)
{
    memset(&ir_s640a_send_buf,0x0,sizeof(ir_s640a_send_buf));
    IR_S640A_RECBUFF_Clear();

    //注册串口接收数据
    COM_API_register_RcvCb(component_map[2].com_type_in,IR_S640a_Data_in_rcv_cb);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:adjust:报文发送指令
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_S640A_SendHandle(uint8_t send_type,uint8_t *data)
{
    IR_S640A_HEADER_T s640a_send_head = {0};
    uint16_t temp_buf = 0;
    bool len_sta = false;
//	uint8_t check_buf = 0;
    uint8_t total_len = 0;
    uint8_t temp_change_l = 0;
    uint8_t temp_change_h = 0;

    s640a_send_head.Header1 = IR_S640A_HEADER;
    s640a_send_head.data_len = 0;//后面计算长度之后重新赋值；
    s640a_send_head.dev_id = IR_S640A_LOCAL_ID;

    switch (send_type)
    {
    case IR_CALIBRATION:/*图像矫正*/
        if (data[0] == 1)//手动背景矫正
        {
            s640a_send_head.ctrl_cmd = IR_S640A_CALIBRATION_BACK;
            s640a_send_head.data_len = 1;
        }
        else if (data[0] == 4)//手动快门矫正
        {
            s640a_send_head.ctrl_cmd = IR_S640A_CALIBRATION_SHUTTER;
            s640a_send_head.data_len = 1;
        }
        else if (data[0] == 5)//自动矫正设置
        {
            s640a_send_head.ctrl_cmd = IR_S640A_CALIBRATION_AUTO;
            s640a_send_head.data_len = 2;
            if(data[1] == 0)
            {
                temp_buf = 0;
            }
            else
            {
                temp_buf = 0xf;
            }
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_SHUTTER_OFF:
        s640a_send_head.ctrl_cmd = IR_S640A_SHUTTER_OFF;
        s640a_send_head.data_len = 1;
        break;

    case IR_IMAGE_ENHANCE:
        s640a_send_head.ctrl_cmd = IR_S640A_IMG_ENAHNCE;
        s640a_send_head.data_len = 2;
        if(data[0] == 0)
        {
            temp_buf = 0;
        }
        else
        {
            temp_buf = 0xf;
        }
        break;
#if 0
    case IR_RENOISE:
        if (data[0] == 0)
        {
            s640a_send_head.ctrl_cmd = IR_S640A_DENOISE_T;
        }
        else if (data[0] == 1)
        {
            s640a_send_head.ctrl_cmd = IR_S640A_DENOISE_S;
        }
        if(data[1] == 0)
        {
            temp_buf = 0;
        }
        else
        {
            temp_buf = 0xf;
        }
        break;
#endif

    case IR_SET_CONTRAST:
        s640a_send_head.ctrl_cmd = IR_S640A_CONTRAST;
        s640a_send_head.data_len = 2;
        temp_buf = data[0];
        break;

    case IR_SET_LIGHT:
        s640a_send_head.ctrl_cmd = IR_S640A_LIGHT;
        s640a_send_head.data_len = 2;
        temp_buf = data[0];
        break;

    case IR_BLACK_WHITE:
        s640a_send_head.ctrl_cmd = IR_S640A_B_W_CH;
        s640a_send_head.data_len = 2;
        if(data[0] == 0)
        {
            temp_buf = 0;//白
        }
        else
        {
            temp_buf = 0xf;//黑
        }
        break;

    case IR_CROSS_CTRL:
        s640a_send_head.ctrl_cmd = IR_S640A_CROSS_HAIR;
        s640a_send_head.data_len = 2;
        if(data[0] == CMD_DISABLE)
        {
            temp_buf = 0;
        }
        else
        {
            temp_buf = 0xf;
        }
        break;

    case IR_CFG_X_POSITION:
        s640a_send_head.ctrl_cmd = IR_S640A_CROSS_X;
        memcpy(&temp_buf,data,sizeof(temp_buf));
        if (temp_buf > IRPNum_H)
        {
            temp_buf = IRPNum_H;
        }
        s640a_send_head.data_len = 3;
        break;

    case IR_CFG_Y_POSITION:
        s640a_send_head.ctrl_cmd = IR_S640A_CROSS_Y;
        memcpy(&temp_buf,data,sizeof(temp_buf));
        if (temp_buf > IRPNum_V)
        {
            temp_buf = IRPNum_V;
        }
        s640a_send_head.data_len = 3;
        break;

    case IR_SET_MIRROR:
        s640a_send_head.ctrl_cmd = IR_S640A_MIRROR;
        temp_buf = data[0];
        break;

    case IR_ZOOM_DIT:
        s640a_send_head.ctrl_cmd = IR_S640A_ZOOM_ELE;
        memcpy(&temp_buf,data,sizeof(temp_buf));
        if (temp_buf == 1)
        {
            temp_buf = 0;
            s640a_send_head.data_len = 2;
        }
        else if (temp_buf == 2)
        {
            temp_buf = 0xf;
            s640a_send_head.data_len = 2;
        }
        else if (temp_buf == 4)
        {
            temp_buf = 0x3;
            s640a_send_head.data_len = 2;
        }
        else if ((temp_buf >= 100)&&(temp_buf <= 800))
        {
            len_sta = true;
            temp_buf = temp_buf;
            s640a_send_head.data_len = 3;
        }
        else if (temp_buf > 800)
        {
            len_sta = true;
            temp_buf = 800;
            s640a_send_head.data_len = 3;
        }
        else
        {
            return PARA_ERR;
        }
        break;

    case IR_SYS_STA_REQ:
        s640a_send_head.ctrl_cmd = IR_S640A_STA_REQ;
        s640a_send_head.data_len = 1;
        break;
#if 1
    case IR_FOCUS_AUTO:
        s640a_send_head.ctrl_cmd = IR_S640A_FOCUS_AUTO;
        s640a_send_head.data_len = 1;
        break;

    case IR_FOCUS_STOP:
        s640a_send_head.ctrl_cmd = IR_S640A_FOCUS_STOP;
        s640a_send_head.data_len = 1;
        break;

    case IR_SHUTTER_ON:
        s640a_send_head.ctrl_cmd = IR_S640A_SHUTTER_ON;
        s640a_send_head.data_len = 1;
        break;
#endif

    case IR_FOCUS_ADD:
        s640a_send_head.ctrl_cmd = IR_S640A_FOCUS;
        temp_buf = 0xf;
        s640a_send_head.data_len = 2;
        break;

    case IR_FOCUS_MINUS:
        s640a_send_head.ctrl_cmd = IR_S640A_FOCUS;
        temp_buf = 0;
        s640a_send_head.data_len = 2;
        break;

    case IR_ZOOM_ADD:
        s640a_send_head.ctrl_cmd = IR_S640A_ZOOM;//(步进0.1)
        temp_buf = 0;
        s640a_send_head.data_len = 2;
        break;

    case IR_ZOOM_MINUS:
        s640a_send_head.ctrl_cmd = IR_S640A_ZOOM;//(步进0.1)
        temp_buf = 0xf;
        s640a_send_head.data_len = 2;
        break;

    case IR_SYS_RES:
        s640a_send_head.ctrl_cmd = IR_S640A_RESET;
        temp_buf = 0x379d;
        s640a_send_head.data_len = 3;
        break;

    default:
        return PARA_ERR;
    }
    //由于普遍数据长度减少了一个，所以在这里统一加1
    s640a_send_head.data_len++;
    //如果，如果出现关键字 F0H、FFH 或 F5H，需要将其变为转义字符 F5H 00H、F5H 0FH、F5H 05H 来发送
    total_len = IR_S640A_HEADER_LEN;
    temp_change_l = temp_buf & 0xff;
    if ((temp_change_l == 0xf0))
    {
        ir_s640a_send_buf[total_len] = 0xf5;
        total_len ++;
        temp_change_l = 0;
        s640a_send_head.data_len++;
    }
    else if (temp_change_l== 0xf5)
    {
        ir_s640a_send_buf[total_len] = 0xf5;
        total_len ++;
        temp_change_l = 5;
        s640a_send_head.data_len++;
    }
    else if (temp_change_l== 0xff)
    {
        ir_s640a_send_buf[total_len] = 0xf5;
        total_len ++;
        temp_change_l = 0xf;
        s640a_send_head.data_len++;
    }
    if (s640a_send_head.data_len != 1)
    {
        ir_s640a_send_buf[total_len] = temp_change_l;
        total_len++;
    }
    temp_change_h = temp_buf >> 8;

    if ((send_type == IR_CFG_X_POSITION)||(send_type == IR_CFG_Y_POSITION)||(len_sta)||(send_type == IR_SYS_RES))
    {
        ir_s640a_send_buf[total_len] = temp_change_h;
        total_len++;
    }
    memcpy(ir_s640a_send_buf,&s640a_send_head,IR_S640A_HEADER_LEN);
    ir_s640a_send_buf[total_len] = UTL_ADD_CHECK(&ir_s640a_send_buf[2], (total_len-2));
    total_len++;
    ir_s640a_send_buf[total_len] = IR_S640_DATA_TAIL;
    total_len++;

    COM_API_Send_Data(component_map[2].com_type_in,ir_s640a_send_buf,total_len);
    return CMD_SUCESS;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:濮挎€侀€氫俊鍗忚
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t IR_S640A_PROCESS_Get(uint8_t *data,uint32_t length)
{
    uint8_t i = 0;
    uint8_t j = 0;
//	IR_S640A_HEADER_T rcv_ir_header;
    //bool data_with_ch = false;
    uint8_t data_change[IR_S640A_RCV_DATA_LEN] = {0};

    IR_S640A_RCV_DATA_T rcv_ir_info;
    uint8_t sum_check = 0;

    SYS_IR_STA_T main_ir_info;
    main_ir_info = CONFIG_Get_Ir_Info();

    //校验计算
    sum_check = UTL_ADD_CHECK(&data[2],(length-4));
    if (data[length-2] != sum_check)
    {
        DEBUG_IR_PRINT("\r\n sum:%x data:%x",sum_check,data[length-2]);
        return RECV_DATA_XOR_ERR;
    }

    memcpy(data_change,data,IR_S640A_HEADER_LEN);
    //遇到转义字符就进行转换,转换后蒋数据放在新的数组里面
    j = 4;
    for (i = 4; i<(length-2); i++)
    {
        if (data[i] == 0xf5)
        {
            if (data[i+1] == 0x0)
            {
                data_change[j] = 0xf0;
            }
            else if (data[i+1] == 0xf)
            {
                data_change[j] = 0xff;
            }
            else if (data[i+1] == 0x5)
            {
                data_change[j] = 0xf5;
            }
            i++;
        }
        else
        {
            data_change[j] = data[i];
        }
        j++;
    }

    data_change[IR_S640A_RCV_DATA_LEN-1] = data[length];
    data_change[IR_S640A_RCV_DATA_LEN-2] = data[length-1];

    memcpy(&rcv_ir_info,data_change,IR_S640A_RCV_DATA_LEN);

    main_ir_info.crosshair_x = rcv_ir_info.crosshair_x;
    main_ir_info.crosshair_y = rcv_ir_info.crosshair_y;
    main_ir_info.contrast_level = rcv_ir_info.contrast;
    main_ir_info.light_level = rcv_ir_info.light_degree;
    main_ir_info.ir_status.crosshair_on = rcv_ir_info.ir_sta.crosshair;
    main_ir_info.ir_status.black_white = rcv_ir_info.ir_sta.w_b_sta;
    main_ir_info.image_enhance_sta = rcv_ir_info.ir_sta.image_enhance_sta;
    main_ir_info.auto_calibration = rcv_ir_info.ir_sta.auto_cal_sta;

    DEBUG_IR_PRINT("\r\n亮度[%d]\r\n对比度[%d]\r\nDDE[%d]\r\n灰度[%d]\r\n变倍[%d]\r\n调焦[%d]\r\n极性[%c]\r\n探测器温度[%dK]\r\n环境温度[%d℃]\r\n镜头温度[%d℃]",
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

    CONFIG_Set_Ir_Info(main_ir_info);
    return 0;
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
RECV_DATA_ERR_STA IR_API_A640s_Period_Handle(void)
{
    if (ir_s640a_rcv_buf.buf_rcved)
    {
        IR_S640A_PROCESS_Get(ir_s640a_rcv_buf.com_data_buf,ir_s640a_rcv_buf.length);

        IR_S640A_RECBUFF_Clear();

        return RECV_DATA_SUC;
    }
    else
    {
        return RECV_DATA_NULL;
    }
}

#endif
