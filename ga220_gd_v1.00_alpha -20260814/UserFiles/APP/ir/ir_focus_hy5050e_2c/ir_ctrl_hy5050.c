#include "stm32f4xx_hal.h"
#include "string.h"
//#include "utl_rcv_in.h"
//#include "Common/com_api.h"
#include "Common/config.h"
//#include "../ir/ir_ctrl.h"

#if 0//IR_NX30_150
#define IR_DATA_SEND_HEADER_1 0xAA
#define IR_DATA_SEND_HEADER_2 0x01
#define IR_DATA_RECV_HEADER_2 0x56
#define IR_SEND_DATA_TAIL 0xCC

#define IR_IN_DATA_MAX_INDEX	10

IR_DATA_SEND_TYPE IR_DATA_SEND;
IR_CTRL_PARA_TYPE IR_CTRL_PARA;

static UTL_DATA_RCV_T ir_serial_rcv;
static DATA_BUF_INFO_T ir_serial_rcv_buf[IR_IN_DATA_MAX_INDEX];//最后中断进来的啊博文就存在这里面，不需要状态位的切换。
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:每一句数据处理完之后，要对com_serial_rcv_buf进行清理
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool IR_HY5050_RECBUFF_Clear(bool release_buf,uint32_t index)
{
    if (index < IR_IN_DATA_MAX_INDEX)
    {
        if (release_buf)
        {
            ir_serial_rcv_buf[index].in_use = false;
        }
        else
        {
            ir_serial_rcv_buf[index].in_use = true;
        }
        ir_serial_rcv_buf[index].buf_rcved = false;
        ir_serial_rcv_buf[index].length = 0;
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:准备下一个BUF进行语句接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool IR_RECBUFF_Reset(void)
{
#if 1
    uint32_t index = 0;
    for(index = 0; index < IR_IN_DATA_MAX_INDEX; index++)
    {
        if (ir_serial_rcv_buf[index].in_use != true)
        {
            break;
        }
    }
    if (index >= IR_IN_DATA_MAX_INDEX)
    {
        /*溢出*/
        ir_serial_rcv.current_buf_info_index = -1;
        return false;
    }
    ir_serial_rcv.current_buf_info_index = index;
    ir_serial_rcv.buf_info = &ir_serial_rcv_buf[index];

    IR_HY5050_RECBUFF_Clear(false,index);
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收结束
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool ir_data_in_rcv_finish(uint8_t *data_ptr,uint32_t len)
{
#if 1
    IR_RECBUFF_Reset();
#endif
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据进来之后，准备新的BUF处理数据
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
bool IR_IN_Data_in_rcv_cb(uint8_t data)
{
    if (ir_serial_rcv.current_buf_info_index == -1)
    {
        IR_RECBUFF_Reset();
    }
    
    return Utl_Phase_Data_Rcv_2head_0tail(&ir_serial_rcv,data);
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
void IR_API_Serial_Data_Init(void)
{
    memset(&ir_serial_rcv,0x0,sizeof(ir_serial_rcv));
    memset(&ir_serial_rcv_buf,0x0,sizeof(ir_serial_rcv_buf));

    ir_serial_rcv.header_1 = IR_DATA_SEND_HEADER_1;
    ir_serial_rcv.header_2 = IR_DATA_RECV_HEADER_2;
    ir_serial_rcv.handle = ir_data_in_rcv_finish;
    ir_serial_rcv.tail1 = IR_SEND_DATA_TAIL;
    ir_serial_rcv.max_size = IR_RECV_DATA_LEN;
    ir_serial_rcv.rcv_status = 0;

    ir_serial_rcv.current_buf_info_index = -1;

    IR_RECBUFF_Reset();
    //注册串口接收数据
    COM_API_register_RcvCb(COM_IR_IN,IR_IN_Data_in_rcv_cb);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:send_type:指令类型
			send_data1:控制数据1
			send_data2：控制数据2
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
IR_DATA_CTRL_T ir_send_buf;
uint8_t IR_PROCESS_SendHandle(IR_DATA_SEND_TYPE send_type,uint8_t *data,uint8_t data_len)
{
    memset(&ir_send_buf,0x0,IR_CTRL_DATA_LEN);

    ir_send_buf.Header1 = IR_DATA_SEND_HEADER_1;
    ir_send_buf.Header2 = IR_DATA_SEND_HEADER_2;
    ir_send_buf.ctrl_cmd = send_type;
    memcpy(&ir_send_buf.ctrl_data1,data,data_len);
    ir_send_buf.tail_1 = IR_SEND_DATA_TAIL;
    ir_send_buf.tail_2 = 0;

    return COM_API_Send_Data(COM_IR_IN,(uint8_t *)&ir_send_buf,sizeof(ir_send_buf));
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
void IR_PROCESS_Get(uint8_t *data,uint32_t length)
{
//    IR_DATA_RECV_T rcv_ir_info;
//    SYS_IR_STA_T main_ir_info;
//    memcpy(&main_ir_info.ir_status,&rcv_ir_info.ir_status,1);
//    main_ir_info.light_level = rcv_ir_info.light_level;
//    main_ir_info.contrast_level = rcv_ir_info.contrast_level;
//    main_ir_info.DDE_level = rcv_ir_info.enhancement;
//    main_ir_info.itr_range = rcv_ir_info.itr_range;
//    main_ir_info.integration_time = UTL_Htons(rcv_ir_info.integration_time);
//    main_ir_info.crosshair_x = UTL_Htons(rcv_ir_info.crosshair_x);
//    main_ir_info.crosshair_y = UTL_Htons(rcv_ir_info.crosshair_y);
//    main_ir_info.work_time = UTL_Htonl(rcv_ir_info.work_time);
//    main_ir_info.startup_cnt = UTL_Htons(rcv_ir_info.startup_cnt);
//    main_ir_info.detect_temp = UTL_Htons(rcv_ir_info.detect_temp);
//    main_ir_info.ambient_temp = UTL_Htons(rcv_ir_info.ambient_temp);
//    main_ir_info.grayscale = UTL_Htons(rcv_ir_info.grayscale);
//    main_ir_info.zoom_position = UTL_Htons(rcv_ir_info.zoom_position);
//    main_ir_info.focus_psition = UTL_Htons(rcv_ir_info.focus_psition);
//    main_ir_info.lens_temp = rcv_ir_info.lens_temp;

//    CONFIG_Set_Ir_Info(main_ir_info);
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
uint8_t IR_API_Period_Handle(void)
{
    int i;

    for(i = 0; i < IR_IN_DATA_MAX_INDEX; i++)
    {
        if (ir_serial_rcv_buf[i].buf_rcved)
        {
            *(ir_serial_rcv_buf[i].com_data_ptr + ir_serial_rcv_buf[i].length) = '\0';

            IR_PROCESS_Get(ir_serial_rcv_buf[i].com_data_ptr,ir_serial_rcv_buf[i].length);

            IR_HY5050_RECBUFF_Clear(true,i);

            return RECV_DATA_SUC;
        }
        else
        {
            return RECV_DATA_NULL;
        }
    }
    
    return RECV_DATA_SUC;
}
#endif
