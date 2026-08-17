#include "Driver/drv_uart.h"
#include "Common/utl_math.h"
#include "Common/config.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
/*波特率说明
VS2030 : 115200

*/
#if VISIBLE_INCLUDE
#include "visible_visca.h"
#include "visible_zoomtable.h"
#define VL_ZOOM_MAX (148.4)
#define VL_ZOOM_MIN (5.2)

#define VISCA_SEND_HEADER_1 0x81
#define VISCA_SEND_HEADER_2 0x01
#define VISCA_SEND_HEADER_3 0x04
#define VISCA_SEND_HEADER_REQ 0x09
#define VISCA_RECV_HEADER_1 0x90

#define VISCA_SEND_DATA_TAIL 0xFF
bool focus_send_sta = false;
bool zoom_send_sta = false;
uint8_t vl_image_saturation[9] = {0x81,0x01,0x04,0x49,0x00,0x00,0x00,0x01,0xFF};  //饱和度01
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:查表函数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
int VISIBLE_binarySearch(const uint16_t* array, int length, uint16_t target) 
{
    int left = 0;
    int right = length - 1;
    int closestIndex = -1;
    uint16_t closestValue = UINT16_MAX;

    while (left <= right) 
    {
        int mid = left + (right - left) / 2;

        if (array[mid] == target) 
        {
            return mid;  // 找到目标值，返回索引
        } 
        else if (array[mid] < target) 
        {
            left = mid + 1;  // 目标值在右半部分
        } 
        else 
        {
            right = mid - 1;  // 目标值在左半部分
        }

        // 更新最接近的值和索引
        if (abs(array[mid] - target) < closestValue) {
            closestValue = abs(array[mid] - target);
            closestIndex = mid;
        }
    }

    return closestIndex;
}


static uint8_t visible_para_ctrl(uint8_t ctrl_type)
{
	uint8_t ctrl_value = 0;
	switch(ctrl_type)
	{
		case ADJUST_ADD:
			ctrl_value = 2;
		break;
		case ADJUST_MINUS:
			ctrl_value = 3;
		break;
		case ADJUST_STOP:
			ctrl_value = 0;
		break;
	
	}
	return ctrl_value;
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
static uint8_t cm8230_cmd = 0;
uint8_t visible_send_buf[10] = {0};
uint8_t Visible_VISCA_Ctrl_SendHandle(uint8_t send_type,uint8_t *ctrl_data)
{
static uint8_t exposure_gear = VL_EXPOSURE_INIT;
static uint8_t saturation_gear = VL_SATURATION_INIT;
static uint8_t image_enhance = VL_IMAGE_ANHANCE_INIT;
static uint8_t vl_light = VL_LIGHT_INIT;
	
	VISIBLE_VISCA_RECV_T rcv_visible_info;
	SYS_VISIBLE_DATA_T sys_vl_info;
	sys_vl_info = CONFIG_Get_Visible_Info();

    uint8_t send_cmd_1 = 0;
    uint8_t send_cmd_2 = 0;
	uint8_t send_num  = 0;
    uint8_t send_buf_temp[4] = {0};
    uint8_t total_len = 0;
	uint32_t index = 0;

    visible_send_buf[0] = VISCA_SEND_HEADER_1;
    visible_send_buf[1] = VISCA_SEND_HEADER_2;
    visible_send_buf[2] = 4;
    total_len = 3;
	cm8230_cmd = send_type;
    memset(send_buf_temp,0,sizeof(send_buf_temp));
    
    switch (send_type)
    {
        case VL_ZOOM_CTRL:
			if (ctrl_data[0] == 0xF)
			{
				visible_send_buf[1] = VISCA_SEND_HEADER_REQ;
				send_cmd_1 = VL_VISCA_VIEW_SET;
				zoom_send_sta = true;
				send_num = 5;
			}
			else
			{
				send_cmd_1 = VL_VISCA_ZOOM;
				send_cmd_2 = visible_para_ctrl(ctrl_data[0]);//0x30（低）~ 0x37（快）相机以可变速度变倍至广角
			}
			break;
    
        case VL_FOCUS_ONE_SET:
            send_cmd_1 = VL_VISCA_FOCUS_AUTO;
            send_cmd_2 = 1;
            break;
    
        case VL_FOCUS_CTRL:
			if (ctrl_data[0] == 0xF)
			{
				visible_send_buf[1] = VISCA_SEND_HEADER_REQ;
				send_cmd_1 = VL_VISCA_FOCUS_REQ;
				focus_send_sta = true;
				send_num = 5;
			}
			else 
			{
				send_cmd_1 = VL_VISCA_FOCUS;
				send_cmd_2 = visible_para_ctrl(ctrl_data[1]);//相机以标准速度变焦至远焦
			}
            break;
    
        case VL_CH_MLARGE:		/*极大视场*/
        case VL_CH_LARGE:			/*大视场*/
        case VL_CH_MI:				/*中视场*/
        case VL_CH_SMA:				/*小视场*/
        case VL_CH_MIN:				/*极小视场*/
            send_cmd_1 = VL_VISCA_VIEW_SET;
            memcpy(send_buf_temp,ctrl_data,sizeof(send_buf_temp));
            send_num = 9;
            break;
    
        case VL_ELE_ZOOM://电子变倍
            send_cmd_1 = VL_VISCA_ZOOM_ELE;
			if (ctrl_data[0] == 1)
			{
				send_cmd_2 = 2;
			}
			else
			{
				send_cmd_2 = 3;
			}
			break;
        case VL_CALI:
            send_cmd_2 = 0;
            send_cmd_1 = VL_VISCA_IMAGE_AUTO;
            break;
        case VL_LIGHT_CTRL:/*亮度*/
			send_cmd_1 = VL_VISCA_LIGHT_CTRL;
			if (ctrl_data[0] == 5)//亮度设置
			{
				send_buf_temp[2] = (ctrl_data[1] & 0xF0) >> 4;
				send_buf_temp[3] = ctrl_data[1] & 0x0F;
				send_num = 9;
			}
			sys_vl_info.light_degree = ctrl_data[1];
            break;

        case VL_IMAGE_ENHANCE:/*增强 锐化设置*/
            send_cmd_1 = VL_VISCA_SHAR;
            if (ctrl_data[0] == 2) // +
            {
                if(image_enhance >= 0xff)
                {
                    image_enhance = 0xff;
                }
                else
                {
                    image_enhance += 0x0A;
                }
            }
            else if (ctrl_data[0] == 3)
            {
                if(image_enhance <= VL_IMAGE_ANHANCE_INIT)
                {
                    image_enhance = VL_IMAGE_ANHANCE_INIT;
                }
                else
                {
                    image_enhance -= 0x0A;
                }
            }
            else if (ctrl_data[0] == 0)//图像增强关
            {
                image_enhance = VL_IMAGE_ANHANCE_INIT;
            }
            //存在设置为任意值的情况，待完善
            send_buf_temp[2] = (image_enhance & 0xF0) >> 4;
			send_buf_temp[3] = image_enhance & 0x0F;
			sys_vl_info.enhance_degree = image_enhance;
            send_num = 9;
        break;
    
        case VL_EXPOSURE:
            send_cmd_1 = VL_VISCA_EXPOSURE_SET;
            if (ctrl_data[0] == CMD_ENABLE)
            {
                if (exposure_gear >= 0xFE)
                {
                    exposure_gear = 0xFE;
                }
                else
                {
                    exposure_gear += 0x0A;
                }
            }
            else if (ctrl_data[1] == CMD_ENABLE)
            {
                
                if (exposure_gear <= 0)
                {
                    exposure_gear = 0;
                }
                else
                {
                    exposure_gear -=0x0A;
                }
            }
			send_buf_temp[2] = (exposure_gear & 0xF0) >> 4;
			send_buf_temp[3] = exposure_gear & 0x0F;
			sys_vl_info.exposure_degree = exposure_gear;
            send_num = 9;
            break;
    
        case VL_EXPOSURE_CTRL:
            if (ctrl_data[0] == 3)
            {
                send_cmd_1 = VL_VISCA_EXPOSURE_SET;
				send_buf_temp[2] = (VL_EXPOSURE_INIT & 0xF0) >> 4;
				send_buf_temp[3] = VL_EXPOSURE_INIT & 0x0F;
                send_num = 9;
            }
            else if (ctrl_data[0] == 1)
            {
                send_cmd_1 = VL_VISCA_EXPOSURE_CTRL;
				send_cmd_2 = 2;
				sys_vl_info.exposure_degree = exposure_gear;
            }
			else if (ctrl_data[0] == 2)
            {
                send_cmd_1 = VL_VISCA_EXPOSURE_CTRL;
				send_cmd_2 = 3;
            }
            break;
#if 1
        case VL_SATURATION_CTRL:
			send_cmd_1 = VL_VISCA_SATURATION;
			if (ctrl_data[0] == 5)  //饱和度设置
            {
				send_buf_temp[3] = ctrl_data[1];
                send_num = 9;
			}
			sys_vl_info.saturation = ctrl_data[1];
        break;

        case VL_CONTRAST_CTRL:
			send_cmd_1 = VL_VISCA_CONTRAST;
			if (ctrl_data[0] == 5)
            {
				send_buf_temp[2] = (ctrl_data[1] & 0xF0) >> 4;
				send_buf_temp[3] = ctrl_data[1] & 0x0F;
                send_num = 9;
			}
			sys_vl_info.contrast_degree = ctrl_data[1];
        break;

        case VL_LIGHT_INHIBITE://强光抑制
            if (ctrl_data[0] == CMD_DISABLE)
            {
                send_cmd_2 = 3;
            }
            else if (ctrl_data[0] == CMD_ENABLE)
            {
                send_cmd_2 = 2;
            }
            send_cmd_1 = VL_VISCA_LIGHT_ENHIBIT;
        break;

#endif
    default:
        return CMD_ERR;
    }
    
    visible_send_buf[total_len] = send_cmd_1;//UTL_ADD_CHECK((uint8_t *)&visible_send_buf.ctrl_cmd,(VISIBLE_SEND_DATA_LEN-4));
    total_len++;
    if (send_num == 9)
    {
        memcpy(&visible_send_buf[total_len],send_buf_temp,sizeof(send_buf_temp));
        total_len += 4;
    }
    else if (send_num == 5)
	{
	
	}
	else
    {
        visible_send_buf[total_len] = send_cmd_2;
        total_len++;
    }
    
    visible_send_buf[total_len] = VISCA_SEND_DATA_TAIL;
    total_len++;


    COM_API_Send_Data(COM_KJG_IN,visible_send_buf,total_len);
#if CLI_INCLUDE
    DEBUG_VL_TX_PRINT("\r\n VL_SEND:");
    DEBUG_VL_TX_PRINT("%02x %02x %02x %02x %02x %02x %02x %02x %02x",
                      visible_send_buf[0],
                      visible_send_buf[1],
                      visible_send_buf[2],
                      visible_send_buf[3],
                      visible_send_buf[4],
                      visible_send_buf[5],
                      visible_send_buf[6],
                      visible_send_buf[7],
                      visible_send_buf[8]);
#endif
    return RECV_DATA_SUC;

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:焦距值和变倍值解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t req_cmd = 0;
static void vl_read_status_process(uint8_t *data)
{
	SYS_VISIBLE_DATA_T sys_vl_info;
	/*编码表和焦距和视场角的关系待完善*/
	static uint16_t VLFocusR2 = 0;
    
	if (req_cmd == 2)
	{
			VLFocusR2 = 0;
			VLFocusR2= data[2]<<12 | data[3]<<8 | data[4]<<4 | data[5];
			if(VLFocusR2 >= (sizeof(ZoomTable)/sizeof (ZoomTable[0])))
			{
				sys_vl_info.zoom_value = (sizeof(ZoomTable)/sizeof(ZoomTable[0])) - 1; 
				sys_vl_info.zoom_value = sys_vl_info.zoom_value*0.01;
			}
			else
			{
				sys_vl_info.zoom_value = ZoomTable[VLFocusR2]*0.01;
			}
	}
	else if (req_cmd == 3)
	{
			sys_vl_info.focus_value = 0;//rcv_visible_info.data_para;
			focus_send_sta = false;
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信协议解析
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
//static uint8_t enter_cnt = 0;
uint8_t VISIBLE_cm8230_process_data_in(uint8_t *data,uint32_t length)
{
		VISIBLE_VISCA_RECV_T rcv_visible_info;
		SYS_VISIBLE_DATA_T sys_vl_info;
#if 0
    if (length > (sizeof(VISIBLE_DATA_RECV_T)))//有8个字节，有12个字节
    {
        return false;
    }
#endif
    memcpy(&rcv_visible_info,data,VISIBLE_RECV_DATA_LEN);
    sys_vl_info = CONFIG_Get_Visible_Info();
	switch(cm8230_cmd)//根据发送执行解析收到的报文，可能会存在错位的情况（但由于实在大循环中执行，应该不会存在错位的情况）
	{
//		case VL_IMAGE_ENHANCE://光圈
//			sys_vl_info.enhance_degree = data[5];
//		break;
//		
//		case VL_VISCA_EXPOSURE_SET: //曝光 亮度
//			sys_vl_info.light_degree = data[5];
//		break;
		
		case VL_STATUS_ASK:
			vl_read_status_process(data);
		break;

//		case VL_VISCA_CONTRAST:
//			sys_vl_info.contrast_degree = (data[2]<<4)|(data[3]);
//		break;
//		case VL_SATURATION_CTRL:
//			sys_vl_info.saturation = (data[2]<<4)|(data[3]);//饱和度
//		break;
		default:

		break;
#if 0
	else if((VLRECEIVE[0] == 0x90)&&(VLRECEIVE[1] == 0x38)&&(VLRECEIVE[2] == 0xFF))
	{
		UserCAM.Initial_Time = HAL_GetTick();
		UserCAM.Initial = 5;
	}
#endif
	} 

    CONFIG_Set_Visible_Info(sys_vl_info);
	return 0;
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
static COM_RECV_INFO_T cm8230_rcv_info;
uint8_t VISIABLE_VISCA_Period_Handle(void)
{
    uint8_t recv_len = 0;

	recv_len = COM_REC_DataAnalysis_nocheck(COM_KJG_VS2030,cm8230_rcv_info.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
//	printf("%x %x %x %x %x %x",vs2030_rcv_info.recv_buf[0],vs2030_rcv_info.recv_buf[1],vs2030_rcv_info.recv_buf[2],vs2030_rcv_info.recv_buf[3],vs2030_rcv_info.recv_buf[4],vs2030_rcv_info.recv_buf[5]);
	VISIBLE_cm8230_process_data_in(cm8230_rcv_info.recv_buf,recv_len); 
    
	return RECV_DATA_SUC;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:串口初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void VISIBLE_VISCA_Serial_Data_Init(void)
{
	cm8230_rcv_info.header1 = VISCA_RECV_HEADER_1;
	cm8230_rcv_info.tail1 = VISCA_SEND_DATA_TAIL;
	cm8230_rcv_info.data_recv_len = 10;
	COM_Rcv_SerialPort_Init(COM_KJG_VS2030, cm8230_rcv_info.header1,cm8230_rcv_info.tail1,cm8230_rcv_info.data_recv_len);
}
#endif
