#include "Common/config.h"
#include "APP/sf/sf_ctrl.h"
#include "bsp/bsp_uart.h"
#include "Driver/drv_uart.h"
#include "APP/motor/motor_ctrl.h"
#include "Bsp/SEGGER_RTT.h"
#include "APP/sf/SF.h"

COM_RECV_INFO_T motor_serial_rcv_buf;
uint32_t Bmq_tmp = 0;
userTxMotorDrvParTypeDef userTxMotorDrvPar;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:初始化数据接收的结构体
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MOTOR_API_Serial_Data_Init(void)
{
	motor_serial_rcv_buf.header1 = 0x55;//发送方
	motor_serial_rcv_buf.header2 = 0;//接收方
	motor_serial_rcv_buf.data_recv_len = 7;//接收数据长度
	motor_serial_rcv_buf.tail1 = 0;//接收帧尾1
	motor_serial_rcv_buf.tail2 = 0;//接收帧尾2
	userTxMotorDrvPar.Current_Par = 0;
	userTxMotorDrvPar.Enable = 0;
	//初始化接收数据环
	COM_Rcv_SerialPort_Init(COM_MOTOR, (motor_serial_rcv_buf.header1<<8|motor_serial_rcv_buf.header2),
	(motor_serial_rcv_buf.tail1<<8|motor_serial_rcv_buf.tail2),motor_serial_rcv_buf.data_recv_len);
	
}


//与无刷驱动通讯处理
uint8_t rxDrv_Flag = 1;
uint8_t DrvBord_huart_rxbuf[32];
uint8_t DrvBord_huart_txbuf[10];
int16_t motor_i;//驱动电流
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:驱动板数据发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
extern uint32_t FYBMQ_Data; //变量赋值给gdbmqdispose()使用;
uint32_t FYBMQ_Data_SF;
void Motor_Send_Handler(uint8_t send_flag,uint8_t ctrl_type,uint8_t *send_para) 
{
//	    uint8_t send_flag = 1;
//     Bmq_tmp = 0xFFFFFF&((0xFFFFF- FYBMQ_Data)<<4);
	// Bmq_tmp = FYBMQ_Data<<4;
   if(FYBMQ_Data >= 25359)
   {
   	FYBMQ_Data_SF = (FYBMQ_Data-25359)<<4;
   }
   else 
   {
	   FYBMQ_Data_SF = (FYBMQ_Data+0xFFFFF-25359)<<4;
   }
	Bmq_tmp = FYBMQ_Data_SF;
	if(send_flag == 1)//电机驱动配置
	{
			DrvBord_huart_txbuf[0] = 0x55;
			DrvBord_huart_txbuf[1] = 0x00;//电流Int16分辨率6.25mA
			DrvBord_huart_txbuf[2] = 0x00;
			DrvBord_huart_txbuf[3] = Bmq_tmp;//外方位编码器
			DrvBord_huart_txbuf[4] = Bmq_tmp>>8;
			DrvBord_huart_txbuf[5] = Bmq_tmp>>16;
			DrvBord_huart_txbuf[6] = ctrl_type;//电机驱动配置指令
			DrvBord_huart_txbuf[7] = send_para[0];//参数
			DrvBord_huart_txbuf[8] = send_para[1];
			DrvBord_huart_txbuf[9] = 0x00;//校验和
			
			send_flag = 0;
	}
	else if (send_flag == 0)
	 {
			DrvBord_huart_txbuf[0] = 0x55;
			DrvBord_huart_txbuf[1] = userTxMotorDrvPar.Current_Par;//电流Int16分辨率0.001mA
			DrvBord_huart_txbuf[2] = userTxMotorDrvPar.Current_Par>>8;
			DrvBord_huart_txbuf[3] = Bmq_tmp;//编码器
			DrvBord_huart_txbuf[4] = Bmq_tmp>>8;
			DrvBord_huart_txbuf[5] = Bmq_tmp>>16;
			DrvBord_huart_txbuf[6] = userTxMotorDrvPar.Enable;//电机驱动软件使能
			DrvBord_huart_txbuf[7] = 0x00;//参数
			DrvBord_huart_txbuf[8] = 0x00;
			DrvBord_huart_txbuf[9] = 0x00;//校验和
		 
		 	//Tx_Drv_cnt++;
		}
	 		for(uint8_t i=1; i<9; i++)
		{
			DrvBord_huart_txbuf[9] += DrvBord_huart_txbuf[i];
		}
#if COM_MOTOR_cmd_debug
	SEGGER_RTT_SetTerminal(1);
	SEGGER_RTT_printf(0 , "COM_MOTOR send:" );
	for(uint8_t k = 0; k < sizeof(DrvBord_huart_txbuf) ;k++)
	{
		
		SEGGER_RTT_printf(0 , "0x%x " , DrvBord_huart_txbuf[k]);	
		if(k == sizeof(DrvBord_huart_txbuf) - 1)
		{
			SEGGER_RTT_printf(0 , " \n" );
		}
	}
	SEGGER_RTT_SetTerminal(0);
#endif	
	COM_API_Send_Data(COM_MOTOR,(uint8_t *)&DrvBord_huart_txbuf,sizeof(DrvBord_huart_txbuf));
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:数据接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
userRxMotorDrvParTypeDef motor_send_info;
userRxMotorCfgParTypeDef motor_read_back;
uint32_t Rx_Drv_cnt = 0,	Rx_Drv_cnt1 = 0;
void RX_Motor_Data(uint8_t *Data,uint8_t Size)
{
	Rx_Drv_cnt1++;
	if(Data[0] == 0x55)
	{
		TxMCData.FYSelfChek.Self = 1;
		Rx_Drv_cnt++;
		motor_send_info.Head = Data[0];//数据头
		motor_send_info.Current = Data[1];//电流
		motor_send_info.Current |= Data[2]<<8;//电流
		motor_send_info.DrvSta = Data[3];//驱动状态
		motor_send_info.ParIde = Data[4];//参数标识
		motor_send_info.ParVal = Data[5];//参数值
		motor_send_info.ParVal |= Data[6]<<8;//参数值
		motor_send_info.Sum = Data[7];//校验和

		if((motor_send_info.ParIde & 0xF0) == 0xF0)//驱动板发送配置信息
		{
			motor_read_back.CfgCmdBac_flg = 1;
			switch(motor_send_info.ParIde)
			{
				case 0xF3://读取比例系数
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.Proportion = motor_send_info.ParVal;					
					break;
				case 0xF5://读取积分系数
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.Integral = motor_send_info.ParVal;					
					break;
				case 0xF7://读取电机极对数
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.PoleNum = motor_send_info.ParVal;					
					break;
				case 0xF8://查询寻零结果
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.SeekSequent = motor_send_info.ParVal;					
					break;
				case 0xFA://读取饱和输出
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.SaturationOut = motor_send_info.ParVal;					
					break;
				case 0xFC://读取对齐电压
					motor_read_back.CfgCmdBac = motor_send_info.ParIde;
					motor_read_back.AlignVoltage = motor_send_info.ParVal;					
					break;
				default:
					break;
			}
		}
	}
	else 
	{
		TxMCData.FYSelfChek.Self = 0;	
	}
}
//获取当前电流
int16_t Motor_Get_Current(void)
{
	return motor_send_info.Current;
}

void Motor_Set_Current(userTxMotorDrvParTypeDef drv_info)
{
	userTxMotorDrvPar.Current_Par = drv_info.Current_Par;
	userTxMotorDrvPar.Enable = drv_info.Enable;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:鎺ユ敹鍚庣殑鏁版嵁澶勭悊
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t MOTOR_API_Period_Handle(void)
{
	uint8_t recv_len = 0;
	recv_len = COM_REC_Data_Direct(COM_MOTOR,motor_serial_rcv_buf.recv_buf);
	if (recv_len == 0)
	{
		return RECV_DATA_NULL;
	}
    RX_Motor_Data(motor_serial_rcv_buf.recv_buf,recv_len);
    return RECV_DATA_SUC;
}

