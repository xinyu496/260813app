#include "Bsp/bsp_can.h"




HAL_StatusTypeDef   HAL_RetVal; 
CAN_RxHeaderTypeDef   CAN_RxStruct;

UDID_CanBus UDID_RxCan_Bus,UDID_TxCan_Bus;
uint8_t RX_CANData[8];

uint8_t can_fifo0_txbuff[8];
CAN_TxHeaderTypeDef CAN_TxStruct;
uint32_t tx_mailbox;


/***************canfd***************/
#if CANFD_ENABLE
FDCAN_TxHeaderTypeDef CANFD_TxStruct;	
FDCAN_RxHeaderTypeDef CAN_RxStruct;
#endif


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:过滤器初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CAN_FILTER_Init(void)
{
    CAN_FilterTypeDef CAN_FilterInitStruct0_T;
    
    CAN_FilterInitStruct0_T.FilterBank=14;						            //过滤器14
	CAN_FilterInitStruct0_T.FilterMode=CAN_FILTERMODE_IDMASK;	            //模糊模式
	CAN_FilterInitStruct0_T.FilterScale=CAN_FILTERSCALE_32BIT;	            //
	CAN_FilterInitStruct0_T.FilterIdHigh= ((((uint32_t)0x1314<<3)|
										 CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		
	CAN_FilterInitStruct0_T.FilterIdLow = (((uint32_t)0x1314<<3)|
									     CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF;  
	CAN_FilterInitStruct0_T.FilterMaskIdHigh= 0xFFFF;			           
	CAN_FilterInitStruct0_T.FilterMaskIdLow= 0xFFFF;			           
	CAN_FilterInitStruct0_T.FilterFIFOAssignment=CAN_FILTER_FIFO0 ;	        //接收到的报文放到FIFO0中
	CAN_FilterInitStruct0_T.FilterActivation=ENABLE;			            //激活过滤器
    HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStruct0_T);
	if(HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterInitStruct0_T) != HAL_OK)
    {
#if debug_bsp_can_flag
        SEGGER_RTT_printf(0 , "CAN INIT ERROR !!!\n");
#endif
    }
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:过滤器初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void CANFD_FILTER_Init(void)
{
    FDCAN_FilterTypeDef CAN_FilterInitStruct0_T;
    
    CAN_FilterInitStruct0_T.IdType = FDCAN_EXTENDED_ID;
    CAN_FilterInitStruct0_T.FilterIndex = 0;		
    CAN_FilterInitStruct0_T.FilterType = FDCAN_FILTER_DUAL;
    CAN_FilterInitStruct0_T.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    CAN_FilterInitStruct0_T.FilterID1 = 0x1314;
    CAN_FilterInitStruct0_T.FilterID2 = 0x1314;
    CAN_FilterInitStruct0_T.RxBufferIndex = 0;
    
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN_FilterInitStruct0_T) != HAL_OK)
    {
#if debug_flag
        SEGGER_RTT_printf(0 , "CAN INIT ERROR !!!\n");
#endif       
    }
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:canfd故障中断
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if(hfdcan == &hfdcan1)
    {
#if debug_flag
        SEGGER_RTT_printf(0 , "CAN ERROR !!!\n");
#endif
    }
	//printf("\r\nCAN????\r\n");
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:canfd接收中断
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
    {
        if (HAL_FDCAN_GetRxMessage(&hfdcan1 , FDCAN_RX_FIFO0, &CAN_RxStruct, can_fifo0_rxbuff) == HAL_OK)
        {
#if debug_flag
        SEGGER_RTT_SetTerminal(1);
        SEGGER_RTT_printf(0 , "receive can1 data : " );
        for(int i = 0 ; i < 8 ; i++)
        {
            SEGGER_RTT_printf(0 , "0x%x " , can_fifo0_rxbuff[i]);
        }
        SEGGER_RTT_printf(0 , "\n" );
        SEGGER_RTT_SetTerminal(0);
#endif              
        }
      
    }
}
#endif

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:接收回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan2)
    {
        uint8_t HAL_RetVal = HAL_CAN_GetRxMessage(&hcan2 , CAN_RX_FIFO0 , &CAN_RxStruct , RX_CANData);
#if debug_bsp_can_flag
        SEGGER_RTT_SetTerminal(1);
        SEGGER_RTT_printf(0 , "receive can1 data : " );
        for(int i = 0 ; i < 8 ; i++)
        {
            SEGGER_RTT_printf(0 , "0x%x " , can_fifo0_rxbuff[i]);
        }
        SEGGER_RTT_printf(0 , "\n" );
        SEGGER_RTT_SetTerminal(0);
#endif        
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:故障回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
#if DEBUG_CAN_DRIVER
    uint32_t err = HAL_CAN_GetError(hcan);

    if (err != HAL_CAN_ERROR_NONE)
    {
        SEGGER_RTT_printf(0 , "CAN Error: 0x%08lX\r\n", err);

        if (err & HAL_CAN_ERROR_BOF)
            SEGGER_RTT_printf(0 , "-> Bus Off\r\n");

        if (err & HAL_CAN_ERROR_EPV)
            SEGGER_RTT_printf(0 , "-> Error Passive\r\n");

        if (err & HAL_CAN_ERROR_EWG)
            SEGGER_RTT_printf(0 , "-> Error Warning\r\n");

        if (err & HAL_CAN_ERROR_STF)
            SEGGER_RTT_printf(0 , "-> Stuff Error\r\n");

        if (err & HAL_CAN_ERROR_FOR)
            SEGGER_RTT_printf(0 , "-> Form Error\r\n");

        if (err & HAL_CAN_ERROR_ACK)
            SEGGER_RTT_printf(0 , "-> ACK Error\r\n");

        if (err & HAL_CAN_ERROR_BR)
            SEGGER_RTT_printf(0 , "-> Bit Recessive Error\r\n");

        if (err & HAL_CAN_ERROR_BD)
            SEGGER_RTT_printf(0 , "-> Bit Dominant Error\r\n");

        if (err & HAL_CAN_ERROR_CRC)
            SEGGER_RTT_printf(0 , "-> CRC Error\r\n");
    }
#endif
    /* 清除错误标志 */
    __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_ERRI);

}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:canfd过滤器初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void CANFD_FILTER_Config(void)
{
    FDCAN_FilterTypeDef CAN_FilterInitStruct0_T;
    
    CAN_FilterInitStruct0_T.IdType = FDCAN_EXTENDED_ID;
    CAN_FilterInitStruct0_T.FilterIndex = 0;		
    CAN_FilterInitStruct0_T.FilterType = FDCAN_FILTER_DUAL;
    CAN_FilterInitStruct0_T.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    CAN_FilterInitStruct0_T.FilterID1 = 0x1314;
    CAN_FilterInitStruct0_T.FilterID2 = 0x1314;
    CAN_FilterInitStruct0_T.RxBufferIndex = 0;
    
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &CAN_FilterInitStruct0_T) != HAL_OK)
    {
#if debug_flag
        SEGGER_RTT_printf(0 , "CAN INIT ERROR !!!\n");
#endif       
    }
}
#endif




/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:canfd发送接口
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void CANFD_Send(uint8_t com_can_id , uint8_t *sendbuff , uint32_t id , uint8_t len)
{
    CANFD_TxStruct.Identifier = 0x1314;
    CANFD_TxStruct.IdType = FDCAN_EXTENDED_ID;
    CANFD_TxStruct.TxFrameType = FDCAN_DATA_FRAME;
    CANFD_TxStruct.DataLength = FDCAN_DLC_BYTES_8;
    CANFD_TxStruct.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    CANFD_TxStruct.BitRateSwitch = FDCAN_BRS_OFF;
    CANFD_TxStruct.FDFormat = FDCAN_FD_CAN;
    CANFD_TxStruct.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    CANFD_TxStruct.MessageMarker = 0x01;
    
    if(com_can_id == 1)
    {
        HAL_FDCAN_AddMessageToTxBuffer(&hfdcan1, &CAN_TxStruct, sendbuff, FDCAN_TX_BUFFER0);
        HAL_FDCAN_EnableTxBufferRequest(&hfdcan1, FDCAN_TX_BUFFER0);
    }
    else
    {
        HAL_FDCAN_AddMessageToTxBuffer(&hfdcan2, &CAN_TxStruct, sendbuff, FDCAN_TX_BUFFER0);
        HAL_FDCAN_EnableTxBufferRequest(&hfdcan2, FDCAN_TX_BUFFER0);
    }
    
    
#if debug_bsp_can_flag
        SEGGER_RTT_printf(0 , "SEND can1 data : " );
        for(int i = 0 ; i < 8 ; i++)
        {
            SEGGER_RTT_printf(0 , "0x%x " , TxData0[i]);
        }
        SEGGER_RTT_printf(0 , "\n" );
#endif       
}
#endif

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:can接收初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void COM_Rcv_CanPort_Init(void)
{
   CAN_FILTER_Init();
   HAL_CAN_Start(&hcan2);
	 HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 5, 0);  // ⭐ 错误中断
   HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
   HAL_CAN_ActivateNotification(&hcan2 , CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:canfd接收初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
#if CANFD_ENABLE
void COM_Rcv_CanFDPort_Init(void)
{
   CANFD_FILTER_Config();
   HAL_FDCAN_ActivateNotification(&hfdcan1, 
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | 
        FDCAN_IT_TX_COMPLETE | 
        FDCAN_IT_ERROR_WARNING | 
        FDCAN_IT_ERROR_PASSIVE | 
        FDCAN_IT_BUS_OFF,
        0);
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:can发送接口
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CAN_Send(uint8_t com_can_id , uint8_t *sendbuff , uint32_t id , uint8_t len)
{
    CAN_TxStruct.DLC = len;
    CAN_TxStruct.ExtId = id;
    CAN_TxStruct.IDE = CAN_ID_EXT;
    CAN_TxStruct.RTR = CAN_RTR_DATA;
    
    if(com_can_id == 1)
    {
        HAL_CAN_AddTxMessage(&hcan1 , &CAN_TxStruct , sendbuff , &tx_mailbox);
    }
    else
    {
        HAL_CAN_AddTxMessage(&hcan2 , &CAN_TxStruct , sendbuff , &tx_mailbox);
    }
    
#if debug_bsp_can_flag
        SEGGER_RTT_printf(0 , "send can1 data : " );
        for(int i = 0 ; i < 8 ; i++)
        {
            SEGGER_RTT_printf(0 , "0x%x " , can_fifo0_txbuff[i]);
        }
        SEGGER_RTT_printf(0 , "\n" );
#endif 
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:主动获取can状态信息
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CAN_Diagnose(void)
{
#if DEBUG_CAN_DRIVER
    uint32_t esr = READ_REG(hcan1.Instance->ESR);

    SEGGER_RTT_printf(0 ,"CAN ESR: 0x%08lX\r\n", esr);

    if (esr & CAN_ESR_BOFF)
        SEGGER_RTT_printf(0 , "Bus Off!\r\n");

    if (esr & CAN_ESR_EPVF)
        SEGGER_RTT_printf(0 , "Error Passive\r\n");

    if (esr & CAN_ESR_EWGF)
        SEGGER_RTT_printf(0 , "Error Warning\r\n");
#endif
}


