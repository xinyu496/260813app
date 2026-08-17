#include "Common/config.h"
#if ETH_INCLUDE
#include "Common/drv_tcp.h"
#include "cli/cli_cmd_line.h"
#include "lwip/tcp.h"
typedef struct
{
	uint16_t d_port;
	uint16_t s_port;
	uint32_t ip;
}TCP_CONFIG_T;
typedef struct
{
	uint8_t *data_ptr;
	uint16_t data_len;
	struct tcp_pcb *tcp_server_pcb;
}TCP_SERVER_DATA_T;

#define TCP_SERVER_PEER_MAX		        MEMP_NUM_TCP_PCB
#define TCP_SERVER_HEART_NO_LINK_CNT	(30 * 60)
#define TCP_SERVER_PEER_START			0

struct tcp_pcb *active_connection[TCP_SERVER_PEER_MAX] = {NULL,NULL,NULL,NULL,NULL} ;
static uint32_t active_connection_num[TCP_SERVER_PEER_MAX];
static uint32_t active_connection_now_num = 0;

struct tcp_pcb *tcp_server_pcb;

static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb);
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_conn_err
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	err       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/

static void tcp_server_remove_peer_info( struct tcp_pcb *peer, const char *func, uint32_t line)
{
	uint32_t i = 0 ;

//	if (DBG_Get_PrintState(MODULE_TYPE_INFO))
//	{
//		TCP_SERVER_Dump();
//	}
	printf("\r\n %s, %d, Handle, %s", func, line, __FUNCTION__);

	for (i = TCP_SERVER_PEER_START ;i < TCP_SERVER_PEER_MAX ;i++)
	{
		if (peer == active_connection[i])
		{
			printf(", %02d, 0x%08x, 0x%08x",
				i, (uint32_t)peer, (uint32_t)active_connection[i]);

			active_connection[i] = NULL ;
		}
	}
//	if (DBG_Get_PrintState(MODULE_TYPE_INFO))
//	{
//		TCP_SERVER_Dump();
//	}
}
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_conn_err
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	err       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
 #define tcp_server_remove_peer(pcb) 	tcp_server_remove_peer_info(pcb,  __FUNCTION__, __LINE__)

static void tcp_server_conn_err(void *arg, err_t err)
{
	TCP_SERVER_DATA_T *hs;

	hs = (TCP_SERVER_DATA_T *)arg;

	tcp_server_remove_peer(hs->tcp_server_pcb) ;
#if 0
	tcp_arg(hs->tcp_server_pcb, NULL);
	tcp_sent(hs->tcp_server_pcb, NULL);
	tcp_recv(hs->tcp_server_pcb, NULL);

	if (hs->tcp_server_pcb != NULL)
	{
		tcp_close(hs->tcp_server_pcb);
	}
#endif
	if (hs != NULL)
	{
		mem_free(hs);
	}
}
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_close_conn
 * Description:
 *	N/A
 * Parameters:
 *	pcb       :
 *	hs        :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static void tcp_server_close_conn_info(struct tcp_pcb *pcb, TCP_SERVER_DATA_T *hs, const char *func, uint32_t line)
{
	err_t err;
//	DBG_INFO_Print("\r\n %s, %d, Handle, %s, 0x%08x", func, line, __FUNCTION__, (UI32_T)pcb);

	tcp_server_remove_peer(pcb) ;

	tcp_arg(pcb, NULL);
	tcp_err(pcb, NULL);
	tcp_recv(pcb, NULL);
	tcp_poll(pcb, NULL, 0);

	if (pcb != NULL)
	{
		err = tcp_close(pcb);
	}

#if 1
	if (err != ERR_OK)
	{
		/* error closing, try again later in poll */
		tcp_poll(pcb, tcp_server_poll, 1);

		printf("\r\n %s, %d, Handle, %s, 0x%08x FAILED !!!", func, line, __FUNCTION__, (uint32_t)pcb);
	}
#endif

	if (hs != NULL)
	{
		mem_free(hs);
	}
}
#define tcp_server_close_conn(pcb, hs) 	tcp_server_close_conn_info(pcb, hs, __FUNCTION__, __LINE__)
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_accept
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 *	err       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static bool tcp_server_accept_peer( struct tcp_pcb *peer)
{
	bool ret_val = false ;
	uint32_t i = 0 ;

//	if (DBG_Get_PrintState(MODULE_TYPE_INFO))
//	{
//		TCP_SERVER_Dump();
//	}

	for (i = TCP_SERVER_PEER_START ;i < TCP_SERVER_PEER_MAX ;i++)
	{
		if (NULL == active_connection[i])
		{
			active_connection[i] = peer ;
			active_connection_num[i] = active_connection_now_num++;

			ret_val = true ;
//DBG_INFO_Print
			printf("\r\n %s %d , 0x%08x, %d, %d ",
				__FUNCTION__, __LINE__, (uint32_t)peer, i, active_connection_num[i]);

			break ;
		}
	}

#if 0
	if(!ret_val)//tcp连接已满，关闭最早的连接，用于当前连接
	{
		UI32_T farthest_connection_peer = 0;

		for (i = 0 ;i < TCP_SERVER_PEER_MAX ;i++) //查找最早的连接
		{
			if(active_connection_num[i] < active_connection_num[farthest_connection_peer])
			{
				farthest_connection_peer = i;
			}
		}

		DBG_INFO_Print("\r\n %s %d, 0x%08x, 0x%08x, %d, ",
			__FUNCTION__, __LINE__, (UI32_T)active_connection[i], (UI32_T)peer, farthest_connection_peer);

		i = farthest_connection_peer;
		tcp_server_close_conn(active_connection[i], active_connection[i]->callback_arg); // 关闭连接

		active_connection[i] = peer ;
		active_connection_num[i] = active_connection_now_num++;

		DBG_INFO_Print("%d ", active_connection_num[i]);

		ret_val = TRUE ;
	}
#endif

//	if (DBG_Get_PrintState(MODULE_TYPE_INFO))
//	{
//		TCP_SERVER_Dump();
//	}

	return ret_val ;
}

/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_send_data
 * Description:
 *	N/A
 * Parameters:
 *	pcb       :
 *	hs        :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static void tcp_server_send_data(struct tcp_pcb *pcb, TCP_SERVER_DATA_T *ts)
{
	err_t wr_err = ERR_OK;

#if 0
	while ((wr_err == ERR_OK) &&
	     (ts->data_ptr != NULL) &&
	     (ts->data_len <= tcp_sndbuf(pcb)))
#endif
	{
		/* enqueue data for transmission */
		wr_err = tcp_write(pcb, ts->data_ptr, ts->data_len, 1);
		if (wr_err == ERR_OK)
		{
			/* we can read more data now */
			tcp_recved(pcb, ts->data_len);

#if 0
			ts->data_ptr = NULL;
#endif
		}
		else if(wr_err == ERR_MEM)
		{
			/* we are low on memory, try later / harder, defer to poll */
		}
		else
		{
			/* other problem ?? */
		}
	}
}

/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_poll
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static err_t tcp_server_poll(void *arg, struct tcp_pcb *pcb)
{
	TCP_SERVER_DATA_T *ts;

	ts = (TCP_SERVER_DATA_T *)arg;

//	DBG_INFO_Print("\r\n %s, %d, ts, 0x%08x, tsdata, 0x%08x, tspcb, 0x%08x, pcb, 0x%08x ",
//		__FUNCTION__, __LINE__,
//		(UI32_T)ts,
//		((ts->data_ptr == NULL) ? (UI32_T)0xffffffff : (UI32_T)ts->data_ptr),
//		((ts == NULL) ? (UI32_T)0xffffffff : (UI32_T)ts->tcp_server_pcb),
//		(UI32_T)pcb);

	if (ts == NULL)
	{
		tcp_abort(pcb);

		return ERR_ABRT;
	}
	else if ((ts->data_ptr != NULL) && (ts->data_len > 0))
	{
		tcp_server_send_data(pcb, ts);
	}

	return ERR_OK;
}

/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_recv
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 *	p         :
 *	err       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static err_t tcp_server_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	int i;
	char *data;
	TCP_SERVER_DATA_T *ts;
	uint8_t recv_data[10] = {0};

    ts = (TCP_SERVER_DATA_T *)arg;

	if (err == ERR_OK && p != NULL)
	{
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

//		if (DBG_Get_PrintState(MODULE_TYPE_MON_TCP_RX))
//		{
//			printf("\r\n TCP Recive Data:");

//			//UTL_HEX_Data_Printf(p->payload, p->len);

//			//COM_API_Send_SerialData(COM_CONSOLE, p->payload,  p->len);
//			//接收数据解析
//			
//		}
		memcpy(recv_data,p->payload,10);
		printf("\r\n %x %x %x %x",recv_data[0],recv_data[1],recv_data[2],recv_data[3]);
//		tcp_server_handle_heart(pcb,p->payload);



		pbuf_free(p);
	}

	if (err == ERR_OK && p == NULL)
	{
		tcp_server_close_conn(pcb, ts);
	}

	return ERR_OK;
}
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_accept
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 *	err       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static err_t tcp_server_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
	TCP_SERVER_DATA_T *hs = NULL;

	if (pcb == NULL)
	{
		return ERR_MEM;
	}

	tcp_setprio(pcb, TCP_PRIO_MIN);

	if (false == tcp_server_accept_peer(pcb))
	{
		tcp_server_close_conn(pcb ,NULL);

		return ERR_CONN ;
	}

	/* Allocate memory for the structure that holds the state of the
	connection. */
	hs = (TCP_SERVER_DATA_T *)mem_malloc(sizeof(TCP_SERVER_DATA_T));

	if (hs == NULL)
	{
		tcp_server_close_conn(pcb ,NULL) ;

		return ERR_MEM;
	}

	/* Initialize the structure. */
	hs->data_ptr = NULL;
	hs->data_len = 0;
	hs->tcp_server_pcb = pcb ;

#if LWIP_TCP_KEEPALIVE
	hs->tcp_server_pcb->so_options |= SOF_KEEPALIVE;
	hs->tcp_server_pcb->flags |= TF_NODELAY;
#endif

	/* Tell TCP that this is the structure we wish to be passed for our
		callbacks. */
	tcp_arg(pcb, hs);

	/* Tell TCP that we wish to be informed of incoming data by a call
		to the tcp_server_recv() function. */
	tcp_recv(pcb, tcp_server_recv);

	tcp_err(pcb, tcp_server_conn_err);

//	tcp_poll(pcb, tcp_server_poll, 4);
	tcp_poll(pcb, tcp_server_poll, 1);

//    DBG_INFO_Print("\r\n TCP Client Connect!");

	return ERR_OK;
}
/*-------------------------------------------------------------------
 * Function Name:
 *	tcp_server_sent
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 *	len       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
static void tcp_server_addr_change_handle( uint32_t if_id)
{
	uint32_t i ;

	for (i = TCP_SERVER_PEER_START ;i < TCP_SERVER_PEER_MAX ;i++)
	{
		if (NULL != active_connection[i])
		{
			tcp_server_close_conn(active_connection[i], active_connection[i]->callback_arg) ;
		}
	}
}

/*-------------------------------------------------------------------
 * Function Name:
 *	TCP_SERVER_Change_MonitorPort
 * Description:
 *	N/A
 * Parameters:
 *	N/A
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
void TCP_SERVER_Change_MonitorPort(void)
{
	tcp_server_addr_change_handle(1);

	if (tcp_server_pcb != NULL)
	{
		tcp_close(tcp_server_pcb) ;
	}

	TCP_SERVER_Init();
}

/*-------------------------------------------------------------------
 * Function Name:
 *	TCP_SERVER_Init
 * Description:tcp初始化
 *	N/A
 * Parameters:
 *	arg       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
void TCP_SERVER_Init(void)
{
    TCP_CONFIG_T tcp_info;
    SYS_LOCAL_IP_T if_info;
	
	
	if_info = CONFIG_Get_Current_ip();

    ip4_addr_t addr;
	err_t err;

//    CONFIG_Get_IfInfoById(PHY_MONITOR, &if_info);
    addr.addr = if_info.ip;//源地址赋值

//    tcp_info = CONFIG_Get_TCP_Config();
	tcp_info.s_port = 5010;
    tcp_server_pcb = tcp_new();

#if LWIP_TCP_KEEPALIVE
	tcp_server_pcb->so_options |= SOF_KEEPALIVE;
    tcp_server_pcb->flags |= TF_NODELAY;
#endif

    if (!tcp_server_pcb)
    {
//        DBG_INFO_Print("\r\n Tcp New Failed.");
		printf("\r\n Tcp New Failed.");
    }
    else
    {
		tcp_bind(tcp_server_pcb, IP_ADDR_ANY, tcp_info.s_port);

//		printf("\r\n %x %x %x",active_connection[0],active_connection[1],active_connection[2]);
		
		tcp_server_pcb = tcp_listen(tcp_server_pcb);
 
		tcp_accept(tcp_server_pcb, tcp_server_accept);

//		ETH_API_Register_AddrChange_Handler(tcp_server_addr_change_handle);//注册tcpip更改后，
//		CONFIG_Register_Cfg_Notify(CONFIG_NOTIFY_MONITOR_TCP, TCP_SERVER_Change_MonitorPort);

//	    DBG_INFO_Print("\r\n TCP Server Init.");
	}
}

/*-------------------------------------------------------------------
 * Function Name:
 *	TCP_SERVER_Send
 * Description:
 *	N/A
 * Parameters:
 *	data_ptr  :
 *	data_len  :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
void TCP_SERVER_Send(uint8_t *data_ptr, uint16_t data_len)
{
#if 1
	uint32_t i ;
	TCP_SERVER_DATA_T *ts;

	for (i = TCP_SERVER_PEER_START ;i < TCP_SERVER_PEER_MAX ;i++)
	{
		if (NULL != active_connection[i])
		{
#if 1
			tcp_write(active_connection[i], data_ptr, data_len, 1);

			tcp_output(active_connection[i]);
			
#else
			ts = (TCP_SERVER_DATA_T *)active_connection[i]->callback_arg;

			ts->data_ptr = data_ptr;
			ts->data_len = data_len;
#endif

//			if (DBG_Get_PrintState(MODULE_TYPE_MON_TCP_TX))
//			{
//				printf("\r\n Send Monitor Tcp(%d):", i);
//				UTL_HEX_Data_Printf(data_ptr, data_len);
//			}
		}
	}
#endif
}


/*-------------------------------------------------------------------
 * Function Name:
 *	TCP_SERVER_1ppsHandle
 * Description:
 *	N/A
 * Parameters:
 *	arg       :
 *	pcb       :
 *	len       :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 23.1.2015 by LmnyL, Create.
 *-----------------------------------------------------------------*/
typedef enum
{
	ETH_LINK_UP,
	ETH_LINK_DOWN,
}ETH_PORT_STATUS_T;
void TCP_SERVER_1ppsHandle(void)
{	uint32_t i = 0;
	ETH_PORT_STATUS_T port_status;
static uint8_t ctrl_time = 0;
	if (HAL_GetTick() < (ctrl_time +1000))
	{
		return;
	}	
	ctrl_time = HAL_GetTick();
	ETH_API_Get_PortStatus(i, &port_status);

	if (!ETH_API_Get_LinkStatus(DEFAULT_IF_ID))
	{//Link Down
		for (i = TCP_SERVER_PEER_START;i < TCP_SERVER_PEER_MAX ;i++)
		{
			if (NULL != active_connection[i])
			{
				tcp_server_close_conn(active_connection[i], active_connection[i]->callback_arg) ;
			}
		}
	}
}

#endif