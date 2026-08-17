#include "Common/config.h"
#include "Driver/drv_udp.h"
#if CLI_INCLUDE
#include "APP/cli/cli_cmd_line.h"
#endif
#if 1
#include "lwip.h"
#include "udp.h"

extern struct netif gnetif;
uint8_t __attribute__((at(0x20006800))) udp_demo_recvbuf[256]= {0};	//UDP接收数据缓冲区 //__attribute__((at(0x20006800)))
uint8_t udp_demo_sendbuf[256]= {0}; //UDP发送数据内容

struct udp_pcb *udppcb;
struct udp_pcb *track_udppcb;
struct udp_pcb *fiber_udppcb;
//UDP服务器回调函数
uint8_t Rx_flg = 0;
uint32_t rxremote_ip = 0;
uint32_t temote_port = 0;
uint32_t RxUdp, RxUdp2 = 0;
ip4_addr_t server_ipaddr;
err_t err;
#define EthRX_Len	25

#if 1
#if 0
typedef struct
{
    struct udp_pcb *udp_server;
    UDP_CONFIG_T cli_udp_cfg;
} UDP_PCB_T;
#endif
uint8_t monitor_udp_input_buf[128] = {0};
UDP_PCB_T track_drv_udp;//master
UDP_PCB_T fiber_drv_udp;
UDP_PCB_T master_drv_udp;

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:注册网络报文接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_recv_fk_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{

    uint16_t payload_len;
    uint8_t *msg_in = p->payload;

    if (pcb != track_drv_udp.udp_server)
    {
        pbuf_free(p);
        return;
    }

    if (pbuf_header(p,UDP_HLEN))
    {
        pbuf_free(p);
        return;
    }
    payload_len = p->tot_len-UDP_HLEN;
    if (payload_len < sizeof(monitor_udp_input_buf))
    {
#if 0
		fk_drv_udp.cli_udp_cfg.dest_port = 8080;
		fk_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;
#endif
        //TRACK_Recv_Data_Process(msg_in,payload_len);
    }
    pbuf_free(p);
    return;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:注册网络报文接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_recv_fiber_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    uint16_t payload_len;
    uint8_t *msg_in = p->payload;

    if (pcb != fiber_drv_udp.udp_server)
    {
        pbuf_free(p);
        return;
    }

    if (pbuf_header(p,UDP_HLEN))
    {
        pbuf_free(p);
        return;
    }
    payload_len = p->tot_len-UDP_HLEN;
    if (payload_len < sizeof(monitor_udp_input_buf))
    {
#if 0
        fiber_drv_udp.cli_udp_cfg.dest_port = 8081;
        fiber_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;
#endif
       // Fiber_Recv_Data_Process(msg_in,payload_len);//处理接收到的数据内容
    }
    pbuf_free(p);
    return;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:注册主控网络报文接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_recv_master_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    uint16_t payload_len;
    uint8_t *msg_in = p->payload;
	static uint8_t recv_cnt = 0;

    if (pcb != master_drv_udp.udp_server)
    {
        pbuf_free(p);
        return;
    }

//    if (pbuf_header(p,UDP_HLEN))
//    {
//        pbuf_free(p);
//        return;
//    }
    payload_len = p->tot_len-UDP_HLEN;
    if (payload_len < sizeof(monitor_udp_input_buf))
    {
#if 0
        master_drv_udp.cli_udp_cfg.dest_port = 8082;
        master_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;
#endif
		//接收数据处理
//        MASTER_Process_data_in_from_fcu(msg_in,payload_len);
		recv_cnt++;
		
    }
    pbuf_free(p);
    return;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络报文初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
struct udp_pcb *UDP_SERVER_API_Init( u16_t port,uint32_t des_ip,uint32_t des_port,void (*recv)(void *arg,struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr,u16_t port))
{
    struct udp_pcb*udp_pcb_ptr = NULL;
    err_t err;
    udp_pcb_ptr = udp_new();
    if (udp_pcb_ptr != NULL)
    {
        err = udp_bind(udp_pcb_ptr,IP_ADDR_ANY,port);//绑定本地IP地址和端口号  //49301
        if (ERR_OK == err)
        {
            err = udp_connect(udp_pcb_ptr,(ip_addr_t*)&des_ip,des_port);////UDP客户端连接到的目的端口和Ip
            if(err == ERR_OK)
            {
                udp_recv(udp_pcb_ptr,recv,(void*)port);//注册接收回调函数
            }
        }
        else
        {
            DEBUG_UDP_PRINT("udp_init_err");
        }
    }
    else
    {
        DEBUG_UDP_PRINT("udp_not_clean");
    }
    return udp_pcb_ptr;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:udp初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void MONITOR_UDP_Config_Init(void)
{
    UDP_CONFIG_T udp_cfg_fk = {0};
    UDP_CONFIG_T udp_cfg_fiber = {0};;
    UDP_CONFIG_T udp_cfg_master = {0};;

    memset(&track_drv_udp,0,sizeof(track_drv_udp));
    memset(&fiber_drv_udp,0,sizeof(fiber_drv_udp));
    memset(&master_drv_udp,0,sizeof(master_drv_udp));
    //同时开三个pcb 以便接收与发送
    //与飞控的UDP通信
    udp_cfg_fk = CONFIG_Get_Current_Udpfk();
    track_drv_udp.udp_server =  UDP_SERVER_API_Init(udp_cfg_fk.s_port,udp_cfg_fk.dest_ip,udp_cfg_fk.dest_port,udp_demo_recv_fk_cb);

    if (track_drv_udp.udp_server == NULL)
    {
        DEBUG_UDP_PRINT("port err");
    }
    else
    {
        track_drv_udp.cli_udp_cfg.dest_port = 8080;/*default port*/
        track_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;/*default ip*/
    }
    //与光纤的UDP通信
    udp_cfg_fiber = CONFIG_Get_Current_Udpfiber();
    fiber_drv_udp.udp_server =  UDP_SERVER_API_Init(udp_cfg_fiber.s_port,udp_cfg_fiber.dest_ip,udp_cfg_fiber.dest_port,udp_demo_recv_fiber_cb);
    if (fiber_drv_udp.udp_server == NULL)
    {
        DEBUG_UDP_PRINT("port err");
    }
    else
    {
        fiber_drv_udp.cli_udp_cfg.dest_port = 8081;/*default port*/
        fiber_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;/*default ip*/
    }
    //与上位机的UDP通信
    udp_cfg_master = CONFIG_Get_Current_Udpmaster();
    master_drv_udp.udp_server =  UDP_SERVER_API_Init(udp_cfg_master.s_port,udp_cfg_master.dest_ip,udp_cfg_master.dest_port,udp_demo_recv_master_cb);
    if (master_drv_udp.udp_server == NULL)
    {
        DEBUG_UDP_PRINT("port err");
    }
    else
    {
        master_drv_udp.cli_udp_cfg.dest_port = 8082;/*default port*/
        master_drv_udp.cli_udp_cfg.dest_ip = 0xffffffff;/*default ip*/
    }
}
#else
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络报文的接收
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    uint16_t data_len = 0;
    struct pbuf *q;

    if(p!=NULL)	//接收到不为空的数据时
    {
        memset(udp_demo_recvbuf,0,EthRX_Len);  //数据接收缓冲区清零
        for(q=p; q!=NULL; q=q->next) //遍历完整个pbuf链表
        {
            //判断要拷贝到UDP_DEMO_RX_BUFSIZE中的数据是否大于UDP_DEMO_RX_BUFSIZE的剩余空间，
            //如果大于的话就只拷贝UDP_DEMO_RX_BUFSIZE中剩余长度的数据，否则的话就拷贝所有的数据
            if(q->len > (EthRX_Len-data_len))
            {
                memcpy(udp_demo_recvbuf+data_len,q->payload,(EthRX_Len-data_len));//拷贝数据
            }
            else
            {
                memcpy(udp_demo_recvbuf+data_len,q->payload,q->len);
            }
            data_len += q->len;
            if (pcb == udppcb)
            {
                MASTER_Process_data_in_from_fcu((uint8_t *)&udp_demo_recvbuf,data_len);
            }
            else if  (pcb == track_udppcb)
            {
                TRACK_Recv_Data_Process((uint8_t *)&udp_demo_recvbuf,data_len);
            }
            else if  (pcb == fiber_udppcb)
            {
                Fiber_Recv_Data_Process((uint8_t *)&udp_demo_recvbuf,data_len);
            }
            if(data_len > EthRX_Len)
            break;
        }

        //if(pcb == udppcb)
        pbuf_free(p);//释放内存
    }
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void Eth_NetWork_Handle(void)
{
    static uint32_t user_internet_systick;
    if(HAL_GetTick() >= user_internet_systick + 2)
    {
        user_internet_systick = HAL_GetTick();

        MX_LWIP_Process();//裸机下轮询处理LWIP
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:网络报文的发送
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void udp_demo_senddata(struct udp_pcb *upcb,uint8_t *data,uint16_t Len)
{
    memset(udp_demo_sendbuf,0,sizeof(udp_demo_sendbuf));
    struct pbuf *ptr;
    ptr=pbuf_alloc(PBUF_TRANSPORT,Len,PBUF_POOL); //申请内存
    if(ptr != NULL)
    {
        memcpy(udp_demo_sendbuf,data,Len);
        ptr->payload=(void*)udp_demo_sendbuf; //填充缓冲区数据

        udp_send(upcb,ptr);	//udp发送数据
        pbuf_free(ptr);//释放内存
    }
}
#endif