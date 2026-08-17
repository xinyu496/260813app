/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Common/config.h"
#if CLI_INCLUDE
#include "APP//cli/cli_cmd_line.h"
#include "Driver/drv_uart.h"
#include "Bsp/bsp_uart.h"
#if VISIBLE_INCLUDE
#include "APP/visible/visible_visca/visible_visca.h"
#endif
#include "Common/utl_math.h"
#include "Common/utl_check.h" 
//执行函数初始化，map文件，执行函数初始化

#if LASER_INCLUDE
//#include "../laser/laser_dyc_15a/laser_dyc_15a.h"
//#include "../laser/laser_lrd_25b07/laser_lrd_25b07.h"
//#include "APP/laser/laser_lsp_lrs_0310/laser_lrs_0310f.h"
#include "APP/laser/laser_lrs_0610a/laser_lrs_0610a.h"
#endif

#if IR_CTRL_INCLUDE
#include "../ir/ir_ctrl_api.h"
#endif
int cli_help(void);
int cli_dis(void);
int cli_set(void);
int cli_prt_info(void);
//指令集以及相关测试指令

ShellCmd_t shellcmd;

//调试打印开关
DEBUG_FILE_MAP_T debug_map[DEBUG_INFO_END] = 
{
	/*cmd*/			/*enum*/			/*sta*/
	{"ingd",		DEBUG_GD_INFO,			0},  
	{"extgd", 		DEBUG_EXT_GD,     		0},	
	{"laser",		DEBUG_LASER_INFO,		0},
	{"lzrx",		DEBUG_LASER_RX,			0},
	{"lztx",		DEBUG_LASER_TX,			0},
	{"irinfo",		DEBUG_IR_INFO,			0},
	{"vlinfo",		DEBUG_VL_INFO,			0},
	{"vlrx",		DEBUG_VL_RX,			0},
	{"vltx",		DEBUG_VL_TX,			0},
	{"nxir",		DEBUG_NXIR_INFO,		0},
	{"udp",			DEBUG_UDP_INFO,			0},
	{"masterrx",	DEBUG_UDP_MASTERRX,		0},
	{"mastertx",	DEBUG_UDP_MASTERTX,		0},
	{"trackrx",		DEBUG_UDP_TRACKRX,		0},
	{"tracktx",		DEBUG_UDP_TRACKTX,		0},
};
static char password[] = "zkyc1234";
#if CLI_INCLUDE
#if 1

__packed typedef struct {
    const char *pCmd;
    const char *pHelp;
    uint8_t (*pInit)(void);//初始化函数
    int (*pFun)(void);//执行函数
} COMMAND_S;



const COMMAND_S CLI_Cmd[] = 
{
    /* cmd              cmd help            init func.      	func. */
    {"help",            "display id",       		NULL,       cli_help},
    {"dis",             "dis ipinfo|version|intr",  NULL,       cli_dis},
    {"set",            	"set ip|udp|port|ir|laser ",NULL,       cli_set},
    {"prt",          	"prt on|off info",     		NULL,       cli_prt_info},
};
#endif
static int cli_help(void)
{
	printf("some info");    
	return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:dis指令的相关处理
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
int cli_dis(void)
{
	SYS_TIME_T code_time = {0};
	UDP_CONFIG_T udp_fk_info = {0};
	if(!shellcmd.pSecondPos)
	{
		printf("usage: move x/y/z/yaw\r\n");
		return -1;
	}

	if (NULL != strstr((char *)shellcmd.pSecondPos, "version"))
	{
		code_time = CONFIG_Get_Sys_Time();//获取编译时间和版本号的打印
		printf("\r\n version: v%d.%02d.%02d-%04d%02d%02d %02d:%02d:%02d",
		APP_HARDWARE_VERSION,APP_REWORK_VERSION,APP_SOFTWARE_VERSION,
		code_time.year,
		code_time.month,
		code_time.day,
		code_time.hour,code_time.minute,code_time.second);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "ipinfo"))
	{
		//打印IP和端口号
//		udp_fk_info = CONFIG_Get_Current_Udpfk();
		printf("\r\n Local_ip: %03d.%03d.%03d.%03d DPort: %d SPort:%d",
			((udp_fk_info.dest_ip)&0xff),
			((udp_fk_info.dest_ip>>8)&0xff),
			((udp_fk_info.dest_ip>>16)&0xff),
			((udp_fk_info.dest_ip>>24)&0xff),
			udp_fk_info.dest_port,
			udp_fk_info.s_port);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "intr"))
	{
		COM_Printf_Dump(2);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "commu"))
	{
		COM_Printf_Dump(1);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "cb"))
	{
		COM_Printf_Dump(3);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "cfg"))
	{
		CONFIG_Printf_Dump();
	
	}
	else
	{
		printf("\r\n %s",CLI_Cmd[1].pHelp);
	}
	return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:set指令的相关处理
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t check_buf[14] = {0x11,0x12,0x13,0x14,0x15,0,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
#define IRPixel		12.0
#define IRPNum_H	640
#define IRPNum_V	512
static int cli_set(void)
{
	uint8_t ctrl_cmd;
	uint8_t ctrl_para = 0;
	//解算控制指令
	ctrl_cmd = atoi(shellcmd.pThirdPos);
	ctrl_para = atoi(shellcmd.pFourthPos);

	if (NULL != strstr((char *)shellcmd.pSecondPos, "laser"))   
	{
#if LASER_INCLUDE
		Laser_Ctrl_SendHandle(ctrl_cmd,ctrl_para,1);
#endif
	
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "ir"))  
	{
#if (IR_CTRL_INCLUDE&IR_LGCS123)
//		Infrared_API_Ctrl_SendHandle(IR_LGCS123,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
#if (IR_CTRL_INCLUDE&IR_NX30_150)
//		Infrared_API_Ctrl_SendHandle(IR_NX30_150,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
	
#if (IR_CTRL_INCLUDE&IR_DYBMC_L640C500A)
		Infrared_API_Ctrl_SendHandle(IR_DYBMC_L640C500A,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
		
#if (IR_CTRL_INCLUDE&IR_S640A_C1)
		Infrared_API_Ctrl_SendHandle(IR_S640A_C1,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif		
#if (IR_CTRL_INCLUDE&IR_TWIN612RG2)
		Infrared_API_Ctrl_SendHandle(IR_TWIN612RG2,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif 
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "vl"))  
	{
#if (VISIBLE_INCLUDE&VL_F15_300)
		Visible_API_Ctrl_SendHandle(VL_F15_300,ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
#if (VISIBLE_INCLUDE&VL_VS2030)
		Visible_VS2030_Ctrl_SendHandle(ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
#if (VISIBLE_INCLUDE&VL_LD)
		Visible_LD_Ctrl_SendHandle(ctrl_cmd,(uint8_t *)&ctrl_para);
#endif
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "cmd"))  //模拟上位机下发指令控制跟踪器
	{
//		CONFIG_Set_Master_Ctrl_Cmd(MASTER_PARA_NULL,ctrl_cmd);
//		CONFIG_Set_Master_Ctrl_Sta(MASTER_CTRL_TRACK);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "gd"))
	{
//		INSIDE_GD_Process_Data_Send(0);
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "check"))
	{
		if (NULL != strstr((char *)shellcmd.pThirdPos, "add"))
		{
			uint16_t add_check = UTL_FletCher16(check_buf,sizeof(check_buf));
			printf("\r\n Flet_check :%d %x",add_check,add_check);
		}
		else if (NULL != strstr((char *)shellcmd.pThirdPos, "xor"))
		{
			uint16_t xor_check = UTL_CRC16_CCITT_FALSE(check_buf,sizeof(check_buf));
			printf("\r\n CCITT_check :%d %x",xor_check,xor_check);
		}
		else if (NULL != strstr((char *)shellcmd.pThirdPos, "crc"))
		{
			uint16_t crc_check = UTL_CRC16_CCITT(check_buf,sizeof(check_buf));
			printf("\r\n crc_check :%d %x",crc_check,crc_check);
		}
	} 
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "math"))
	{
		if (NULL != strstr((char *)shellcmd.pThirdPos, "view"))
		{
			float view = UTL_View_Calc(IRPixel,IRPNum_H,ctrl_para);
			printf("\r\n view :%f",view);
		}
		else if (NULL != strstr((char *)shellcmd.pThirdPos, "focus"))
		{
			float focus = UTL_Focus_Calc(IRPixel,IRPNum_H,ctrl_para);
			printf("\r\n focus :%f",focus);
		}
	}
	else if (NULL != strstr((char *)shellcmd.pSecondPos, "arm"))
	{
		bool lmc_sta = CONFIG_Get_LMC_Sta();
		if (CONFIG_Get_LMC_Sta() == true)//if在线校轴打开
		{
			CONFIG_Set_LMC_Sta(false);//在线校轴关闭
		}
		uint8_t temp_buf[12] = {0};
		CONFIG_Set_Master_Ctrl_Para(MASTER_CTRL_SF,temp_buf);
	}

	return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:prt指令的相关处理
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static int cli_prt_info(void)
{
	//打开或关闭相应的打印
	for(uint8_t i = 0; i < 10; i++)
	{
		if (0 != strstr(shellcmd.pThirdPos,debug_map[i].debug_file))
		{
			if (NULL != strstr((char *)shellcmd.pSecondPos, "on")) 
			{
				debug_map[i].debug_sta = true;
			}
			else if (NULL != strstr((char *)shellcmd.pSecondPos, "off"))
			{
				debug_map[i].debug_sta = false;
			}
		}
	}
	return 0;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:分割函数
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
char *split(const char *pBuf)
{
	char *pPos = strchr(pBuf, ' ');//指向目的字符后面的数据首地址
	if(pPos) {
		int len = strlen(pPos);
		int i;

		for(i = 0; i < len; i++) 
		{
			if(*pPos == ' ') 
			{
				pPos++;
			}
			else 
			{
				break;
			}
		}
		if(i == len) 
		{
			pPos = NULL;
		}
	}

	return pPos;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:初始化函数
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void CLI_Cmd_Init(void)
{
	COM_Rcv_SerialPort_Init(4,0,0,0);
}
#if 1
#ifndef MAX
#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif
int32_t shellMatch(char *cmd)
{
	char tmp[32] = {0};
	uint32_t len;

	// 清除所有的参数
	shellcmd.pSecondPos = NULL;
	shellcmd.pThirdPos = NULL;
	shellcmd.pFourthPos = NULL;

	for(uint32_t i = 0; i < sizeof(CLI_Cmd) / sizeof(COMMAND_S); i++) 
	{
		if(CLI_Cmd[i].pCmd != NULL)
		{
			uint32_t len1 = strlen(CLI_Cmd[i].pCmd);
			char *pPos = strchr(cmd, ' ');//指向目的字符后面的数据首地址
			if(pPos) 
			{
				len = pPos - cmd;
			}
			else 
			{
				len = strlen(cmd);
			}
			strncpy(tmp, cmd, MIN(sizeof(tmp) - 1, len));

			if(len1 == len) 
			{
				if(strcmp(CLI_Cmd[i].pCmd, tmp) == 0)
				{
					return i;
				}
			} 
		}
	}
	return -1;
}
#endif
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:周期性执行函数
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t com_cli_recv[20] = {0};
#define COM_COMMON_USE 1
void CLI_Cmd_Period_Handle(void)
{
   //后去输入的字符串进行赋值
	
	uint16_t recv_len = 0;
	DEBUG_DATA_T *debug_ptr = NULL;
#if COM_COMMON_USE
	debug_ptr = COM_REC_Debug_Data();

	if ((debug_ptr->debug_length == 0)||(debug_ptr->debug_length > 20))
	{
#else
	if (0 == COM_REC_Data_Direct(COM_DEBUG_ON,com_cli_recv))
	{
#endif
		return;
	}
#if COM_COMMON_USE
	memcpy(shellcmd.cmd,debug_ptr->debug_data,20);
	debug_ptr->debug_length = 0;
#else
	memcpy(shellcmd.cmd,com_cli_recv,20);
#endif
	//printf("%s",com_cli_recv);

	
//	if (0 != strstr(shellcmd.cmd,"zkyc1234on"))//输入密码正确则进入调试状态
//	{
//		shellcmd.password_sta = true;
//	}
//	
//	if (0 != strstr(shellcmd.cmd,"zkyc1234off"))//否则退出调试
//	{
//		shellcmd.password_sta = false;
//	}
//	
//	if (shellcmd.password_sta == false)
//	{
//		return;
//	}
//	else
	{
		shellcmd.hasCmd = true;
	}
	
	if(shellcmd.hasCmd)
	{
		shellcmd.hasCmd = false;
		//空格前的第一个字符串与MAP文件的cmd进行比对，返回将结果是指令对应的数组id
		int id = shellMatch(shellcmd.cmd);

		if(id < 0) 
		{
			return;/*没有一个匹配的上的，直接返回，退出函数*/
		}
		else
		{
			if(CLI_Cmd[id].pFun) //判断函数是否有，有的话就执行函数
			{
				// 分割参数
				shellcmd.pSecondPos = split(shellcmd.cmd);
				if(shellcmd.pSecondPos)
				{
					shellcmd.pThirdPos = split(shellcmd.pSecondPos);
					if(shellcmd.pThirdPos)
					{
						shellcmd.pFourthPos = split(shellcmd.pThirdPos);
					}
				}
				// 调用map对应的执行函数
				int ret = CLI_Cmd[id].pFun();
				
				if(!ret)
				{
					//shell_printf("\r\n<OK>\r\n");
				}
				else
				{
					//shell_printf("\r\n<ERROR>\r\n");
				}
			}
			else 
			{
				//执行函数没有，就退出函数
				return;
			}
		}
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:
*PARAMETERS:中断数据解析
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static bool ctrl_debug_sta = false;
#if 0
void CLI_Rx_Handle(uint8_t *rx_buff,uint16_t rx_len) 
{
#if 0
	//待完善
	if (0 != strstr(rx_buff,password))
	{
		//shellcmd.hasCmd = false;
		ctrl_debug_sta = false;
	}
	else
	{
		ctrl_debug_sta = true;
	}
	
	if (ctrl_debug_sta)
#endif
	{
		shellcmd.hasCmd = true;
		memcpy(shellcmd.cmd,rx_buff,rx_len);
	}
}
#endif
#endif
#endif
