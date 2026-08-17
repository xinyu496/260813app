#include "APP/xk/xk.h"

//keyStruct XKStruct[13];			//按键结构体数组

///*******************
//function:	循环获取按键状态并执行逻辑（13个）
//time:		251104
//author:		liuxinyu
//para:
//result:
//declare:	
//*********************/
//static void XKLogic_1(void)//搜索模式
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[0]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:	//人工		
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x02;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "mode rengong\n");	
//#endif 
//				break;
//			case 2:	//半自动		
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x02;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "mode banzidong\n");	
//#endif 
//				break;
//			case 3:	//全自动	
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x02;
//					ZKPort.tdata[2] = cnt;	
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "mode quanzidong\n");	
//#endif 
//					cnt = 0;	
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_2(void)//搜索速度
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[1]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:	//1倍		
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x01;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "speed 1\n");	
//#endif 
//				break;
//			case 2:	//2倍		
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x01;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "speed 2\n");	
//#endif 
//				break;
//			case 3:	//3倍	
//					ZKPort.tdata[0] = 0x03;
//					ZKPort.tdata[1] = 0x01;
//					ZKPort.tdata[2] = cnt;	
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;	
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "speed 3\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_3(void)//选择曲线（上）
//{
//	if(keyScan(&XKStruct[2]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x04;
//		ZKPort.tdata[1] = 0x05;
//		ZKPort.tdata[2] = 0x01;
//		memset(ZKPort.tdata+3 , 0 , 8);
//		ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "quxian up\n");	
//#endif 
//	}
//}
//static void XKLogic_4(void)//测试模式
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[3]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:	//功能性测试		
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x05;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "gongnengxingceshi\n");	
//#endif 
//				break;
//			case 2:	//系统校靶		
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x05;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "xitongjiaoba\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_5(void)//待机模式
//{
//	if(keyScan(&XKStruct[4]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x02;
//		ZKPort.tdata[1] = 0x06;
//		ZKPort.tdata[2] = 0x00;
//		memset(ZKPort.tdata+3 , 0 , 8);
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "daijimoshi\n");	
//#endif 
//	}
//}
//static void XKLogic_6(void)//确认
//{
//	if(keyScan(&XKStruct[5]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x04;
//		ZKPort.tdata[1] = 0x07;
//		ZKPort.tdata[2] = 0x00;
//		memset(ZKPort.tdata+3 , 0 , 8);
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "queren\n");	
//#endif 
//	}
//}
//static void XKLogic_7(void)//系统复位
//{
//	if(keyScan(&XKStruct[6]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x02;
//		ZKPort.tdata[1] = 0x03;
//		ZKPort.tdata[2] = 0x02;
//		memset(ZKPort.tdata+3 , 0 , 8);
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "xitongfuwei\n");	
//#endif 
//	}
//}
//static void XKLogic_8(void)//指向复位
//{
//	if(keyScan(&XKStruct[7]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x02;
//		ZKPort.tdata[1] = 0x03;
//		ZKPort.tdata[2] = 0x01;
//		memset(ZKPort.tdata+3 , 0 , 8);
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "zhixiangfuwei\n");	
//#endif 
//	}
//}
//static void XKLogic_9(void)//视场切换
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[8]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:	//1*1		
//					ZKPort.tdata[0] = 0x01;
//					ZKPort.tdata[1] = 0x01;
//					ZKPort.tdata[2] = 0x04;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "1*1\n");	
//#endif 
//				break;
//			case 2:	//10*10		
//					ZKPort.tdata[0] = 0x01;
//					ZKPort.tdata[1] = 0x01;
//					ZKPort.tdata[2] = 0x03;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "10*10\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_10(void)//选择曲线（下）
//{
//	if(keyScan(&XKStruct[9]) == cmdEnable)
//	{
//		ZKPort.tdata[0] = 0x04;
//		ZKPort.tdata[1] = 0x05;
//		ZKPort.tdata[2] = 0x02;
//		memset(ZKPort.tdata+3 , 0 , 8);
//		ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "quxian down\n");	
//#endif 
//	}
//}
//static void XKLogic_11(void)//电十字
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[10]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:	//十字丝显示		
//					ZKPort.tdata[0] = 0x01;
//					ZKPort.tdata[1] = 0x05;
//					ZKPort.tdata[2] = 0x01;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "shizisi display\n");	
//#endif 
//				break;
//			case 2:	//十字丝消隐		
//					ZKPort.tdata[0] = 0x01;
//					ZKPort.tdata[1] = 0x05;
//					ZKPort.tdata[2] = 0x00;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "shizisi no display\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_12(void)//自检模式
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[11]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1://上电自检
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x04;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "shangdianzijian\n");	
//#endif 
//				break;
//			case 2://周期自检
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x04;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "zhouqizijian\n");	
//#endif 
//				break;
//			case 3://启动自检
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x04;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "qidongzijian\n");	
//#endif 
//				break;
//			case 4://维护自检
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x04;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "weihuzijian\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
//static void XKLogic_13(void)//锁定解锁
//{
//	static uint8_t cnt = 0;

//	if(keyScan(&XKStruct[12]) == cmdEnable)
//	{
//		cnt++;
//		switch(cnt)
//		{
//			case 1:
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x07;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "suoding\n");	
//#endif 
//				break;
//			case 2:
//					ZKPort.tdata[0] = 0x02;
//					ZKPort.tdata[1] = 0x07;
//					ZKPort.tdata[2] = cnt;
//					memset(ZKPort.tdata+3 , 0 , 8);
//					ZKCmdSend();
//					cnt = 0;
//#if	DEBUG   
//	SEGGER_RTT_printf(0 , "jiesuo\n");	
//#endif 
//				break;
//			default:
//				break;
//		}
//	}
//}
///*==============================================================
//*FUNCTION NAME:系统性周期性处理
//*DISCRIPTION:对于需要一直监控状态变化，并控制相关变量的处理
//*PARAMETERS:
//*RETURN:
//*N/A
//*NOTES:
//*HISTORY:
//*==============================================================*/
//static void XKLogic(void)
//{
//	XKLogic_1();		//按键扫描
//	XKLogic_2();		//按键扫描
//	XKLogic_3();        //按键扫描
//	XKLogic_4();        //按键扫描
//	XKLogic_5();        //按键扫描
//	XKLogic_6();        //按键扫描
//	XKLogic_7();        //按键扫描
//	XKLogic_8();        //按键扫描
//	XKLogic_9();        //按键扫描
//	XKLogic_10();       //按键扫描
//	XKLogic_11();       //按键扫描
//	XKLogic_12();       //按键扫描
//	XKLogic_13();       //按键扫描
//}






