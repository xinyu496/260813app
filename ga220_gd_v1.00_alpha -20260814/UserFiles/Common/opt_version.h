
/**
  ******************************************************************************
  * @file    com.h
  * @brief   This file contains all the function prototypes for
  *          the com.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OPT_VERSION_H
#define __OPT_VERSION_H

/*=================编译时间配置=========================*/
#define YEAR ((((__DATE__ [7] - '0')*10 +(__DATE__ [8] - '0'))*10 + (__DATE__ [9] - '0'))*10 +(__DATE__ [10] - '0'))
#define MONTH ( __DATE__[2] == 'n'?(__DATE__ [1]=='a'?1:6)\
:__DATE__ [2]=='b'?2\
:__DATE__ [2]=='r'?(__DATE__ [0]=='M'?3:4)\
:__DATE__ [2]=='y'?5\
:__DATE__ [2]=='n'?6\
:__DATE__ [2]=='l'?7\
:__DATE__ [2]=='g'?8\
:__DATE__ [2]=='p'?9\
:__DATE__ [2]=='t'?10\
:__DATE__ [2]=='v'?11:12)
#define DAY ((__DATE__ [4]==' '?0:((__DATE__ [4]-'0')*10)) + (__DATE__ [5]-'0'))
#define HOUR ((__TIME__ [0]==' '?0:((__TIME__ [0]-'0')*10)) + (__TIME__ [1]-'0'))
#define MINUTE ((__TIME__ [3]==' '?0:((__TIME__ [3]-'0')*10)) + (__TIME__ [4]-'0'))
#define SECOND ((__TIME__ [6]==' '?0:((__TIME__ [6]-'0')*10)) + (__TIME__ [7]-'0'))
//编译日期；例：20251230
#define DATE_INT (YEAR *10000 + MONTH *100 + DAY)
//编译具体时间；例：00123013-12时30分13秒
#define TIME_INT (HOUR * 10000 + MINUTE*100 +SECOND)
/*=================软件平台版本号=========================*/
#define SOFTWARE_SPLAT_VERSION		"v1.00.02"
/*=================设备信息配置=========================*/
typedef enum
{
	SOFTWARE_ARM_MC = 0xA1,
	SOFTWARE_ARM_SF = 0xA2,
	SOFTWARE_TRACK = 0xA4,
	SOFTWARE_FPGA = 0xA5,
	SOFTWARE_DRIVER = 0xA6,
	SOFTWARE_ARM_SFFW = 0xA7,
	SOFTWARE_ARM_SFFY = 0xA8,
	SOFTWARE_ARM_SFHG = 0xA9,
	SOFTWARE_ARM_UPPER = 0xAA,
	SOFTWARE_STORAGE = 0xAB,/*存储仪*/
}SOFTWARE_ID_E;
#define APP_SOFT_NUM		SOFTWARE_ARM_SFFY//软件配置项编号
#define NAME_CARD_ID		2501002//铭牌编号
#define HW_DEVICE_ID		2501002//设备编号
#define PROJECT_NUMBER		"AOES-AA43-2601"
/*
版本号统一为3段，首字母小写v1.00.00；
第一段：v1，当硬件发生变更时，改变该字段，如v1.00.00—>v2.00.00;
第二段：00，当设备发生需求变更、设备在外场出现bug，需要外场联调、维修、返厂维修时，变更该字段，如v1.00.00—>v1.01.00；
第三段：00，当设备在设计、生产、测试期间发生的自升级，变更该字段，如v1.00.00—>v1.00.01；
注意：版本号的产生，应在当前阶段测试未发现新问题之后。
测试阶段应当使用特殊标识以区分正式版本，1. Alpha版本：内部测试版（如v1.00.01.a.1）；2． Beta版本：外场测试版（如v1.00.01.b.2）。
*/
/*=================版本号配置=========================*/
#define APP_HARDWARE_VERSION  1 		/*硬件版本*/
#define APP_REWORK_VERSION	  0			/*入过受控库之后的版本*/
#define APP_SOFTWARE_VERSION  0			/*自升级版本*/
#define APP_TEST_VERSION	  "a.0"  	/*测试版本号*/
#endif
