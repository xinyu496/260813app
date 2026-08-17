
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
#ifndef __OPT_MODULE_H
#define __OPT_MODULE_H

/*=================组件模块宏控制==========================*/
#define ETH_INCLUDE     		0/*网络*/

#define FM503G_0A         		0
#define JV_MINS_6G        		1
#define GD_INSIDE_INCLUDE 		JV_MINS_6G   /*内惯导*/

#define ZH_PLANE_GD       		1
#define JV_MINS_3C        		0
#define FK_GD					0
#define GD_EXT_INCLUDE		ZH_PLANE_GD	/*外惯导*/

#define LASER_LSP_LD_0820		0	
#define LASER_LRD_0301			0
#define LASER_LRS_0310F			0
#define LASER_LRS_0610A			0
#define LASER_LSP_0410 			0
#define LASER_DYC_15A 			0
#define LASER_LRD_1200_G 		0
#define LASER_LR3000 			0
#define LASER_15H 				0
#define LASER_LRD_25B07			0 /*DYB-01YC1协议一致 波特率 460800*/
#define LASER_INCLUDE     	(LASER_LRS_0610A)/*激光*/

#define VL_INCLUDE_NONE     	0x0
#define VL_F15_300			  	0x1/*可见光*/
#define VL_VS2030			  	0x2/*VS65130协议一致*/
#define VL_VS65130				0x4
#define VL_LD					0x8/*凌动可见光协议一致*/
#define VL_CM8230				0x10
#define VISIBLE_INCLUDE			(VL_F15_300)//(VL_VS2030|VL_F15_300)

#define IR_INCLUDE_NONE			0x0 
#define IR_DYBMC_L640C500A		0x1
#define IR_LGCS123 				0x2/*lgcs123红外*/
#define IR_NX30_150				0x4
#define IR_S640A_C1				0x8
#define IR_HY5050				0x10
#define IR_LGCS6122 			0x20/*lgcs123红外*/
#define IR_TWIN612RG2			0x40
#define IR_CTRL_INCLUDE   		IR_INCLUDE_NONE	//(IR_NX30_150|IR_LGCS123)
 /*编码器*/
#define BMQ_UEF072		1
#define BMQ_ECODER32S	0
#define BMQ_INCLUDE		BMQ_UEF072

#define SF_INCLUDE			 	0  /*伺服*/

#define AA320_TRACK				0
#define UNITY_TRACK				1
#define VIDEO_TRACK_INCLUDE	 UNITY_TRACK/*视频跟踪器*/

/*上级系统*/
#define MASTER_INCLUDE 			1

/*与上位机的交互*/
#define SYS_IN_INCLUDE			0/*上位机输入*/
#define SYS_OUT_INCLUDE			0/*上位机输出--关联各个外设，因此关闭某部件的宏时，需要关闭该宏；或者检查该宏控的报文是否正确*/
#define MASTER_INCLUDE 			1

#define CLI_INCLUDE				0
#define ONLINE_UPGRADE_INCLUDE	0/*在线升级*/
#define FLASH_INCLUDE			0

/*=================通用功能模块宏控制==========================*/

#define GEOTRACK      1/*地理跟踪/随动*/
#define EOFOLLOW      1/*EO随动*/
#define GEOPOSTION    1/*目标定位*/

//配置功能 0 - 关		1-开
#define GEO_TRA				1			/*地理跟踪*/
#define TARGET_POS		1			/*目标定位*/
#define LMC_STA				1			/*LMC*/

//用于目标定位和地理跟踪的数据源
#define SOURCE_PLANEGD	1		//飞机惯导 或 安装于基座的内惯导
#if GD_INSIDE_INCLUDE
	#define SOURCE_INNERGD	2		//内惯导（比如安装于载荷框架内）
#endif

#if GEO_TRA	//配置地理跟踪数据来源
	#define GEO_TRA_SOUR 		SOURCE_PLANEGD //SOURCE_INNERGD
#else
	#define GEO_TRA_SOUR 		0
#endif

#if TARGET_POS //配置目标定位数据来源
	#define TARGET_POS_SOUR 		SOURCE_INNERGD //SOURCE_INNERGD
#else
	#define TARGET_POS_SOUR 	0
#endif

#endif

