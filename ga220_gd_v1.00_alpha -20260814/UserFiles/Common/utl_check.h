/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __UTL_CHECK_H
#define __UTL_CHECK_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include "Common/base_inc.h"
/* USER CODE BEGIN Includes */
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:和校验
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t UTL_ADD_CHECK(uint8_t *adddata,uint16_t length);
/*==============================================================
*FUNCTION NAME:校验和计算
*DISCRIPTION:校验和计算(求和取反版)
*PARAMETERS:IN-需要校验的数据;OUT-ADD_CHECK
*RETURN:ADD_CHECK
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t UTL_ADD_CHECK_REV(uint8_t *data,uint16_t length);
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:异或校验
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t UTL_XOR_CHECK(uint8_t *data,uint16_t len);
/*==============================================================
*FUNCTION NAME:CRC校验
*DISCRIPTION:CRC校验
*PARAMETERS:IN-需要校验的数据;OUT-CRC_CHECK
*RETURN:CRC_CHECK
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t UTL_CRC_CHECK(uint8_t *data,uint16_t len);
/*==============================================================
 * Function Name:UTL_MATH_Get_CRC8
 * Description:
 *	get crc8
 * Parameters:
 *	val       :
 * Return:crc8
 * Note:
 *	N/A
 * History:
*==============================================================*/
uint8_t UTL_Get_CRC8(uint8_t *ptr, uint16_t len);
/*==============================================================
 * Function Name:CRC16_CCITT_FALSE
 * Description: get crc8
 * Parameters:
 *	val       :
 * Return:crc8
 * Note:
 *	N/A
 * History:
*==============================================================*/
uint16_t UTL_CRC16_CCITT_FALSE(uint8_t* pMsg, uint32_t size);
/*==============================================================
 * Function Name:	CRC16_CCITT
 * Description:get crc16
 * Parameters:
 *	val:
 * Return: crc8
 * Note:
 *	N/A
 * History:
*==============================================================*/
uint16_t UTL_CRC16_CCITT(uint8_t* pMsg, uint32_t size);
/*===================================================================
 * FUNCTION NAME:UTL_FletCher16
 * DESCRIPTION:累加和取模
 *	N/A
 * PARAMETERS:
 *	I16Ulen   :
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *=================================================================*/
uint16_t UTL_FletCher16(const uint8_t *pbuf, uint16_t len);

#endif 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
