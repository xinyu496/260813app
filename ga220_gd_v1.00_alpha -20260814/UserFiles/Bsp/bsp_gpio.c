/**
  ******************************************************************************
  * @file           : bsp_gpio.c
  * @brief          : This file provides code for the configuration
  *          		  of the USART instances.
  ******************************************************************************
  * @attention
  *
  * @Copyright 		(c) Sichuan Zhongke Youcheng Technology Co.,Ltd.
  * @Author			: wangbao
  * @Version		: 1.0
  * @Date			: 2025.10.09
  * @History:
  *     +------------+---------------------------------------------------------+
  *		| 2025.10.09 |	重定义所有IO输出输入宏，IO控制请调用bsp_gpio.h,允许重定义IO名
  *     +------------+---------------------------------------------------------+
  ******************************************************************************
  */
#include "bsp_gpio.h"
uint8_t exit_isr_flag = 0;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:外部中断回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == gpioPin4)
    {
		 exit_isr_flag = 1;
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IO翻转
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void SYS_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	do{ GPIOx->ODR^=GPIO_Pin; }while(0);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:读取IO状态
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t SYS_GPIO_PIN_Get(GPIO_TypeDef *p_gpiox, uint16_t pinx)
{
    if (p_gpiox->IDR & pinx)
    {
        return 1;   
    }
    else
    {
        return 0;   
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:设置IO状态
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void SYS_GPIO_PIN_Set(GPIO_TypeDef *p_gpiox, uint16_t pinx, uint8_t status)
{
    if (status & 0X01)
    {
        p_gpiox->BSRR |= pinx;  				//设置GPIOx的pinx为1
    }
    else
    {
        p_gpiox->BSRR |= (uint32_t)pinx << 16;  //设置GPIOx的pinx为0
    }
}













