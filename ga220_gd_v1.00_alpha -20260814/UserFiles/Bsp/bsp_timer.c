/**
  ******************************************************************************
  * @file           : hw_timer.c
  * @brief          : This file provides code for the configuration
  *          		  of the timer instances.
  ******************************************************************************
  * @attention
  *
  * @Copyright 		(c) Sichuan Zhongke Youcheng Technology Co.,Ltd.
  * @Author			: wangbao
  * @Version		: 1.0
  * @Date			: 2025.10.09
  * @History:
  *     +------------+---------------------------------------------------------+
  *		| 2025.10.09 |	创建文件,完成基本功能
  *     +------------+---------------------------------------------------------+
  ******************************************************************************
  */
#include "Bsp/bsp_timer.h"
#include "Bsp/bsp_uart.h"
#include "spi.h"
#include "stm32f4xx.h"

#define PWM_INTERNAL_CLOCK (16800000)  //（timer挂载的APB频率）/（预分频系数+1）
#define SPI1_SAMPLE_BUSY_TIMEOUT_MS     (3U)
#define SPI1_DMA_ABORT_TIMEOUT_MS       (10U)

uint32_t tick_s = 0;
uint32_t tick_ms = 0;

#ifdef HAL_TIM_MODULE_ENABLED
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim3;
extern SPI_HandleTypeDef hspi1;
#else
TIM_HandleTypeDef htim6;
SPI_HandleTypeDef hspi1;
#endif

volatile uint32_t com_spi_start_cnt;
volatile uint32_t com_spi_skip_cnt;
volatile uint32_t com_spi_recover_cnt;
volatile uint32_t com_spi_start_fail_cnt;


/** @brief 应用层 1ms 任务待处理节拍（TIM6 递增，主循环 TAKE 取走） */
static volatile uint32_t s_tick_1ms;
USER_CNT_T User_Tick;
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:定时器中断计数
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t ticksss = 0;
void TIM6_ISR(void)
{
	ticksss++;
    User_Tick.EthSend++;    //以太网计数
    User_Tick.VLCtrl++;     //可见光计数
    User_Tick.IRCtrl++;     //
    User_Tick.LaserCtrl++;  //
    User_Tick.InnerGD++;    //
    User_Tick.CmdHandle++;  //
    User_Tick.SF++;         //伺服计数
    User_Tick.TrackCtrl++;  //
    User_Tick.GEO_tick++;   //陀螺计数
    s_tick_1ms++;
}

uint32_t GET_TIM_FLAG_1ms(void)
{
    return s_tick_1ms;
}

void CLEAR_TIM_FLAG_1ms(void)
{
    s_tick_1ms = 0U;
}

uint32_t TAKE_TIM_FLAG_1ms(void)
{
    uint32_t pending;

    __disable_irq();
    pending = s_tick_1ms;
    s_tick_1ms = 0U;
    __enable_irq();

    return pending;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:定时器终端
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void timer_IRQ_USERHandler(void)
{
    COM_Uart_TxTimeout_Handler();

    //串口接收超时判断
    //COM_API_Communicate_Judg_Timer();

    //串口中断计数
    TIM6_ISR();
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:定时器中断回调
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) {
        timer_IRQ_USERHandler();
    }
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:定时器中断初始化
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TIMER_Interupt_Init(void)
{
#if TIME1_ENABLE
    HAL_TIM_Base_Start_IT(&htim1);
#endif
#if TIME2_ENABLE
    HAL_TIM_Base_Start_IT(&htim2);
#endif
#if TIME3_ENABLE
//    HAL_TIM_Base_Start_IT(&htim3);
#endif
#if TIME4_ENABLE
    HAL_TIM_Base_Start_IT(&htim4);
#endif
#if TIME5_ENABLE
    HAL_TIM_Base_Start_IT(&htim5);
#endif
#if TIME6_ENABLE
    HAL_TIM_Base_Start_IT(&htim6);
#endif
#if TIME7_ENABLE
    HAL_TIM_Base_Start_IT(&htim7);
#endif
#if TIME8_ENABLE
    HAL_TIM_Base_Start_IT(&htim8);
#endif
#if TIME9_ENABLE
    HAL_TIM_Base_Start_IT(&htim9);
#endif
#if TIME10_ENABLE
    HAL_TIM_Base_Start_IT(&htim10);
#endif
#if TIME11_ENABLE
    HAL_TIM_Base_Start_IT(&htim11);
#endif
#if TIME12_ENABLE
    HAL_TIM_Base_Start_IT(&htim12);
#endif
#if TIME13_ENABLE
    HAL_TIM_Base_Start_IT(&htim13);
#endif
#if TIME14_ENABLE
    HAL_TIM_Base_Start_IT(&htim14);
#endif
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:秒计时
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
int32_t ticks_timeout(uint32_t *ticks, uint32_t timeout)
{
    if (tick_s - (*ticks) > timeout) {
        *ticks = tick_s;
        return 1;
    }
    return 0;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:毫秒计时
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
int32_t tickms_timeout(uint32_t *tickms, uint32_t timeout)
{
    if (tick_ms - (*tickms) > timeout) {
        *tickms = tick_ms;
        return 1;
    }
    return 0;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取定时器号
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static TIM_HandleTypeDef timer_number_get(uint8_t number)
{
    TIM_HandleTypeDef timer_num;

    switch (number) {
#if TIME1_ENABLE
        case 1: timer_num = htim1; break;
#endif
#if TIME2_ENABLE
        case 2: timer_num = htim2; break;
#endif
#if TIME3_ENABLE
        case 3: timer_num = htim3; break;
#endif
#if TIME4_ENABLE
        case 4: timer_num = htim4; break;
#endif
#if TIME5_ENABLE
        case 5: timer_num = htim5; break;
#endif
#if TIME6_ENABLE
        case 6: timer_num = htim6; break;
#endif
#if TIME7_ENABLE
        case 7: timer_num = htim7; break;
#endif
#if TIME8_ENABLE
        case 8: timer_num = htim8; break;
#endif
#if TIME9_ENABLE
        case 9: timer_num = htim9; break;
#endif
#if TIME10_ENABLE
        case 10: timer_num = htim10; break;
#endif
#if TIME11_ENABLE
        case 11: timer_num = htim11; break;
#endif
#if TIME12_ENABLE
        case 12: timer_num = htim12; break;
#endif
#if TIME13_ENABLE
        case 13: timer_num = htim13; break;
#endif
#if TIME14_ENABLE
        case 14: timer_num = htim14; break;
#endif
        default: break;
    }

    return timer_num;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取高级定时器输出通道号
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint32_t timer_channel_get(uint8_t number)
{
    uint32_t num;

    switch (number) {
        case 1:
            num = TIM_CHANNEL_1;  //PWM通道1
            break;
        case 2:
            num = TIM_CHANNEL_2;  //PWM通道2
            break;
        case 3:
            num = TIM_CHANNEL_3;  //PWM通道3
            break;
        case 4:
            num = TIM_CHANNEL_4;  //PWM通道4
            break;
        default: break;
    }

    return num;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:开启PWM
*PARAMETERS:number:定时器号；channel：输出通道；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TIMER_Pwm_Init(uint8_t number, uint8_t channel)
{
    TIM_HandleTypeDef timer_num;
    uint32_t output_channel;

    timer_num = timer_number_get(number);
    output_channel = timer_channel_get(channel);

    HAL_TIM_PWM_Start(&timer_num, output_channel);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:关闭PWM
*PARAMETERS:number:定时器号；channel：输出通道；
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TIMER_Pwm_Stop(uint8_t number, uint8_t channel)
{
    TIM_HandleTypeDef timer_num;
    uint32_t output_channel;

    timer_num = timer_number_get(number);
    output_channel = timer_channel_get(channel);

    HAL_TIM_PWM_Stop(&timer_num, output_channel);
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:修改高级定时器PWM周期和占空比
*PARAMETERS:number:定时器号；channel：输出通道；freq：输出频率；dutyCycle：占空比
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void TIMER_Pwm_Update_HighTimer(uint8_t number, uint8_t channel, uint32_t freq, uint8_t dutyCycle)
{
    TIM_HandleTypeDef timer_num;
    uint32_t output_channel;

    timer_num = timer_number_get(number);
    output_channel = timer_channel_get(channel);

    uint32_t period = (PWM_INTERNAL_CLOCK / freq) - 1;
    uint16_t pulse = (period + 1) * dutyCycle / 100;

    __HAL_TIM_SET_AUTORELOAD(&timer_num, period);             //更新周期
    __HAL_TIM_SetCompare(&timer_num, output_channel, pulse);  //更新占空比
}
