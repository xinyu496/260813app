#ifndef __BSP_TIMER_H
#define __BSP_TIMER_H
#include <stdint.h>
#include "Common/base_inc.h"

#define  TIME1_ENABLE  0
#define  TIME2_ENABLE  0
#define  TIME3_ENABLE  0
#define  TIME4_ENABLE  0
#define  TIME5_ENABLE  0
#define  TIME6_ENABLE  1
#define  TIME7_ENABLE  0
#define  TIME8_ENABLE  0
#define  TIME9_ENABLE  0
#define  TIME10_ENABLE  0
#define  TIME11_ENABLE  0
#define  TIME12_ENABLE  0
#define  TIME13_ENABLE  0
#define  TIME14_ENABLE  0



/*用户计数*/
typedef struct
{
    uint32_t EthSend;
    uint32_t VLCtrl;
    uint32_t IRCtrl;
    uint32_t LaserCtrl;
    uint32_t TrackCtrl;
    uint32_t InnerGD;
    uint32_t SF;
    uint32_t FWSF;
    uint32_t FYSF;
    uint32_t CmdHandle;
    uint32_t GEO_tick;
    uint32_t Fiber;
    uint32_t ViewMatch;
    uint32_t iwdg_tick;
    uint32_t IBITStart;
} USER_CNT_T;


extern uint32_t tick_s;
extern uint32_t tick_ms;

extern USER_CNT_T User_Tick;

extern volatile uint32_t com_spi_start_cnt;
extern volatile uint32_t com_spi_skip_cnt;
extern volatile uint32_t com_spi_recover_cnt;
extern volatile uint32_t com_spi_start_fail_cnt;

void TIMER_Interupt_Init(void);
//PWM周期、占空比修改
void PWM_Update_HighTimer(uint8_t number , uint8_t channel , uint32_t freq , uint8_t dutyCycle);

int32_t ticks_timeout(uint32_t* ticks, uint32_t timeout);

int32_t tickms_timeout(uint32_t* tickms, uint32_t timeout);

void TIMER_Pwm_Init(uint8_t number , uint8_t channel);//开启PWM
void TIMER_Pwm_Stop(uint8_t number , uint8_t channel);//关闭PWM

uint32_t GET_TIM_FLAG_1ms(void);
void CLEAR_TIM_FLAG_1ms(void);
/** @brief 原子取走并清零待处理 1ms 节拍数（主循环补跑用，避免丢 tick） */
uint32_t TAKE_TIM_FLAG_1ms(void);

#endif
