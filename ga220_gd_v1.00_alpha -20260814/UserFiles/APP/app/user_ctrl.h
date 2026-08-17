#ifndef __USER_CTRL_H
#define __USER_CTRL_H

#include "main.h"
#include "math.h"
#include "config/config.h"
#define LaserPower_On 	HAL_GPIO_WritePin(Laser_IO_GPIO_Port, Laser_IO_Pin, GPIO_PIN_RESET);
#define LaserPower_Off 	HAL_GPIO_WritePin(Laser_IO_GPIO_Port, Laser_IO_Pin, GPIO_PIN_SET);
#define IRPower_On  	HAL_GPIO_WritePin(IR_IO_GPIO_Port, IR_IO_Pin, GPIO_PIN_RESET);
#define IRPower_Off		HAL_GPIO_WritePin(IR_IO_GPIO_Port, IR_IO_Pin, GPIO_PIN_SET);
#define FAN_Off  	HAL_GPIO_WritePin(FAN_IO_GPIO_Port, FAN_IO_Pin, GPIO_PIN_RESET);
#define FAN_On		HAL_GPIO_WritePin(FAN_IO_GPIO_Port, FAN_IO_Pin, GPIO_PIN_SET);

//#define led_run		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

extern TIM_HandleTypeDef htim6;

/***********************************函数定义********************************************/
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
    uint32_t Z3OSD;

    uint32_t ViewMatch;

    uint32_t iwdg_tick;
    uint32_t IBITStart;
} User_TickTypeDef;

/*初始化状态机*/
typedef enum
{
    PERIGHERALS_INIT_WAIT,
    PERIGHERALS_INIT_STEP1,
    PERIGHERALS_INIT_STEP2,
    PERIGHERALS_INIT_STEP3,
    PERIGHERALS_INIT_STEP4,
    PERIGHERALS_INIT_END,
} PERIGHERALS_INIT_STEP_ENUM;
/**************常用功能函数***************/
void IWDG_Refresh_Period_Handle(void);
void TIM6_ISR(void);
void USER_Ctrl_GEO_process(void);
void USER_Ctrl_Hw_Power_init(void);
void USER_Ctrl_System_Handle(void);
float USER_Ctrl_View_Match(float curr_view);
/**************常用功能函数***************/

/**************通信控制函数***************/
void LaserCtrl_Loop_Handle(void);
void USER_Ctrl_Track_Handle(void);
void USER_Ctrl_VL_Handle(void);
void USER_Ctrl_Laser_Handle(void);
void USER_Ctrl_IR_Handle(void);
void USER_Ctrl_SF_Handle(void);
void USER_Ctrl_Inner_Handle(void);
/**************通信控制函数***************/
#endif
