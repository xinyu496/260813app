#ifndef __BSP_ADC_H
#define __BSP_ADC_H
#include "Common/base_inc.h"


#define ADC1_ENABLE 0
#define ADC2_ENABLE 0
#define ADC3_ENABLE 0


#define ADC1_CHANEL_NUM		1			//ADC通道数
#define ADC1_COLLECT_NUM	10			//采集次数，用于取平均值


#define ADC2_CHANEL_NUM		2			//ADC通道数
#define ADC2_COLLECT_NUM	2			//采集次数，用于取平均值

#define ADC3_CHANEL_NUM		2			//ADC通道数
#define ADC3_COLLECT_NUM	2			//采集次数，用于取平均值



//滑动平均值滤波
float Moving_Average_updata(uint16_t num , uint16_t *buff);
//获取电压值
float Electronic_get(uint16_t value , float vref);
//adc初始化
void ADC_START_Init(void);
//adc重新启动
void ADC_restart(void);
#endif
