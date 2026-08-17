#include "Bsp/bsp_adc.h"



//ADC1采集
__IO uint16_t ADC1_CovertedValue[ADC1_CHANEL_NUM * ADC1_COLLECT_NUM] = {0}; //采集数据

//ADC2采集
uint16_t ADC2_CovertedValue[ADC2_CHANEL_NUM * ADC2_COLLECT_NUM] = {0}; //采集数据

//ADC3采集
uint16_t ADC3_CovertedValue[ADC3_CHANEL_NUM * ADC3_COLLECT_NUM] = {0}; //采集数据

void ADC_START_Init(void)
{
	HAL_ADC_Start_DMA(&hadc1 , (uint32_t*)ADC1_CovertedValue , ADC1_CHANEL_NUM * ADC1_COLLECT_NUM);
}


void ADC_restart(void)
{
	HAL_ADC_Stop_DMA(&hadc1);
	ADC_START_Init();
}

/*==============================================================
*FUNCTION NAME:滑动平均值滤波
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES: 
*HISTORY:
*==============================================================*/
float Moving_Average_updata(uint16_t num, uint16_t *buff)
{
    uint16_t i, m;
    float data;
    m = num;

    for(i = 0 ; i < m ; i++ )
    {
        data += (float)buff[i] / 100;
    }

    data = data / m;
    data = data * 100;
    return data;
}


/*==============================================================
*FUNCTION NAME:获取电压值
*DISCRIPTION:
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
float Electronic_get(uint16_t value, float vref)
{
    float data = value / 4095 * vref;
    return data;
}
















