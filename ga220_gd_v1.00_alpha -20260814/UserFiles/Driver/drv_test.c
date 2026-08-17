#include "drv_test.h"






/****************dac输出*****************/
void dac_ouput(void)
{
	static uint16_t dac_val = 2048;
	
	if(tick_s > 0)
	{
		tick_s = 0;
		if(dac_state == 0)
		{
			dac_val += 500;
			if(dac_state > 3500)
			{
				dac_state = 1;
			}
		}
		else
		{
			dac_val -= 500;
			if(dac_state < 1000)
			{
				dac_state = 0;
			}
		}
		
//		HAL_DAC_Stop(&hdac , DAC_CHANNEL_1);
//		HAL_DAC_SetValue(&hdac , DAC_CHANNEL_1 , DAC_ALIGN_12B_R , dac_val);
//		HAL_DAC_Start(&hdac , DAC_CHANNEL_1);
	}
}










