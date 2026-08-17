#include "Driver/drv_uart.h"

#if defined(STM32F4)
#include "stm32F4xx_hal.h"
#elif defined(STM32H7)
#include "stm32H7xx_hal.h"
#endif
#include "BootApp/BootApp.h"

static COM_RECV_INFO_T upgrade_app_recv;

uint16_t boot_upgrade_app_period_handle(void)
{
    uint8_t rev_len = 0;
    rev_len = COM_REC_Data_Direct(COM_BOOT, upgrade_app_recv.recv_buf);
    uint8_t* Data = upgrade_app_recv.recv_buf;

    if((Data[0] == 0x5A) && (Data[1] == 0xA5) && (Data[4] == 0xAA)
            && (Data[5] == 0x03) && (rev_len == 16))
    {
		Flash_Erase_Sector(FALSH_Transmit_ADDR);
		Flash_WriteNoErase(FALSH_Transmit_ADDR, Data, 16);
        HAL_NVIC_SystemReset();
    }

    return 0;
}
