#ifndef __BOOTAPP_H
#define __BOOTAPP_H
#include "Bsp/bsp_flash.h"

#if defined(STM32F4)
#define FALSH_Transmit_ADDR		 FLASH_F4ADDR_SECTOR_10
#elif defined(STM32H7)
#define FALSH_Transmit_ADDR		 ((uint32_t)0x080C0000)
#endif

uint16_t boot_upgrade_app_period_handle(void);

#endif

