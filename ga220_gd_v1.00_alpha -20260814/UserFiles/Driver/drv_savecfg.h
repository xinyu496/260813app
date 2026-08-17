#ifndef __DRI_SAVECFG_H
#define __DRI_SAVECFG_H
#include "Bsp/bsp_flash.h"

/*保存信息的起始地址*/
#if defined(STM32F4)
    #define     SAVECFG_BEGIN_ADDR      FLASH_F4ADDR_SECTOR_11
#elif defined(STM32H7)
    #define     SAVECFG_BEGIN_ADDR      FLASH_BANK1_H7ADDR_SECTOR_127
#endif
/*可存储字节数*/  /* 128k */
#define     SAVECFG_AVAILABLE_BYTE  (0x20000)
/* 保存数据间隔字节数 */
#define     SAVE_INTERCAL_BYTE      (4)


uint16_t write_config_crc(uint8_t* pconfig, uint16_t size);
uint16_t read_config_crc(uint8_t* pconfig, uint16_t size);

#endif /*__DRI_SAVECFG_H*/
