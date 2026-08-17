#ifndef __BSP_FLASH_H
#define __BSP_FLASH_H

#include <stdint.h>

#include "Common/base_inc.h"
#include "Bsp/bsp_status.h"
#include <stdbool.h>

#if defined(STM32F4)
/*************用户根据芯片手册定义flash扇区地址****************/
/*扇区0起始地址，16kbytes*/
#define FLASH_F4ADDR_SECTOR_0		((uint32_t)FLASH_BASE)
/*扇区1起始地址，16kbytes*/
#define FLASH_F4ADDR_SECTOR_1		((uint32_t)FLASH_BASE | 0x4000)
/*扇区2起始地址，16kbytes*/
#define FLASH_F4ADDR_SECTOR_2		((uint32_t)FLASH_BASE | 0x8000)
/*扇区3起始地址，16kbytes*/
#define FLASH_F4ADDR_SECTOR_3		((uint32_t)FLASH_BASE | 0xC000)
/*扇区4起始地址，64kbytes*/
#define FLASH_F4ADDR_SECTOR_4		((uint32_t)FLASH_BASE | 0x10000)
/*扇区5起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_5		((uint32_t)FLASH_BASE | 0x20000)
/*扇区6起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_6		((uint32_t)FLASH_BASE | 0x40000)
/*扇区7起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_7		((uint32_t)FLASH_BASE | 0x60000)
/*扇区8起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_8		((uint32_t)FLASH_BASE | 0x80000)
/*扇区9起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_9		((uint32_t)FLASH_BASE | 0xA0000)
/*扇区10起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_10		((uint32_t)FLASH_BASE | 0xC0000)
/*扇区11起始地址，128kbytes*/
#define FLASH_F4ADDR_SECTOR_11		((uint32_t)FLASH_BASE | 0xE0000)
/*************用户根据芯片手册定义flash扇区地址****************/
#elif defined(STM32H7)
/*扇区0起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_0		((uint32_t)0x08000000)
/*扇区16起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_16		((uint32_t)0x08020000)
/*扇区32起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_32		((uint32_t)0x08040000)
/*扇区48起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_48		((uint32_t)0x08060000)
/*扇区64起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_64		((uint32_t)0x08080000)
/*扇区80起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_80		((uint32_t)0x080A0000)
/*扇区96起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_96		((uint32_t)0x080C0000)
/*扇区112起始地址，bank1,128kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_112		((uint32_t)0x080E0000)
/*扇区127起始地址，bank1,8kbytes*/
#define FLASH_BANK1_H7ADDR_SECTOR_127		((uint32_t)0x080FE000)

#define FLASH_BANK2_H7ADDR_SECTOR_0			((uint32_t)0x08100000)
/*扇区16起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_16		((uint32_t)0x08120000)
/*扇区32起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_32		((uint32_t)0x08140000)
/*扇区48起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_48		((uint32_t)0x08160000)
/*扇区64起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_64		((uint32_t)0x08180000)
/*扇区80起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_80		((uint32_t)0x081A0000)
/*扇区96起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_96		((uint32_t)0x081C0000)
/*扇区112起始地址，Bank2,128kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_112		((uint32_t)0x081E0000)
/*扇区127起始地址，Bank2,8kbytes*/
#define FLASH_BANK2_H7ADDR_SECTOR_127		((uint32_t)0x081FE000)
#endif /*STM32F4*/

#if 0
/*stm32h743*/
/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */
 
/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */
#endif


status_t Flash_Erase_Sector(uint32_t address);
uint16_t Flash_WriteNoErase(uint32_t address, uint8_t *data, uint32_t size);
uint16_t Flash_Read(uint32_t address, uint8_t *data, uint32_t size);
bool STMFLASH_Write(uint32_t WriteAddr,uint32_t* pBuffer,uint32_t NumToWrite);

#endif /*__BSP_FLASH_H*/
