/**
  ******************************************************************************
  * @file           : hw_flash.c
  * @brief          : This file provides code for the configuration
  *          		  of the flash instances.
  ******************************************************************************
  * @attention
  *
  * @Copyright 		(c) Sichuan Zhongke Youcheng Technology Co.,Ltd.
  * @Author			: wangbao
  * @Version		: 1.0
  * @Date			: 2025.10.30
  * @History:
  *     +------------+---------------------------------------------------------+
  *		| 2025.10.30 |	创建文件,完成基本功能
  *     +------------+---------------------------------------------------------+
  ******************************************************************************
  */
#include "Bsp/bsp_flash.h"
#define STM32FLASH_WAITTIME	     50000

/*==============================================================
*FUNC:	STMFLASH_ReadWord
*DESC:	读指定地址的字（4字节数据）
*PARAM:	(in)	addr	读取的地址
*RETURN:	数据
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t STMFLASH_ReadWord(uint32_t addr)
{
    return *(uint32_t *)addr;
}

/*==============================================================
*FUNC:	STMFLASH_ReadNWord
*DESC:	读指定地址的数据（n字数据）
*PARAM:	(in)	addr	读取的地址
	(out)	pBuffer		存放读出数据地址
	(in)	NumToRead	数据字个数
*RETURN:	数据
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void STMFLASH_ReadNWord(uint32_t ReadAddr, uint32_t *pBuffer,
                        uint32_t NumToRead)
{
    uint32_t i;

    for(i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadWord(ReadAddr);
        ReadAddr += 4;
#if defined(STM32F4)
        FLASH_WaitForLastOperation(STM32FLASH_WAITTIME);
#elif defined(STM32H7)
#endif
    }
}

/*==============================================================
*FUNC:	Flash_Read
*DESC:	从Flash读取数据（n字节数据）
*PARAM:	(out)	pBuffer		存储读取数据的缓冲区
		(in)	addr		起始地址
		(in)	size		要读取的字节数
*RETURN:	状态码
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t Flash_Read(uint32_t address, uint8_t *data, uint32_t size)
{
    // 简单检查地址范围（可根据实际情况调整）
#if defined(STM32F4)
    if(address < FLASH_F4ADDR_SECTOR_0 || address >= FLASH_F4ADDR_SECTOR_11)
    {
        return STATUS_FLASH_ERR_INV_ADDR;
    }

#elif defined(STM32H7)

    if(address < FLASH_BANK1_H7ADDR_SECTOR_0 || address >= FLASH_END)
    {
        return STATUS_FLASH_ERR_INV_ADDR;
    }

#endif

    // Flash映射到内存空间，可以直接读取
    for(uint32_t i = 0; i < size; i++)
    {
        data[i] = *(uint8_t*)(address + i);
    }

    return STATUS_FLASH_OK;
}

#if defined(STM32F4)
/*==============================================================
*FUNC:	STMFLASH_GetFlashSector
*DESC:	根据要写入的地址判断擦除的扇区
*PARAM:	(in)	addr		地址
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if(addr < FLASH_F4ADDR_SECTOR_1)
    {
        return FLASH_SECTOR_0;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_2)
    {
        return FLASH_SECTOR_1;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_3)
    {
        return FLASH_SECTOR_2;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_4)
    {
        return FLASH_SECTOR_3;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_5)
    {
        return FLASH_SECTOR_4;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_6)
    {
        return FLASH_SECTOR_5;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_7)
    {
        return FLASH_SECTOR_6;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_8)
    {
        return FLASH_SECTOR_7;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_9)
    {
        return FLASH_SECTOR_8;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_10)
    {
        return FLASH_SECTOR_9;
    }
    else if(addr < FLASH_F4ADDR_SECTOR_11)
    {
        return FLASH_SECTOR_10;
    }

    return FLASH_SECTOR_11;
}
#elif defined(STM32H7)

uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
#if defined (STM32H743xx)
    if(addr >= 0x08000000 && addr < 0x08100000)
        return (addr - 0x08000000)/0x20000;
#elif defined (STM32H7A3xx)
	if(addr >= FLASH_BANK1_BASE && addr < FLASH_BANK2_BASE)
		return (addr - FLASH_BANK1_BASE)/0x2000;
	else if(addr >= FLASH_BANK2_BASE && addr < FLASH_END)
		return (addr - FLASH_BANK2_BASE)/0x2000;
#endif
    return 0xff;
}

uint8_t STMFLASH_GetFlashBank(uint32_t addr)
{
    if(addr >= FLASH_BANK1_BASE && addr < FLASH_BANK2_BASE)
    {
        return FLASH_BANK_1;
    }
    else if(addr >= FLASH_BANK2_BASE && addr < FLASH_END)
    {
        return FLASH_BANK_2;
    }

    return 0;
}
#endif /*STM32F4*/
/*==============================================================
*FUNC:	Flash_Erase_Sector
*DESC:	擦除指定扇区
*PARAM:	(in)	sector  扇区编号(0-11)
*RETURN:	状态码
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
status_t Flash_Erase_Sector(uint32_t address)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t sectorError = 0;

    if(address >= 0x1fff0000)//避免踩踏到系统存储器
    {
        return STATUS_FLASH_ERR_INV_ADDR;
    }

    uint8_t sector = STMFLASH_GetFlashSector(address);
    // 检查扇区号有效性
#if defined(STM32F4)
    if(sector > FLASH_SECTOR_11)
#elif defined(STM32H7)
#if defined (STM32H743xx)
	if(sector > FLASH_SECTOR_7)
#elif defined (STM32H7A3xx)
    if(sector > FLASH_SECTOR_127)
#endif
#endif
    {
        return STATUS_FLASH_ERR_INV_ADDR;
    }

    // 1. 解锁Flash
    if(HAL_FLASH_Unlock() != HAL_OK)
    {
        return STATUS_FLASH_ERR_UNLOCK;
    }

    // 2. 清除所有错误标志
#if defined(STM32F4)
    // 清除错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32H7)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
#endif
    // 3. 配置擦除参数
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
#if defined(STM32H7)
    eraseInit.Banks = STMFLASH_GetFlashBank(address);//选择bank
#endif
    eraseInit.Sector = sector;
    eraseInit.NbSectors = 1;
	eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
#if defined(STM32H7)	
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7V - 3.6V
#endif
    // 4. 执行擦除（阻塞模式，带超时）
    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    // 5. 上锁Flash
    HAL_FLASH_Lock();
    return (status == HAL_OK) ? STATUS_FLASH_OK : STATUS_FLASH_ERR_ERASE;
}
/*==============================================================
*FUNC:	Flash_Write
*DESC:	写入数据到Flash
*PARAM:	(in)	sector  扇区编号(0-11)
        (in)    address 起始地址(必须是合法地址)
        (in)    size    数据大小(字节)
*RETURN:	状态码
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t Flash_WriteNoErase(uint32_t address, uint8_t *data, uint32_t size)
{
    uint16_t status = STATUS_FLASH_OK;
    uint32_t i;
    uint32_t new_data;
    uint32_t alignedSize;
	uint8_t new_data_t = 0;
    // 检查地址对齐（F405支持字节、半字、字写入）
    // 为简单起见，我们使用字(32位)写入，所以地址需要4字节对齐
    if(address & 0x3)
    {
        // 如果地址不对齐，调整到对齐地址
        address = address & ~0x3;
    }

    // 解锁Flash
    if(HAL_FLASH_Unlock() != HAL_OK)
    {
        return STATUS_FLASH_ERR_UNLOCK;
    }

#if defined(STM32F4)
    // 清除错误标志
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
                           FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
#elif defined(STM32H7)
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP |
                           FLASH_FLAG_WRPERR | FLASH_FLAG_PGSERR);
#endif
    // 按字(32位)写入
    alignedSize = (size + 3) & ~0x3;  // 向上对齐到4字节

    for(i = 0; i < alignedSize; i += 4)
    {
        // 构造要写入的32位数据
#if defined(STM32H7)
		if(i + 3 < size)
        {
            // 有4个有效字节
            new_data[new_data_t] = (data[i + 3] << 24) | (data[i + 2] << 16) |
                       (data[i + 1] << 8) | data[i];
        }
        else if(i < size)
        {
            // 不足4字节，剩余部分填充0xFF(Flash擦除状态)
            new_data[new_data_t] = 0xFFFFFFFF;
            uint8_t *pTemp = (uint8_t *)new_data;

            for(uint32_t j = i; j < size && j < i + 4; j++)
            {
                pTemp[j - i] = data[j];
            }
        }
        else
        {
            // 超出size范围，写入0xFF
            new_data[new_data_t] = 0xFFFFFFFF;
        }
#elif defined(STM32F4)
        if(i + 3 < size)
        {
            // 有4个有效字节
            new_data = (data[i + 3] << 24) | (data[i + 2] << 16) |
                       (data[i + 1] << 8) | data[i];

        }
        else if(i < size)
        {
            // 不足4字节，剩余部分填充0xFF(Flash擦除状态)
            new_data = 0xFFFFFFFF;
            uint8_t *pTemp = (uint8_t *)&new_data;

            for(uint32_t j = i; j < size && j < i + 4; j++)
            {
                pTemp[j - i] = data[j];
            }
        }
        else
        {
            // 超出size范围，写入0xFF
            new_data = 0xFFFFFFFF;
        }
#endif
        // 检查目标地址是否为0xFF（可写状态）
        // 只能将1变为0，不能将0变为1
        uint32_t current = *(uint32_t*)(address + i);

        // 确保不会将0变为1
#if defined(STM32H7)
        if((current & new_data[new_data_t]) != new_data[new_data_t])
        {
            // 需要先擦除
            status = STATUS_FLASH_ERR_WRITE;
            break;
        }
#elif defined(STM32F4)
        if((current & new_data) != new_data)
        {
            // 需要先擦除
            status = STATUS_FLASH_ERR_WRITE;
            break;
        }
#endif

        // 执行编程操作
#if defined(STM32F4)
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   address + i,
                                   new_data);
#elif defined(STM32H7)
	if(new_data_t == 3)
		{
			new_data_t = 0;
			status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                   address,
                                   (uint32_t)new_data);
		}
		new_data_t++;
#endif

        if(status != HAL_OK)
        {
			break;
        }

        // 可选：验证写入的数据（立即读取验证）
//        if(*(__IO uint32_t * )(address + i) != new_data)
//        {
//            status = STATUS_FLASH_ERR_WRITE;
//            break;
//        }
    }

    // 上锁Flash
    HAL_FLASH_Lock();
    return status;
}
