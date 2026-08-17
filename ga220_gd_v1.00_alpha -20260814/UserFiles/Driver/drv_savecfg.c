#include "Driver/drv_savecfg.h"
#include "Common/utl_check.h"
#include <string.h>

/*==============================================================
*FUNC:	write_config_crc
*DESC:	加校验之后，将配置写入flash中
*PARAM:	(in)	pconfig		带写入参数的指针
*RETURN:	状态码
	STATUS_SUCCESS 	: 成功
 	STATUS_ERROR	: 失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t write_config_crc(uint8_t* pconfig, uint16_t size)
{
    uint16_t* pcrc = (uint16_t*)(pconfig + (size - 2));   // 让指针指向 crc
    *(pcrc) = UTL_CRC16_CCITT(pconfig, (size - 2));  // 计算crc
    Flash_Erase_Sector(SAVECFG_BEGIN_ADDR);
    uint16_t ret = Flash_WriteNoErase(SAVECFG_BEGIN_ADDR, pconfig, size);
    return ret;
}
/*==============================================================
*FUNC:	flash_load_syserr
*DESC:	从flash中读取配置
*PARAM:	(out)	pconfig		传出参数，读取配置的指针
*RETURN:	状态码
	STATUS_SUCCESS 	: 成功
 	STATUS_ERROR	: 失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint16_t read_config_crc(uint8_t* pconfig, uint16_t size)
{
    Flash_Read(SAVECFG_BEGIN_ADDR, pconfig, size);
//    // 计算读出的数据校验
    uint16_t crc = UTL_CRC16_CCITT(pconfig, (size - 2));
    uint16_t* pcrc = (uint16_t*)(pconfig + ( - 2));   // 让指针指向 crc

    if(crc != *pcrc)
    {
        memset(pconfig, 0, size); // 值不对清除读出值
        return STATUS_ERROR;
    }

    return STATUS_SUCCESS;
}



