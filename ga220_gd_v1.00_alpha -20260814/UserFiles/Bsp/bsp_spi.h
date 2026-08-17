#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include "Common/base_inc.h"

void BSP_SPI_Reg_Write(uint8_t reg_addr,uint8_t *reg_buf,uint8_t size);
void BSP_SPI_Reg_Read(uint8_t *reg_addr,uint8_t read_len,uint8_t *reg_buf,uint8_t size);

#endif

