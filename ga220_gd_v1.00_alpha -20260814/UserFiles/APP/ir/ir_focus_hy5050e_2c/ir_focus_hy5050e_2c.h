#ifndef __IR_FOCUS_HY5050e_2C_H
#define	__IR_FOCUS_HY5050e_2C_H
#include <stdint.h>
#include "Common/config.h"
__packed typedef struct
{
    uint8_t sync;
    uint8_t addr;
    uint8_t	cmd1;
    uint8_t	cmd2;
    uint8_t data1;
    uint8_t data2;
    uint8_t checksum;
} IR_HY5050E_DATA_T;

uint8_t Ir_Ctrl_hy5050e_Cmd_Data_send(uint8_t cmd, uint16_t param);

RECV_DATA_ERR_STA IR_API_Hy5050e_Period_Handle(void);
#endif
