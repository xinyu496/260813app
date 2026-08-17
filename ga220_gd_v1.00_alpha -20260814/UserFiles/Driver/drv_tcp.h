#ifndef __DRV_TCP_H
#define __DRV_TCP_H
#include "Common/base_inc.h"
#if ETH_INCLUDE
#include "lwip/ip_addr.h"
#include "lwip.h"

void TCP_SERVER_Init(void);
void TCP_SERVER_Send(uint8_t *data_ptr, uint16_t data_len);
#endif
#endif
