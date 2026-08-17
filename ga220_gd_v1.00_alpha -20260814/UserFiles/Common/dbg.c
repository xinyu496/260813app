/*===================================================================
 *
 * COPYRIGHT:
 *
 * FILE NAME:
 *	os_debug.c
 * DESCRIPTION:
 *	N/A
 * HISTORY:
 *	V1.00 2013.1.22 By LmnyL, Create/Update
 *
 *=================================================================*/
#include "Common/dbg.h"
DBG_INTR_CNT_T dbg_intr_cnt;

DBG_PRT_T dbg_state[MODULE_TYPE_END];
static const char *const module_name[]=
{
	DEBUG_MODULE_NAME
};
#define DBG_BUF_MAGIC			        0x44424742
#define MAX_DEBUG_STACK1_LEN	        64
#define MAX_DEBUG_STACK2_LEN	        64
#define MAX_DEBUG_BUF_LEN		        (MAX_DEBUG_STACK1_LEN + MAX_DEBUG_STACK2_LEN + 5 + 1)
/*===================================================================
 * FUNCTION NAME:
 *	DBG_Get_ModuleName
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	module:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
uint8_t *DBG_Get_ModuleName(MODULE_TYPE_T module)
{
	return (uint8_t *)module_name[module];
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Set_PrintState
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	dbg_print_date:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
bool DBG_Set_PrintState(DBG_PRT_T *dbg_print_date)
{
	bool module_exist = false;
	uint32_t module;

	/* Check ALL */
	if (!strcasecmp((char *)dbg_print_date->module_name, "ALL"))
	{
		/* Prt ON/off ALL modules */
		for (module = 0; module < MODULE_TYPE_END; module++)
		{
//			if (module == MODULE_TYPE_SYS)
//			{
//				if(CLI_LOGIN_Get_LoginLevel() == CLI_CMD_LEVEL_ROOT)
//				{
//					dbg_state[(MODULE_TYPE_T)module].state = true;
//				}
//				else
//				{
//					dbg_state[(MODULE_TYPE_T)module].state = false;
//				}
//			}
//			else
//			{
//				dbg_state[(MODULE_TYPE_T)module].state = dbg_print_date->state;
//			}
		}

		return true;
	}

	if (!(dbg_print_date->module < MODULE_TYPE_END))
	{
		for (module = 0; module < MODULE_TYPE_END; module++)
		{
			if (strcasecmp((char *)dbg_print_date->module_name, module_name[(MODULE_TYPE_T)module]))
			{
				continue;
			}
			module_exist = true;
			break;
		}
		if (!module_exist)
		{
			return false;
		}
	}
	else
	{
		module = dbg_print_date->module;
	}

	if (module > MODULE_TYPE_END)
	{
		return false;
	}

	dbg_state[module].state = dbg_print_date->state;

	return true;
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Set_PrintStateByModuleType
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	dbg_print_date:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
bool DBG_Set_PrintStateByModuleType(MODULE_TYPE_T module_type, bool state)
{
	dbg_state[module_type].state = state;

	return true;
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Get_PrintState
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	module:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
bool DBG_Get_PrintState(MODULE_TYPE_T module)
{
	return dbg_state[module].state;
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Init
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	N/A
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
bool DBG_Init(void)
{
	memset(&dbg_state, 0x0, sizeof(dbg_state));
	memset(&dbg_intr_cnt, 0x0, sizeof(dbg_intr_cnt));

//	PERIP_EEPROM_Init();

	dbg_state[MODULE_TYPE_TX].state = false;
	dbg_state[MODULE_TYPE_TXTS].state = false;
	dbg_state[MODULE_TYPE_RX].state = false;
	dbg_state[MODULE_TYPE_RXTS].state = false;
	dbg_state[MODULE_TYPE_PHY].state = false;
	dbg_state[MODULE_TYPE_TIME].state = false;
	dbg_state[MODULE_TYPE_PTP_TIMER].state = false;
	dbg_state[MODULE_TYPE_PTP_MSG].state = false;
	dbg_state[MODULE_TYPE_PTP_SERVO].state = false;
	dbg_state[MODULE_TYPE_PTP_TS].state = false;
	dbg_state[MODULE_TYPE_FLT].state = false;
	dbg_state[MODULE_TYPE_NTP].state = false;
	dbg_state[MODULE_TYPE_REC].state = false;
	dbg_state[MODULE_TYPE_PE_DBG].state = false;
	dbg_state[MODULE_TYPE_PE].state = false;
	dbg_state[MODULE_TYPE_BDC_RCV].state = false;
	dbg_state[MODULE_TYPE_TIME].state = false;
	dbg_state[MODULE_TYPE_ENV].state = false;
	dbg_state[MODULE_TYPE_PTP_TADJ].state = false;

	dbg_state[MODULE_TYPE_EXT_RS_OFM].state = false;

	dbg_state[MODULE_TYPE_SYS].state = false;
	dbg_state[MODULE_TYPE_TOD].state = false;
	dbg_state[MODULE_TYPE_SYNC].state = false;
	dbg_state[MODULE_TYPE_TAME].state = false;
	dbg_state[MODULE_TYPE_TAME_RB].state = false;
	dbg_state[MODULE_TYPE_TDC].state = false;
	dbg_state[MODULE_TYPE_TDC_1].state = false;
	dbg_state[MODULE_TYPE_TDC_2].state = false;

	dbg_state[MODULE_TYPE_INFO].state = false;

	dbg_state[MODULE_TYPE_GPS].state = false;
#if RELEASE_INCLUDE
	dbg_state[MODULE_TYPE_REC].state = false;
	dbg_state[MODULE_TYPE_HP].state = false;
#else
	dbg_state[MODULE_TYPE_REC].state = true;
	dbg_state[MODULE_TYPE_HP].state = true;
#endif
	dbg_state[MODULE_TYPE_GPS_DATA].state = false;

	DBG_Set_PrintStateByModuleType(MODULE_TYPE_SYS, false);

	return true;
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Clear_HardFaultInfo
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	N/A
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.19 by LmnyL, Create
 *=================================================================*/
void DBG_Clear_HardFaultInfo(void)
{
//	uint32_t i;
//	uint32_t  data[MAX_DEBUG_BUF_LEN];
//	memset(data, 0xff, sizeof(data));

//	for (i = EEPROM_DBG_PAGE_START; i < EEPROM_DBG_PAGE_END; i++)
//	{
//		DBG_INFO_Print("\r\n Clear Page: %d...", i);

//		PERIP_EEPROM_Write_ByOffset(EEPROM_PAGE_TO_OFFSET(i),(uint8_t *)data, sizeof(data));

//		DBG_INFO_Print("Done");
//	}
}

/*===================================================================
 * FUNCTION NAME:
 *	dbg_save_hardfaultinfo
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	info:
 *	len:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.18 by LmnyL, Create
 *=================================================================*/
static void dbg_save_hardfaultinfo(uint32_t *info, uint32_t len)
{
//	uint32_t flash_write_base_addr;

//	/* Clear Old Hard Fault Info */
//	DBG_Clear_HardFaultInfo();

//	flash_write_base_addr = EEPROM_DBG_PAGE_START;

//	PERIP_EEPROM_Write_ByOffset(EEPROM_PAGE_TO_OFFSET(flash_write_base_addr),(uint8_t *)info, 4*len);
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Dump_HardInfo
 * DESCRIPTION:
 *	And then you can extract the stacked registers in C:
 *	hard fault handler in C,
 *	with stack frame location as input parameter
 * PARAMETERS:
 *	hardfault_args:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.5 by LmnyL, Create
 *=================================================================*/
void DBG_Dump_HardInfo(uint32_t *dbg_info, uint32_t debug_buf_len)
{
	int i;

	if (*(dbg_info + MAX_DEBUG_BUF_LEN - 1) == DBG_BUF_MAGIC)
	{
		printf ("\r\n\r\n[Hard Fault Handler]:");
		printf ("\r\n BFAR = 0x%08x", *(dbg_info + 0));
		printf ("\r\n CFSR = 0x%08x", *(dbg_info + 1));
		printf ("\r\n HFSR = 0x%08x", *(dbg_info + 2));
		printf ("\r\n DFSR = 0x%08x", *(dbg_info + 3));
		printf ("\r\n AFSR = 0x%08x\r\n", *(dbg_info + 4));

		for (i = 0; i < MAX_DEBUG_STACK1_LEN; i++)
		{
			if(((i) % 4) == 0)
			{
				printf("\r\n");
			}

			printf(" %08x", *(dbg_info + i + 5));
		}
		printf("\r\n ------Stack-----");
		for (i = 0; i < MAX_DEBUG_STACK2_LEN; i++)
		{
			if(((i) % 4) == 0)
			{
				printf("\r\n");
			}

			printf(" %08x", *(dbg_info + i + MAX_DEBUG_STACK1_LEN + 5));
		}
	}
}

/*===================================================================
 * FUNCTION NAME:
 *	dbg_handle_hardfault_c
 * DESCRIPTION:
 *	And then you can extract the stacked registers in C:
 *	hard fault handler in C,
 *	with stack frame location as input parameter
 * PARAMETERS:
 *	hardfault_args:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.5 by LmnyL, Create
 *=================================================================*/
void dbg_handle_hardfault_c(uint32_t * hardfault_args)
{
static uint32_t stacked_r0;
static uint32_t stacked_r1;
static uint32_t stacked_r2;
static uint32_t stacked_r3;
static uint32_t stacked_r12;
static uint32_t stacked_lr;
static uint32_t stacked_pc;
static uint32_t stacked_psr;
static uint32_t *stack_addr;
static uint32_t debug_buf[MAX_DEBUG_BUF_LEN];
static uint32_t debug_buf_len = 0;
static uint32_t cpu_sp = 0;

	stacked_r0 = (hardfault_args[0]);
	stacked_r1 = (hardfault_args[1]);
	stacked_r2 = (hardfault_args[2]);
	stacked_r3 = (hardfault_args[3]);
	stacked_r12 = (hardfault_args[4]);
	stacked_lr = (hardfault_args[5]);
	stacked_pc = (hardfault_args[6]);
	stacked_psr = (hardfault_args[7]);

	stack_addr = (uint32_t *)hardfault_args;

	cpu_sp = __get_MSP();

	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED38)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED28)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED2C)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED30)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED3C)));

	for (stacked_r1 = 0; stacked_r1 < MAX_DEBUG_STACK1_LEN; stacked_r1++)
	{
		debug_buf[debug_buf_len++] = *stack_addr;

		stack_addr++;
	}

	/* 获取Stack */
	stack_addr = (uint32_t *)(cpu_sp);
	for (stacked_r1 = 0; stacked_r1 < MAX_DEBUG_STACK2_LEN; stacked_r1++)
	{
		debug_buf[debug_buf_len++] = *stack_addr;

		stack_addr++;
	}
	debug_buf[debug_buf_len++] = DBG_BUF_MAGIC;

	/* Save To flash */
//	dbg_save_hardfaultinfo(debug_buf, debug_buf_len);

	DBG_Dump_HardInfo(debug_buf, debug_buf_len);

	/* Reboot */
//	NVIC_SystemReset();

	while(1){};
}

/*===================================================================
 * FUNCTION NAME:
 *	HardFault_Handler
 * DESCRIPTION:
 *	hard fault handler wrapper in assembly,
 *	it extract the location of stack frame and pass it,
 *  to handler in C as pointer.
 * PARAMETERS:
 *	N/A
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.5 by LmnyL, Create
 *=================================================================*/
//register unsigned int current_lr __ASM("lr");
//register unsigned int current_msp __ASM("msp");
//register unsigned int current_psp __ASM("psp");

//内联式汇编
//__attribute__((naked)) void HardFault_Handler(void)  //__ASM 
//{
//	__asm volatile(
//		//"IMPORT	dbg_handle_hardfault_c\n"
//		"TST LR, #4\n"
//		"ITE EQ\n"
//		"MRSEQ R0,MSP\n"
//		"MRSNE R0,PSP\n"
//		"LDR R1, =dbg_handle_hardfault_c\n"
//		"BX R1\n");
//		//"B __cpp(dbg_handle_hardfault_c)\n");
//	//IMPORT	dbg_handle_hardfault_c
////	__asm("MOVS R0,#4");
////	__asm("MOV R1,LR");
////	__asm("TST R0,R1");
////	__asm("BEQ _MSP");
////	__asm("MRS R0,PSP");
////	__asm("B __cpp(dbg_handle_hardfault_c)");
////	__asm("_MSP:");
////	__asm("MRS R0, MSP");
////	__asm("B __cpp(dbg_handle_hardfault_c)");
//}
//嵌入式汇编
// void HardFault_Handler(void)//__asm
//{
//	//IMPORT	dbg_handle_hardfault_c
////	TST LR, #4
////	ITE EQ
////	MRSEQ R0, MSP
////	MRSNE R0, PSP
////	B __cpp(dbg_handle_hardfault_c)
//		while(1){};
//}

/*-------------------------------------------------------------------
 * Function Name:
 *	dbg_handle_hardfault_c_without_reboot
 * Description:
 *	N/A
 * Parameters:
 *	hardfault_args:
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *	Version 1.00 - 4.4.2018 by LmnyL, Create.
 *-----------------------------------------------------------------*/
void dbg_handle_hardfault_c_without_reboot(uint32_t * hardfault_args)
{
static uint32_t stacked_r0;
static uint32_t stacked_r1;
static uint32_t stacked_r2;
static uint32_t stacked_r3;
static uint32_t stacked_r12;
static uint32_t stacked_lr;
static uint32_t stacked_pc;
static uint32_t stacked_psr;
static uint32_t *stack_addr;
static uint32_t debug_buf[MAX_DEBUG_BUF_LEN];
static uint32_t debug_buf_len = 0;
static uint32_t cpu_sp = 0;

	stacked_r0 = (hardfault_args[0]);
	stacked_r1 = (hardfault_args[1]);
	stacked_r2 = (hardfault_args[2]);
	stacked_r3 = (hardfault_args[3]);
	stacked_r12 = (hardfault_args[4]);
	stacked_lr = (hardfault_args[5]);
	stacked_pc = (hardfault_args[6]);
	stacked_psr = (hardfault_args[7]);

	stack_addr = (uint32_t *)hardfault_args;

	cpu_sp = __get_MSP();

	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED38)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED28)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED2C)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED30)));
	debug_buf[debug_buf_len++] = (*((volatile unsigned long *)(0xE000ED3C)));

	for (stacked_r1 = 0; stacked_r1 < MAX_DEBUG_STACK1_LEN; stacked_r1++)
	{
		debug_buf[debug_buf_len++] = *stack_addr;

		stack_addr++;
	}

	/* ??è?Stack */
	stack_addr = (uint32_t *)(cpu_sp);
	for (stacked_r1 = 0; stacked_r1 < MAX_DEBUG_STACK2_LEN; stacked_r1++)
	{
		debug_buf[debug_buf_len++] = *stack_addr;

		stack_addr++;
	}
	debug_buf[debug_buf_len++] = DBG_BUF_MAGIC;

	/* Save To flash */
//	dbg_save_hardfaultinfo(debug_buf, debug_buf_len);

	DBG_Dump_HardInfo(debug_buf, debug_buf_len);
}

///*-------------------------------------------------------------------
// * Function Name:
// *	void
// * Description:
// *	N/A
// * Parameters:
// *	void      :
// * Return:
// *	N/A
// * Note:
// *	N/A
// * History:
// *	Version 1.00 - 4.4.2018 by LmnyL, Create.
// *-----------------------------------------------------------------*/
//__ASM void HardFault_HandlerWithOutReboot(void)
//{
//	IMPORT	dbg_handle_hardfault_c_without_reboot
//	TST LR, #4
//	ITE EQ
//	MRSEQ R0, MSP
//	MRSNE R0, PSP
//	B __cpp(dbg_handle_hardfault_c_without_reboot)
//}
/*===================================================================
 * FUNCTION NAME:
 *	DBG_Get_DbgBuffer
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	uint32_t:
 *	uint32_t:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.3.18 by LmnyL, Create
 *=================================================================*/
bool DBG_Get_DbgBuffer( uint32_t *info, uint32_t len)
{
//	uint32_t flash_write_base_addr = EEPROM_DBG_PAGE_START;

//	return PERIP_EEPROM_Read_ByOffset(EEPROM_PAGE_TO_OFFSET(flash_write_base_addr),(uint8_t*)info, 4*len);
}

/*===================================================================
 * FUNCTION NAME:
 *	DBG_Dump_IntrCnt
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	N/A
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2016.7.1 by LmnyL, Create
 *=================================================================*/
void DBG_Dump_IntrCnt(void)
{
	printf("\r\n Debug Intr Cnt:");

	printf("\r\n %-8s: %d", "nmi", dbg_intr_cnt.nmi_handler);
	printf("\r\n %-8s: %d", "uart1", dbg_intr_cnt.usart_1_handler);
	printf("\r\n %-8s: %d", "uart2", dbg_intr_cnt.usart_2_handler);
	printf("\r\n %-8s: %d", "uart3", dbg_intr_cnt.usart_3_handler);
	printf("\r\n %-8s: %d", "uart4", dbg_intr_cnt.usart_4_handler);
	printf("\r\n %-8s: %d", "uart5", dbg_intr_cnt.usart_5_handler);
	printf("\r\n %-8s: %d", "uart6", dbg_intr_cnt.usart_6_handler);
	printf("\r\n %-8s: %d", "uart7", dbg_intr_cnt.usart_7_handler);
	printf("\r\n %-8s: %d", "uart8", dbg_intr_cnt.usart_8_handler);
	printf("\r\n %-8s: %d", "memmgt", dbg_intr_cnt.memmanage_handler);
	printf("\r\n %-8s: %d", "bus", dbg_intr_cnt.busfault_handler);
	printf("\r\n %-8s: %d", "usage", dbg_intr_cnt.usagefault_handler);
	printf("\r\n %-8s: %d", "svc", dbg_intr_cnt.svc_handler);
	printf("\r\n %-8s: %d", "dbgmon", dbg_intr_cnt.dbgmon_handler);
	printf("\r\n %-8s: %d", "pendsv", dbg_intr_cnt.pendsv_handler);
	printf("\r\n %-8s: %d", "systick", dbg_intr_cnt.systick_handler);
	printf("\r\n %-8s: %d", "sdio", dbg_intr_cnt.sdio_handler);
	printf("\r\n %-8s: %d", "sdsdiodma", dbg_intr_cnt.sd_sdio_dma_handler);
	printf("\r\n %-8s: %d", "eth_rx", dbg_intr_cnt.eth_rx_handler);
	printf("\r\n %-8s: %d", "e9_5", dbg_intr_cnt.exit9_5_handler);
	printf("\r\n %-8s: %d", "e9_5_phy", dbg_intr_cnt.exit9_5_handler_phy);
	printf("\r\n %-8s: %d", "ptp_sx", dbg_intr_cnt.exit9_5_handler_5);
	printf("\r\n %-8s: %d", "ptp_rx", dbg_intr_cnt.exit0_handler);
	printf("\r\n %-8s: %d", "e_1", dbg_intr_cnt.exit1_handler);
	printf("\r\n %-8s: %d", "rtc", dbg_intr_cnt.exit2_handler);
	printf("\r\n %-8s: %d", "e15_10", dbg_intr_cnt.exit15_10_handler);
	printf("\r\n %-8s: %d", "e15_10_tdc_1pps", dbg_intr_cnt.exit15_10_handler_tdc_1pps);
	printf("\r\n %-8s: %d", "e15_10_tdc_npps", dbg_intr_cnt.exit15_10_handler_tdc_npps);
	printf("\r\n %-8s: %d", "t1cc", dbg_intr_cnt.tim1_cc_handler);
	printf("\r\n %-8s: %d", "t1cc1", dbg_intr_cnt.tim1_cc1_handler);
	printf("\r\n %-8s: %d", "t1cc2", dbg_intr_cnt.tim1_cc2_handler);
	printf("\r\n %-8s: %d", "t1cc3", dbg_intr_cnt.tim1_cc3_handler);
	printf("\r\n %-8s: %d", "t1cc4", dbg_intr_cnt.tim1_cc4_handler);
	printf("\r\n %-8s: %d", "t2cc", dbg_intr_cnt.tim2_cc_handler);
	printf("\r\n %-8s: %d", "t2cc1", dbg_intr_cnt.tim2_cc1_handler);
	printf("\r\n %-8s: %d", "t2cc2", dbg_intr_cnt.tim2_cc2_handler);
	printf("\r\n %-8s: %d", "t2cc3", dbg_intr_cnt.tim2_cc3_handler);
	printf("\r\n %-8s: %d", "t2cc4", dbg_intr_cnt.tim2_cc4_handler);
	printf("\r\n %-8s: %d", "t1_up_t10", dbg_intr_cnt.tim1_up_tim10_handler);
	printf("\r\n %-8s: %d", "t1_trg_com_t11", dbg_intr_cnt.tim1_trg_com_tim11_handler);
	printf("\r\n %-8s: %d", "t8_up_t13", dbg_intr_cnt.tim8_up_tim13_handler);
	printf("\r\n %-8s: %d", "t8_cc", dbg_intr_cnt.tim8_cc_handler);
	printf("\r\n %-8s: %d", "t9_cc", dbg_intr_cnt.tim9_cc_handler);
	printf("\r\n %-8s: %d", "can1_rx0", dbg_intr_cnt.can1_rx0_handler);
	printf("\r\n %-8s: %d", "t7", dbg_intr_cnt.tim7_handler);
	printf("\r\n %-8s: %d", "t4", dbg_intr_cnt.tim4_handler);
	printf("\r\n %-8s: %d", "t5", dbg_intr_cnt.tim5_handler);
}


/*===================================================================
 * FUNCTION NAME:
 *	DBG_Set_PrintState
 * DESCRIPTION:
 *	N/A
 * PARAMETERS:
 *	dbg_print_date:
 * RETURN:
 *	N/A
 * NOTES:
 *	N/A
 * HISTORY:
 *	Version 1.0 - 2015.1.22 by LmnyL, Create
 *=================================================================*/
bool DBG_Set_PrintStatem( DBG_PRT_T *dbg_print_date)
{
	bool module_exist = false;
	uint32_t module;

	/* Check ALL */
	if (!strcasecmp((char *)dbg_print_date->module_name, "ALL"))
	{
		/* Prt ON/off ALL modules */
		for (module = 0; module < MODULE_TYPE_END; module++)
		{
			if (module == MODULE_TYPE_SYS)
			{
				dbg_state[(MODULE_TYPE_T)module].state = true;
			}
			else
			{
				dbg_state[(MODULE_TYPE_T)module].state = dbg_print_date->state;
			}
		}

		return true;
	}

	if (!(dbg_print_date->module < MODULE_TYPE_END))
	{
		for (module = 0; module < MODULE_TYPE_END; module++)
		{
			if (strcasecmp((char *)dbg_print_date->module_name, module_name[(MODULE_TYPE_T)module]))
			{
				continue;
			}
			module_exist = true;
			break;
		}
		if (!module_exist)
		{
			return false;
		}
	}
	else
	{
		module = dbg_print_date->module;
	}

	if (module > MODULE_TYPE_END)
	{
		return false;
	}

	dbg_state[module].state = dbg_print_date->state;

	return true;
}

#if !RELEASE_INCLUDE
void DBG_AssertFailed(const char *file, const char *function, uint32_t line)
{
	printf("\r\nDebug abort: %s, %s, %u\r\n", file, function, line);
	
	while(1);
}
#endif
