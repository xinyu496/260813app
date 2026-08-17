/*===================================================================
 *
 * COPYRIGHT:
 *
 * FILE NAME:
 *	dbg.h
 * DESCRIPTION:
 *	N/A
 * HISTORY:
 *	Version 1.00 - 2015.1.22 by LmnyL, File Create
 *
 *=================================================================*/
#ifndef __DBG_H__
#define __DBG_H__
#include "base_inc.h"
//#include "option/opt_base_dbg.h"
#define SERIAL_BYPASS_ENABLE	"combypassenable"
#define SERIAL_BYPASS_DISABLE	"combypassdisable"

typedef struct
{
	uint32_t nmi_handler;
	uint32_t usart_1_handler;
	uint32_t usart_2_handler;
	uint32_t usart_3_handler;
	uint32_t usart_4_handler;
	uint32_t usart_5_handler;
	uint32_t usart_6_handler;
	uint32_t usart_7_handler;
	uint32_t usart_8_handler;
	uint32_t memmanage_handler;
	uint32_t busfault_handler;
	uint32_t usagefault_handler;
	uint32_t svc_handler;
	uint32_t dbgmon_handler;
	uint32_t pendsv_handler;
	uint32_t systick_handler;
	uint32_t sdio_handler;
	uint32_t sd_sdio_dma_handler;
	uint32_t eth_rx_handler;
	uint32_t exit9_5_handler;
	uint32_t exit9_5_handler_phy;
	uint32_t exit9_5_handler_5;
	uint32_t exit0_handler;
	uint32_t exit1_handler;
	uint32_t exit2_handler;
	uint32_t exit3_handler;
	uint32_t exit15_10_handler;
	uint32_t exit15_10_handler_tdc_1pps;
	uint32_t exit15_10_handler_tdc_npps;
	uint32_t tim1_cc_handler;
	uint32_t tim2_cc_handler;
	uint32_t tim2_cc1_handler;
	uint32_t tim2_cc2_handler;
	uint32_t tim2_cc3_handler;
	uint32_t tim2_cc4_handler;
	uint32_t tim1_cc1_handler;
	uint32_t tim1_cc2_handler;
	uint32_t tim1_cc3_handler;
	uint32_t tim1_cc4_handler;
	uint32_t tim1_up_tim10_handler;
	uint32_t tim1_trg_com_tim11_handler;
	uint32_t tim8_up_tim13_handler;
	uint32_t tim8_cc_handler;
	uint32_t tim9_cc_handler;
	uint32_t can1_rx0_handler;
	uint32_t tim7_handler;
	uint32_t tim4_handler;
	uint32_t tim5_handler;
}DBG_INTR_CNT_T;

extern DBG_INTR_CNT_T dbg_intr_cnt;

typedef enum
{
	MODULE_TYPE_SYS,			/* SYS */
	MODULE_TYPE_INFO,			/* INFO */
	MODULE_TYPE_DBG,
	MODULE_TYPE_TX,				/* TX */
	MODULE_TYPE_TXTS,			/* TX Timestamp */
	MODULE_TYPE_RX,				/* RX */
	MODULE_TYPE_RXTS,			/* RX Timestamp */
	MODULE_TYPE_PHY,			/* PHY */
	MODULE_TYPE_TIME,			/* SYSTEM Time */
	MODULE_TYPE_CFG,			/* Configure */
	MODULE_TYPE_PTP_TIMER,		/* PTP timer */
	MODULE_TYPE_PTP_MSG,		/* PTP Message */
	MODULE_TYPE_PTP_PKT,		/* PTP PKT */
	MODULE_TYPE_PTP_SERVO,		/* PTP Servo */
	MODULE_TYPE_PTP_UC,			/* PTP Unicast */
	MODULE_TYPE_PTP_TS,			/* PTP TS */
	MODULE_TYPE_PTP_TSPENDING,	/* PTP TS Pending */
	MODULE_TYPE_PTP_TADJ,		/* PTP Time Adjust */
	MODULE_TYPE_PTP_ERR,		/* PTP ERR */
	MODULE_TYPE_FLT,			/* utl filter */
	MODULE_TYPE_FLTDATA,		/* utl filter */
	MODULE_TYPE_NTP,			/* NTP */
	MODULE_TYPE_TFTP,			/* TFTP */
	MODULE_TYPE_SYNC,			/* Sync translate */
	MODULE_TYPE_SYNCDBG,		/* SYNC DBG */
	MODULE_TYPE_TAME,			/* OCXO Tame */
	MODULE_TYPE_TAMED,			/* OCXO Tame Data */
	MODULE_TYPE_TAMEDBG,		/* OCXO Tame Debug Data */
	MODULE_TYPE_REC,			/* REC */
	MODULE_TYPE_POS,			/* POS */
	MODULE_TYPE_GPS,			/* GPS */
	MODULE_TYPE_BD,				/* BD */
	MODULE_TYPE_UDP,			/* UDP */
	MODULE_TYPE_TOD,			/* TOD Send */
	MODULE_TYPE_BDC_RCV,		/* BDC Input */
	MODULE_TYPE_TC_BDCR_BACKUP,	/* BDC BackUp Input */
	MODULE_TYPE_PE,				/* PE */
	MODULE_TYPE_PE_DBG,			/* PE Debug */
	MODULE_TYPE_ENV,			/* ENV */
	MODULE_TYPE_DATA_SEND,		/* data send */
	MODULE_TYPE_DATA_READ,		/* data read */
	MODULE_TYPE_BDCCODE,		/* BDC Code */
	MODULE_TYPE_UI,				/* UI  */
	MODULE_TYPE_LEAP,			/* LEAP  */
	MODULE_TYPE_MULTISRC,		/* MULTISRC */
	MODULE_TYPE_TQ,				/* Time quality */
	MODULE_TYPE_HP,				/* HP */
	MODULE_TYPE_ANT,			/* ANT */
	MODULE_TYPE_SA45,			/* SA45 */
	MODULE_TYPE_TODRCV,			/* TOD Rcv */
	MODULE_TYPE_TDC,			/* TDC */
	MODULE_TYPE_TDCDBG,			/* TDC Debug */
	MODULE_TYPE_SATDBG,			/* SATELITES DBG */
	MODULE_TYPE_MONITOR,		/* MONITOR DBG */
	MODULE_TYPE_RTC,			/* RTC DBG */
	MODULE_TYPE_RB,				/* RB DBG */
	MODULE_TYPE_RBADJ,			/* RB ADJ */
	MODULE_TYPE_OC,				/* OC TAME */
	MODULE_TYPE_PLL,
	MODULE_TYPE_TDC_1,
	MODULE_TYPE_TDC_2,
	MODULE_TYPE_TDCDBG10M,
	MODULE_TYPE_RBDATA,			/* RB Data  */
	MODULE_TYPE_RBNOTE,			/* RB NOTE  */
	MODULE_TYPE_RBSG,			/* RB Sigma delta */
	MODULE_TYPE_IONUTC,			/* INONUC TAME */
	MODULE_TYPE_RR,				/* RCV BTM by CAN*/
	MODULE_TYPE_RT,				/* RCV HC by COM1 */
	MODULE_TYPE_HC,				/* hc */
	MODULE_TYPE_TAME_RB,		/* RB Tame */
	MODULE_TYPE_TAMED_RB,		/* RB Tame Data */
	MODULE_TYPE_TAMEDBG_RB,		/* RB Tame Debug Data */
	MODULE_TYPE_RD54,
	MODULE_TYPE_EXT_RS,
	MODULE_TYPE_EXT_RS_OFM,
	MODULE_TYPE_EXT_RS_MPD,
	MODULE_TYPE_DT,
	MODULE_TYPE_AVG,
    MODULE_TYPE_SNMP,

    MODULE_TYPE_TODLXR,
    MODULE_TYPE_TODLXS,

    MODULE_TYPE_MON_TCP_RX,
    MODULE_TYPE_MON_TCP_TX,

    MODULE_TYPE_MULTI_MSG,

    MODULE_TYPE_MON_UDP_RX,
    MODULE_TYPE_MON_UDP_TX,

    MODULE_TYPE_MON_LX_RX,
    MODULE_TYPE_MON_LX_TX,


	MODULE_TYPE_FPGA,
	MODULE_TYPE_QERRIN,
	MODULE_TYPE_QERR,
	MODULE_TYPE_FPGA_PE,
	MODULE_TYPE_STACKMSG,
	MODULE_TYPE_FPGA_STATUS,
	MODULE_TYPE_FPGA_CHECK,

	MODULE_TYPE_ANTI_SPOOF,
	MODULE_TYPE_ANTI_SPOOF_RESULT,
	MODULE_TYPE_ANTI_SPOOF_DEBUG,
	MODULE_TYPE_ANTI_SPOOFDP,
	MODULE_TYPE_ANTI_SPOOFDP_DEBUG,
	MODULE_TYPE_DPLL_TDC,
	MODULE_TYPE_INTERFERE,
	MODULE_TYPE_TAME_TEST,
	MODULE_TYPE_TAMERB_TEST,
	MODULE_TYPE_JAM,
	MODULE_TYPE_DPLL_TDC_DBG,
	MODULE_TYPE_INTERFACE,
	MODULE_TYPE_HDR,
	MODULE_TYPE_BDC_SEND,
	MODULE_TYPE_ETHRX_HEAD,
	MODULE_TYPE_ETHTX_HEAD,
	MODULE_TYPE_GNSS,
	MODULE_TYPE_NMEA,
	MODULE_TYPE_MEM_LEAK,
	MODULE_TYPE_GPS_DATA,
	MODULE_TYPE_BDC_DATA_IN,
	/* End */
	MODULE_TYPE_END,
}MODULE_TYPE_T;

#define MAX_MODULE_NAME_LEN 20

/* Module name */
#define DEBUG_MODULE_NAME	\
	"SYS",					\
	"INFO",					\
	"DBG",					\
	"TX",					\
	"TTS",					\
	"RX",					\
	"RTS",					\
	"PHY",					\
	"TIME",					\
	"CFG",					\
	"PTIMER",				\
	"PMSG",					\
	"PPKT",					\
	"PSERVO",				\
	"PUC",					\
	"PTS",					\
	"PTSP",					\
	"PTADJ",				\
	"PERR",					\
	"FILTER",				\
	"FILTERDATA",			\
	"NTP",					\
	"TFTP",					\
	"SYNC",					\
	"SYNCDBG",				\
	"TAME",					\
	"TAMED",				\
	"TAMEDBG",				\
	"REC",					\
	"POS",					\
	"GPS",					\
	"BD",					\
	"UDP",					\
	"TOD",					\
	"BDCRCV",				\
	"BDCRCVB",				\
	"PE",					\
	"PEDBG",				\
	"ENV",					\
	"DSEND",				\
	"DREAD",				\
	"BDCCODE",				\
	"UI",					\
	"LEAP",					\
	"MULTISRC",				\
	"TQ",					\
	"HP",					\
	"ANT",					\
	"SA45",					\
	"TODRCV",				\
	"TDC",					\
	"TDCDBG",				\
	"SATDBG",				\
	"MONITOR",				\
	"RTC",					\
	"RB",					\
	"RBADJ",				\
	"OC",					\
	"PLL",					\
	"TDC1",					\
	"TDC2",					\
	"TDCDBG10M",			\
	"RBDATA",				\
	"RBNOTE",				\
	"RBSG",					\
	"IONUTC",				\
	"RR",					\
	"RT",					\
	"HC",					\
	"TAMERB",				\
	"TAMEDRB",				\
	"TAMEDBGRB",			\
	"RD54",					\
	"EXTRS",				\
	"EXTRSOFM",				\
	"EXTRSMPD",				\
	"DT",					\
	"AVG",					\
	"SNMP",					\
	"TODLXR",				\
	"TODLXS",				\
	"MTCPRX",               \
	"MTCPTX",               \
	"MULTI",				\
	"MUDPRX",               \
	"MUDPTX",               \
	"MLXRX",                \
	"MLXTX",                \
	"FPGA",             	\
	"QERRIN",             	\
	"QERR",                	\
	"FPGAPE",               \
	"STACKMSG",             \
	"FPGASTATUS",          	\
	"FPGACHECK",          	\
	"ANTISPOOF",          	\
	"ANTISPOOFRES",        	\
	"ANTISPOOFDBG",    		\
	"ANTISPOOFDP",          \
	"ANTISPOOFDPDBG",       \
	"DPLLTDC",       		\
	"INTERFERE",       		\
	"TAMETEST",       		\
	"TAMERBTEST",       	\
	"JAM",       			\
	"DPLLTDCDBG",       	\
	"INTERFACE",       		\
	"HDR",       			\
	"BDCSEND",       		\
	"EHHDRRX",        		\
	"EHHDRTX",        		\
	"GNSS",        		\
	"NMEA",				\
	"LEAK",		          	\
	"GPSDATA", \
	"BDCINFO", \
	NULL

typedef struct
{
	MODULE_TYPE_T module;
	bool state;
	uint8_t module_name[MAX_MODULE_NAME_LEN];
}DBG_PRT_T;

/* External */
extern DBG_PRT_T dbg_state[MODULE_TYPE_END];

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
uint8_t *DBG_Get_ModuleName(MODULE_TYPE_T module);

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
bool DBG_Set_PrintState(DBG_PRT_T *dbg_print_date);

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
bool DBG_Set_PrintStateByModuleType(MODULE_TYPE_T module_type,bool state);

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
bool DBG_Get_PrintState(MODULE_TYPE_T module);

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
bool DBG_Init(void);

#define KERNEL_DBG_Print(module, format, ...) \
{\
if(dbg_state[module].state)\
printf(format, ##__VA_ARGS__);\
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
//__asm void HardFault_Handler(void);
//__attribute__((naked,used)) void HardFault_Handler(void);
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
void DBG_Clear_HardFaultInfo(void);

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
bool DBG_Get_DbgBuffer(uint32_t *info, uint32_t len);
void dbg_handle_hardfault_c(uint32_t * hardfault_args);
/*===================================================================
 *
 *	Put your Debug API under here
 *
 *=================================================================*/
#define DBG_INFO_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_INFO, x, ##__VA_ARGS__)
#define DBG_TX_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TX, x, ##__VA_ARGS__)
#define DBG_DBG_Print(x, ...)		KERNEL_DBG_Print(MODULE_TYPE_DBG, x, ##__VA_ARGS__)
#define DBG_TX_TS_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TXTS, x, ##__VA_ARGS__)
#define DBG_RX_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_RX, x, ##__VA_ARGS__)
#define DBG_RX_TS_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_RXTS, x, ##__VA_ARGS__)
#define DBG_TIME_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TIME, x, ##__VA_ARGS__)
#define DBG_PTP_TIMER_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_TIMER, x, ##__VA_ARGS__)
#define DBG_PTP_MSG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_MSG, x, ##__VA_ARGS__)
#define DBG_PTP_PKT_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_PKT, x, ##__VA_ARGS__)
#define DBG_PTP_SERVO_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_SERVO, x, ##__VA_ARGS__)
#define DBG_PTP_UNICAST_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_PTP_UC, x, ##__VA_ARGS__)
#define DBG_PTP_TS_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_TS, x, ##__VA_ARGS__)
#define DBG_PTP_TS_Pending_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_PTP_TSPENDING, x, ##__VA_ARGS__)
#define DBG_PTP_TADJ_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_TADJ, x, ##__VA_ARGS__)
#define DBG_FLT_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_FLT, x, ##__VA_ARGS__)
#define DBG_FLT_DATA_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_FLTDATA, x, ##__VA_ARGS__)
#define DBG_NTP_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_NTP, x, ##__VA_ARGS__)
#define DBG_TFTP_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TFTP, x, ##__VA_ARGS__)
#define DBG_SYNC_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_SYNC, x, ##__VA_ARGS__)
#define DBG_TAME_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TAME, x, ##__VA_ARGS__)
#define DBG_TAME_DATA_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TAMED, x, ##__VA_ARGS__)
#define DBG_TAME_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TAMEDBG, x, ##__VA_ARGS__)
#define DBG_REC_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_REC, x, ##__VA_ARGS__)
#define DBG_POS_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_POS, x, ##__VA_ARGS__)
#define DBG_GPS_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_GPS, x, ##__VA_ARGS__)
#define DBG_BD_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_BD, x, ##__VA_ARGS__)
#define DBG_UDP_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_UDP, x, ##__VA_ARGS__)
#define DBG_CFG_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_CFG, x, ##__VA_ARGS__)
#define DBG_TOD_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TOD, x, ##__VA_ARGS__)
#define DBG_PE_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_PE, x, ##__VA_ARGS__)
#define DBG_BDC_Rcv_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_BDC_RCV, x, ##__VA_ARGS__)
#define DBG_BDC_BackUp_Rcv_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_TC_BDCR_BACKUP, x, ##__VA_ARGS__)
#define DBG_ENV_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_ENV, x, ##__VA_ARGS__)
#define DBG_PE_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PE_DBG, x, ##__VA_ARGS__)
#define DBG_SYS_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_SYS, x, ##__VA_ARGS__)
#define DBG_DataSend_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_DATA_SEND, x, ##__VA_ARGS__)
#define DBG_DateRead_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_DATA_READ, x, ##__VA_ARGS__)
#define DBG_BDCCODE_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_BDCCODE, x, ##__VA_ARGS__)
#define DBG_UI_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_UI, x, ##__VA_ARGS__)
#define DBG_LEAP_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_LEAP, x, ##__VA_ARGS__)
#define DBG_MULTISRC_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_MULTISRC, x, ##__VA_ARGS__)
#define DBG_TIMEQUALITY_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_TQ, x, ##__VA_ARGS__)
#define DBG_HP_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_HP, x, ##__VA_ARGS__)
#define DBG_ANT_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_ANT, x, ##__VA_ARGS__)
#define DBG_SA45_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_SA45, x, ##__VA_ARGS__)
#define DBG_PHY_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_PHY, x, ##__VA_ARGS__)
#define DBG_PTP_Err_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PTP_ERR, x, ##__VA_ARGS__)
#define DBG_TODRCV_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TODRCV, x, ##__VA_ARGS__)
#define DBG_TDC_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TDC, x, ##__VA_ARGS__)
#define DBG_TDC_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TDCDBG, x, ##__VA_ARGS__)
#define DBG_SAT_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_SATDBG, x, ##__VA_ARGS__)
#define DBG_MONITOR_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_MONITOR, x, ##__VA_ARGS__)
#define DBG_RTC_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_RTC, x, ##__VA_ARGS__)
#define DBG_RB_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_RB, x, ##__VA_ARGS__)
#define DBG_RB_ADJ_DBG_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_RBADJ, x, ##__VA_ARGS__)
#define DBG_RB_OC_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_OC, x, ##__VA_ARGS__)
#define DBG_SYNCDBG_DBG_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_SYNCDBG, x, ##__VA_ARGS__)
#define DBG_PLL_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_PLL, x, ##__VA_ARGS__)
#define DBG_TDC_1_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TDC_1, x, ##__VA_ARGS__)
#define DBG_TDC_2_Print(x, ...)		    KERNEL_DBG_Print(MODULE_TYPE_TDC_2, x, ##__VA_ARGS__)
#define DBG_TDC10M_DBG_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_TDCDBG10M, x, ##__VA_ARGS__)
#define DBG_RB_DATA_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_RBDATA, x, ##__VA_ARGS__)
#define DBG_RB_NOTE_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_RBNOTE, x, ##__VA_ARGS__)
#define DBG_RB_SG_Print(x, ...)	        KERNEL_DBG_Print(MODULE_TYPE_RBSG, x, ##__VA_ARGS__)
#define DBG_IONUTC_DBG_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_IONUTC, x, ##__VA_ARGS__)
#define DBG_RR_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_RR, x, ##__VA_ARGS__)
#define DBG_HC_DBG_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_HC, x, ##__VA_ARGS__)
#define DBG_TAME_RB_Print(x, ...)		KERNEL_DBG_Print(MODULE_TYPE_TAME_RB, x, ##__VA_ARGS__)
#define DBG_TAME_DATA_RB_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_TAMED_RB, x, ##__VA_ARGS__)
#define DBG_TAME_DBG_RB_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_TAMEDBG_RB, x, ##__VA_ARGS__)
#define DBG_RD54_Print(x, ...)			KERNEL_DBG_Print(MODULE_TYPE_RD54, x, ##__VA_ARGS__)
#define DBG_EXT_RS_Print(x, ...)		KERNEL_DBG_Print(MODULE_TYPE_EXT_RS, x, ##__VA_ARGS__)
#define DBG_EXT_RS_OFM_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_EXT_RS_OFM, x, ##__VA_ARGS__)
#define DBG_EXT_RS_MPD_Print(x, ...)	KERNEL_DBG_Print(MODULE_TYPE_EXT_RS_MPD, x, ##__VA_ARGS__)
#define DBG_DT_Print(x, ...)			KERNEL_DBG_Print(MODULE_TYPE_DT, x, ##__VA_ARGS__)
#define DBG_AVGPrint(x, ...)			KERNEL_DBG_Print(MODULE_TYPE_AVG, x, ##__VA_ARGS__)
#define DBG_SNMP_Print(x, ...)	        KERNEL_DBG_Print(MODULE_TYPE_SNMP, x, ##__VA_ARGS__)
#define DBG_TODLXR_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TODLXR, x, ##__VA_ARGS__)
#define DBG_TODLXS_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_TODLXS, x, ##__VA_ARGS__)
#define DBG_MON_TCP_TX_Print(x, ...)    KERNEL_DBG_Print(MODULE_TYPE_MON_TCP_TX, x, ##__VA_ARGS__)
#define DBG_MON_TCP_RX_Print(x, ...)    KERNEL_DBG_Print(MODULE_TYPE_MON_TCP_RX, x, ##__VA_ARGS__)
#define DBG_MULTI_Print(x, ...)	        KERNEL_DBG_Print(MODULE_TYPE_MULTI_MSG, x, ##__VA_ARGS__)
#define DBG_MON_UDP_TX_Print(x, ...)    KERNEL_DBG_Print(MODULE_TYPE_MON_UDP_TX, x, ##__VA_ARGS__)
#define DBG_MON_LX_RX_Print(x, ...)     KERNEL_DBG_Print(MODULE_TYPE_MON_LX_RX, x, ##__VA_ARGS__)
#define DBG_FPGA_Print(x, ...)          KERNEL_DBG_Print(MODULE_TYPE_FPGA, x, ##__VA_ARGS__)
#define DBG_QERR_Print(x, ...)     		KERNEL_DBG_Print(MODULE_TYPE_QERR, x, ##__VA_ARGS__)
#define DBG_QERRIN_Print(x, ...)     	KERNEL_DBG_Print(MODULE_TYPE_QERRIN, x, ##__VA_ARGS__)
#define DBG_FPGA_PE_Print(x, ...)     	KERNEL_DBG_Print(MODULE_TYPE_FPGA_PE, x, ##__VA_ARGS__)
#define DBG_STACKMSG_Print(x, ...)     	KERNEL_DBG_Print(MODULE_TYPE_STACKMSG, x, ##__VA_ARGS__)
#define DBG_FPGA_STATUS_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_FPGA_STATUS, x, ##__VA_ARGS__)
#define DBG_FPGA_CHECK_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_FPGA_CHECK, x, ##__VA_ARGS__)
#define DBG_ANTI_SPOOF_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_ANTI_SPOOF, x, ##__VA_ARGS__)
#define DBG_ANTI_SPOOF_RESULT_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_ANTI_SPOOF_RESULT, x, ##__VA_ARGS__)
#define DBG_ANTI_SPOOF_Debug_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_ANTI_SPOOF_DEBUG, x, ##__VA_ARGS__)
#define DBG_ANTI_SPOOF_DP_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_ANTI_SPOOFDP, x, ##__VA_ARGS__)
#define DBG_ANTI_SPOOF_DP_Debug_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_ANTI_SPOOFDP_DEBUG, x, ##__VA_ARGS__)
#define DBG_DPLL_TDC_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_DPLL_TDC, x, ##__VA_ARGS__)
#define DBG_INTERFERE_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_INTERFERE, x, ##__VA_ARGS__)
#define DBG_TAME_TEST_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_TAME_TEST, x, ##__VA_ARGS__)
#define DBG_TAMERB_TEST_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_TAMERB_TEST, x, ##__VA_ARGS__)
#define DBG_JAM_Print(x, ...) 			KERNEL_DBG_Print(MODULE_TYPE_JAM, x, ##__VA_ARGS__)
#define DBG_DPLL_TDC_DBG_Print(x, ...) 		KERNEL_DBG_Print(MODULE_TYPE_DPLL_TDC_DBG, x, ##__VA_ARGS__)
#define DBG_INTERFACE_Print(x, ...) 	KERNEL_DBG_Print(MODULE_TYPE_INTERFACE, x, ##__VA_ARGS__)
#define DBG_HDR_Print(x, ...) 			KERNEL_DBG_Print(MODULE_TYPE_HDR, x, ##__VA_ARGS__)
#define DBG_BDC_Send_Print(x, ...)	    KERNEL_DBG_Print(MODULE_TYPE_BDC_SEND, x, ##__VA_ARGS__)
#define DBG_ETH_RX_HDR_Print(x, ...)	    	KERNEL_DBG_Print(MODULE_TYPE_ETHRX_HEAD, x, ##__VA_ARGS__)
#define DBG_ETH_TX_HDR_Print(x, ...)	    	KERNEL_DBG_Print(MODULE_TYPE_ETHTX_HEAD, x, ##__VA_ARGS__)
#define DBG_GNSS_Print(x, ...)	    	KERNEL_DBG_Print(MODULE_TYPE_GNSS, x, ##__VA_ARGS__)
#define DBG_NMEA_Print(x, ...)	    	KERNEL_DBG_Print(MODULE_TYPE_NMEA, x, ##__VA_ARGS__)
#define DBG_MEM_LEAK_Print(x, ...) 		KERNEL_DBG_Print(MODULE_TYPE_MEM_LEAK, x, ##__VA_ARGS__)
#define DBG_EXIT_GPS_Print(x, ...) 		KERNEL_DBG_Print(MODULE_TYPE_GPS_DATA, x, ##__VA_ARGS__)
#define DBG_BDC_INFO_Print(x, ...) 		KERNEL_DBG_Print(MODULE_TYPE_BDC_DATA_IN, x, ##__VA_ARGS__)
/*===================================================================
 *
 *	assert for unit test
 *
 *=================================================================*/
#if !RELEASE_INCLUDE
void DBG_AssertFailed(const char *file, const char *function, uint32_t line);
#define DBG_ASSERT(expr) ((expr) ? (void)0 : DBG_AssertFailed(__FILE__, __FUNCTION__, __LINE__))
#else
#define DBG_ASSERT(expr) ((void)0)
#endif /* RELEASE_INCLUDE */

#endif

