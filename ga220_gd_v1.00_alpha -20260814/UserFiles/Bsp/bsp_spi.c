#include "Bsp/bsp_spi.h" 
#include "Bsp/bsp_timer.h"
/* USER CODE BEGIN Includes */
typedef enum
{
	SPI_DRV_PIN_SDO,
	SPI_DRV_PIN_SDI,
	SPI_DRV_PIN_CLK,
	SPI_DRV_PIN_CS1,
	SPI_DRV_PIN_CS2,
	SPI_DRV_PIN_CS3,
	SPI_DRV_PIN_CS4,
	SPI_DRV_PIN_END,
}SPI_DRV_PIN_T;

typedef struct
{
	SPI_DRV_PIN_T pin_type;
	uint16_t gpio_pin;
	GPIO_TypeDef *gpio;
}SPI_DRV_MAP_T;

static SPI_DRV_MAP_T spi_drv_map[SPI_DRV_PIN_END] =
{
	{SPI_DRV_PIN_SDO, 			GPIO_PIN_5, 	GPIOE},
	{SPI_DRV_PIN_SDI, 			GPIO_PIN_3, 	GPIOE},
	{SPI_DRV_PIN_CLK, 			GPIO_PIN_6, 	GPIOE},
	{SPI_DRV_PIN_CS1,			GPIO_PIN_6, 	GPIOG},
	{SPI_DRV_PIN_CS2,			GPIO_PIN_4, 	GPIOC},
	{SPI_DRV_PIN_CS3,			GPIO_PIN_4, 	GPIOC},
	{SPI_DRV_PIN_CS4,			GPIO_PIN_4, 	GPIOC},
};

#define SPI_DRV_ADDR_MASK                      0x1FF
#define SPI_DRV_OFFSET_TO_READ_CMD(offset)		(uint16_t)(((offset) & SPI_DRV_ADDR_MASK) << 6)
#define SPI_DRV_OFFSET_TO_WRITE_CMD(offset)		(uint16_t)((((offset) & SPI_DRV_ADDR_MASK) << 6) | 0x8000)

#define SPI_DRV_CS1_Set(data)	HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CS1].gpio, spi_drv_map[SPI_DRV_PIN_CS1].gpio_pin, data)
#define SPI_DRV_CS2_Set(data)	HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CS2].gpio, spi_drv_map[SPI_DRV_PIN_CS2].gpio_pin, data)
#define SPI_DRV_CS3_Set(data)	HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CS3].gpio, spi_drv_map[SPI_DRV_PIN_CS3].gpio_pin, data)
#define SPI_DRV_CS4_Set(data)	HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CS4].gpio, spi_drv_map[SPI_DRV_PIN_CS4].gpio_pin, data)

#define SPI_DRV_SDO_H()		HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_SDO].gpio, spi_drv_map[SPI_DRV_PIN_SDO].gpio_pin, 1)
#define SPI_DRV_SDO_L()		HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_SDO].gpio, spi_drv_map[SPI_DRV_PIN_SDO].gpio_pin, 0)

#define SPI_DRV_CLK_H()		HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CLK].gpio, spi_drv_map[SPI_DRV_PIN_CLK].gpio_pin, 1)
#define SPI_DRV_CLK_L()		HAL_GPIO_WritePin(spi_drv_map[SPI_DRV_PIN_CLK].gpio, spi_drv_map[SPI_DRV_PIN_CLK].gpio_pin, 0)

#define SPI_DRV_SDI_Get()		HAL_GPIO_ReadPin(spi_drv_map[SPI_DRV_PIN_SDI].gpio, spi_drv_map[SPI_DRV_PIN_SDI].gpio_pin)

/*Assigned the specific pins for those control signal*/
#define SPI_DRV_PIN_CS1_Active()		SPI_DRV_CS1_Set(0)
#define SPI_DRV_PIN_CS1_Inactive()		SPI_DRV_CS1_Set(1)

#define SPI_DRV_PIN_CS2_Active()		SPI_DRV_CS2_Set(0)
#define SPI_DRV_PIN_CS2_Inactive()		SPI_DRV_CS2_Set(1)

#define SPI_DRV_PIN_CS3_Active()		SPI_DRV_CS3_Set(0)
#define SPI_DRV_PIN_CS3_Inactive()		SPI_DRV_CS3_Set(1)

#define SPI_DRV_PIN_CS4_Active()		SPI_DRV_CS4_Set(0)
#define SPI_DRV_PIN_CS4_Inactive()		SPI_DRV_CS4_Set(1)

#define SPI_DRV_send_1()\
{\
	SPI_DRV_CLK_L();\
	SPI_DRV_SDO_H();\
	SPI_DRV_CLK_H();\
}

#define SPI_DRV_send_0()\
{\
	SPI_DRV_CLK_L();\
	SPI_DRV_SDO_L();\
	SPI_DRV_CLK_H();\
}

/* USER CODE END Includes */
#ifdef HAL_SPI_MODULE_ENABLED
extern SPI_HandleTypeDef hspi1;
// SPI_HandleTypeDef hspi1;
#else
SPI_HandleTypeDef hspi1;
#endif

#define SPI_CONNECT_TIMEOUT 10
#if 1
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:SPI写
*PARAMETERS:reg_addr：写指令；reg_buf：写的内容；size：写的长度
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void BSP_SPI_Reg_Write(uint8_t reg_addr,uint8_t *reg_buf,uint8_t size)
{
	uint8_t write_opbuf[size+1];

	write_opbuf[0] = reg_addr;
	for(uint8_t i=0; i<size; i++)
	{
		write_opbuf[i + 1] = reg_buf[i];
	}
	
	if(size < 4)
	{
		SPI_DRV_PIN_CS1_Active();
		HAL_SPI_Transmit(&hspi1, write_opbuf, size+1, 5);
		SPI_DRV_PIN_CS1_Inactive();
	}
	else 
	{
		SPI_DRV_PIN_CS1_Inactive();
		SPI_DRV_PIN_CS1_Active();
		HAL_SPI_Transmit_DMA(&hspi1, write_opbuf, size+1);
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:SPI读
*PARAMETERS:reg_addr：读指令；reg_buf；读的内容；size：返回数据的长度
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void BSP_SPI_Reg_Read(uint8_t *reg_addr,uint8_t read_len,uint8_t *reg_buf,uint8_t size)
{
	uint8_t read_opbuf[size+read_len];
	
//	read_opbuf[0] = (reg_addr&0xff);
//	read_opbuf[1] = ((reg_addr>>8)&0xff);
//	read_opbuf[0] |= 0x80;//根据传感器手册进行更改
	memcpy(read_opbuf,reg_addr,read_len);

	if(size < 8)
	{
		SPI_DRV_PIN_CS1_Active();
		HAL_SPI_TransmitReceive(&hspi1, read_opbuf, reg_buf, size+read_len, 5);
		SPI_DRV_PIN_CS1_Inactive();
	}
	else 
	{
		SPI_DRV_PIN_CS1_Inactive();
		SPI_DRV_PIN_CS1_Active();
		HAL_SPI_TransmitReceive_DMA(&hspi1, read_opbuf, reg_buf, size+read_len);
	}
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:SPI采集数据完成时的回调，采集完成后中断标志位，
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t spi_rx_cnt = 0;
uint8_t spi_rx_callback_flag = 0;
void HAL_SPI_TxRxCpltCallback ( SPI_HandleTypeDef *hspi )
{
    if ( hspi == &hspi1 )
    {
        spi_rx_callback_flag = 0;
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:通信超时判断
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void SPI_Communicate_Judg_Timer(void)
{
	if (spi_rx_cnt++ > SPI_CONNECT_TIMEOUT)
	{
		spi_rx_callback_flag = 1;
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:获取通信超时标志位
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint8_t SPI_Connect_Sta(void)
{
	return spi_rx_callback_flag;
}

uint8_t recv_data[6] = {0};
#define READ_PRODUCT_ID  (0x9f)
#define WRITE_CMD  (0x02)
#define READ_CMD  (0x03)
void test_spi_info(void)
{
	uint8_t data_send = 0;
	uint8_t data_adress[10] = {0};
	data_send = READ_PRODUCT_ID;
	if (User_Tick.InnerGD >= 100)
	{
		BSP_SPI_Reg_Read(&data_send,1,recv_data,3);
		User_Tick.InnerGD = 0;
	}
	if ( User_Tick.CmdHandle == 5000)
	{
		data_send = WRITE_CMD;
		data_adress[0] = 0xff;
		data_adress[1] = 0x00;
		data_adress[2] = 0x00;
		data_adress[3] = 0x55;
		data_adress[4] = 0xaa;
		BSP_SPI_Reg_Write(data_send,data_adress,8);
	}
}
#else
/*-------------------------------------------------------------------
 * Function Name:
 *	spi_fgpa_htons
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint16_t SPI_DRV_Htons(uint16_t n)
{
  return ((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}

/*-------------------------------------------------------------------
 * Function Name:
 *	spi_fgpa_ntohs
 * Description:
 *	N/A
 * Parameters:
 *	n         :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint16_t SPI_DRV_Ntohs(uint16_t n)
{
  return ((n & 0xff) << 8) | ((n & 0xff00) >> 8);
}
void spi_delay(uint32_t delay)
{
	while(delay)
	delay--;
}

/*-------------------------------------------------------------------
 * Function Name:
 * Description:
 *	N/A
 * Parameters:
 *	data      :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
static uint8_t spi_drv_read_data(void)
{
	uint8_t tmp_data_ui8;
	uint8_t tmp_cnt;
	uint8_t read_data_ui8;

	tmp_data_ui8 = 0x80;
	read_data_ui8 = 0x00;

	for (tmp_cnt = 8; tmp_cnt > 0; tmp_cnt--)
	{
		SPI_DRV_CLK_L();
		spi_delay(5);
		if (SPI_DRV_SDI_Get())
		{
			read_data_ui8 |= tmp_data_ui8;
		}
		tmp_data_ui8 = tmp_data_ui8 >> 1;

		SPI_DRV_CLK_H();
		spi_delay(5);
	}

	SPI_DRV_CLK_L();
	spi_delay(5);

	return read_data_ui8;
}

/*-------------------------------------------------------------------
 * Function Name:
 *	fpga_drv_send_data
 * Description:
 *	N/A
 * Parameters:
 *	data      :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
static void spi_drv_send_data(uint8_t data)
{
	uint8_t tmp_data_ui8;
	uint8_t tmp_cnt;

	tmp_data_ui8 = 0x80;

	for (tmp_cnt = 8; tmp_cnt > 0; tmp_cnt--)
	{
		if ((data & tmp_data_ui8) == 0)
		{
			SPI_DRV_send_0();
		}
		else
		{
			SPI_DRV_send_1();
		}

		spi_delay(10);

		tmp_data_ui8 = tmp_data_ui8 >> 1;
	}

	SPI_DRV_CLK_L();
	spi_delay(10);
}

/*-------------------------------------------------------------------
 * Function Name:
 *	fpga_drv_send_bytes
 * Description:
 *	N/A
 * Parameters:
 *	txBuf     :
 *	txLen     :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
static void spi_drv_send_bytes(uint8_t *txBuf ,int txLen)
{
	int i;

	for (i = 0 ; i < txLen ;i++)
	{
		spi_drv_send_data(*(txBuf+i)) ;
	}
}
static void spi_drv_read_bytes(uint8_t *rxBuf ,int rxLen)
{
	int i;

	for (i = 0 ; i < rxLen ;i++)
	{
		*(rxBuf+i) = spi_drv_read_data() ;
	}
}

/*-------------------------------------------------------------------
 * Function Name:
 *	SPI_DRV_Read_16
 * Description:
 *	N/A
 * Parameters:
 *	offset    :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint16_t SPI_DRV_Read_16(uint16_t offset)
{
	uint16_t rx = 0;
	uint16_t send_data = 0;

	SPI_DRV_PIN_CS1_Active() ;

	send_data = SPI_DRV_OFFSET_TO_READ_CMD(offset);
	send_data = SPI_DRV_Htons(send_data);
	spi_drv_send_bytes((uint8_t *)&send_data, sizeof(uint16_t));

	spi_drv_read_bytes((uint8_t *)&rx, sizeof(uint16_t));
	rx = SPI_DRV_Ntohs(rx);
	SPI_DRV_PIN_CS_Inactive();
	return (rx);
}

/*-------------------------------------------------------------------
 * Function Name:
 *	SPI_DRV_Read_Serial
 * Description:
 *	N/A
 * Parameters:
 *	offset    :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint16_t SPI_DRV_Read_Serial(uint16_t offset, uint8_t *data, uint16_t read_bytes)
{
	uint16_t rx = 0;
	uint16_t send_data = 0;

	SPI_DRV_PIN_CS1_Active() ;

	send_data = SPI_DRV_OFFSET_TO_READ_CMD(offset);
	send_data = SPI_DRV_Htons(send_data);
	spi_drv_send_bytes((uint8_t *)&send_data, sizeof(uint16_t));

	spi_drv_read_bytes(data, read_bytes);

	rx = SPI_DRV_Ntohs(rx);

	SPI_DRV_PIN_CS1_Inactive();

	return (rx);
}

/*-------------------------------------------------------------------
 * Function Name:
 *	SPI_DRV_Write_16
 * Description:
 *	N/A
 * Parameters:
 *	offset    :
 *	value     :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint8_t SPI_DRV_Write_16(uint16_t offset, uint16_t value)
{
	uint16_t send_data = 0;

	SPI_DRV_PIN_CS1_Active() ;

	send_data = SPI_DRV_OFFSET_TO_WRITE_CMD(offset);
	send_data = SPI_DRV_Htons(send_data);
	spi_drv_send_bytes((uint8_t *)&send_data, sizeof(uint16_t));

	value = SPI_DRV_Htons(value);
	spi_drv_send_bytes((uint8_t *)&value, sizeof(uint16_t));

	SPI_DRV_PIN_CS1_Inactive();

	return 1;
}

/*-------------------------------------------------------------------
 * Function Name:
 *	SPI_DRV_Write_SerialData
 * Description:
 *	N/A
 * Parameters:
 *	offset    :
 *	value     :
 * Return:
 *	N/A
 * Note:
 *	N/A
 * History:
 *-----------------------------------------------------------------*/
uint8_t SPI_DRV_Write_SerialData(uint16_t offset, uint8_t *data_buf, uint16_t len)
{
	uint16_t i;
	uint16_t send_data = 0;
	uint16_t value = 0;

	for (i = 0; i < len; i++)
	{
		SPI_DRV_PIN_CS1_Active();

		send_data = SPI_DRV_OFFSET_TO_WRITE_CMD(offset);
		send_data = SPI_DRV_Htons(send_data);
		spi_drv_send_bytes((uint8_t *)&send_data, sizeof(uint16_t));

		value = SPI_DRV_Htons(*(data_buf + i));
		spi_drv_send_bytes((uint8_t *)&value, 2);

		SPI_DRV_PIN_CS1_Inactive();
	}

	return 1;
}
#endif











