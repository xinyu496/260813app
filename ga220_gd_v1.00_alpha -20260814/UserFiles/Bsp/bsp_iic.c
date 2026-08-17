#include "bsp_iic.h"
#include "stm32f4xx_hal_i2c.h"

/***************************模拟IIC***************************/
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:1us延时(i=30)
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
void delayUs(void)
{
	uint8_t i;

	for(i = 0 ; i < 30 ; i++);
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC 起始信号(模拟)
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void IIC_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    delayUs();
    SDA_LOW();
    delayUs();
    SCL_LOW();
    delayUs();
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC 停止信号(模拟)
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void IIC_Stop(void)
{
    SDA_LOW();
    SCL_HIGH();
    delayUs();
    SDA_HIGH();
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC写一个字节
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void IIC_SendOneByte(uint8_t data)
{
    uint8_t i = 0;
    //发送字节的高7位
    for ( i = 0; i < 8; i++)
	{		
		if (data & 0x80)
		{
			SDA_HIGH();
		}
		else
		{
			SDA_LOW();
		}
		delayUs();
		SCL_HIGH();
		delayUs();
		SCL_LOW();
		if (i == 7)
		{
			 SDA_HIGH(); // 释放总线
		}
		data <<= 1;	/* 左移一个bit */
		delayUs();
	}
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC读一个字节
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint8_t IIC_ReadOneByte(void)
{
    uint8_t i;
	uint8_t value;

	/* 读到第1个bit为数据的bit7 */
	value = 0;
	for (i = 0; i < 8; i++)
	{
		value <<= 1;
		SCL_HIGH();
		delayUs();
		if (SDA_READ())
		{
			value++;
		}
		SCL_LOW();
		delayUs();
	}
	return value;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC等待应答信号
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static uint8_t IIC_WaitAck(void)
{
    uint8_t data;
    
    SDA_HIGH();
    delayUs();
    SCL_HIGH();
    delayUs();
    if (SDA_READ())	/* CPU读取SDA口线状态 */
	{
		data = 1;
	}
	else
	{
		data = 0;
	}
    SCL_LOW();
    delayUs();
    
    return data;
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC产生应答信号
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void IIC_Ack(void)
{
    SDA_LOW();
    delayUs();
    SCL_HIGH();
    delayUs();
    SCL_LOW();
    delayUs();
    SDA_HIGH();
}
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:IIC产生非应答信号
*PARAMETERS:
*RETURN:
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
static void IIC_NAck(void)
{
    SDA_HIGH();
    delayUs();
    SCL_HIGH();
    delayUs();
    SCL_LOW();
    delayUs();
}

#if hardware_iic_mode
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机寄存器写入多个字节
*PARAMETERS: slave_addr SLAVE地址   reg_addr：寄存器地址  reg_dat：内存地址  len：写入字节数量  
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_WRITE_DATA(I2C_HandleTypeDef hi2c , uint16_t slave_addr , uint8_t reg_addr , uint8_t* reg_dat ,uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&hi2c , slave_addr , reg_addr , I2C_MEMADD_SIZE_8BIT , reg_dat , len , 100);
	while(HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY);

	while (HAL_I2C_IsDeviceReady(&hi2c, 0xA0, 300, 300) == HAL_TIMEOUT);

	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机寄存器写入1个字节
*PARAMETERS:reg_addr：寄存器地址  reg_dat：写入数据
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_WRITE_Onebyte(I2C_HandleTypeDef hi2c ,uint16_t slave_addr , uint8_t reg_addr , uint8_t *reg_dat)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_I2C_Mem_Write(&hi2c , slave_addr , reg_addr , I2C_MEMADD_SIZE_8BIT , reg_dat , 1 , 100);
	while(HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY);

	while (HAL_I2C_IsDeviceReady(&hi2c, 0xA0, 300, 300) == HAL_TIMEOUT);

	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY)
	{
		
	}
	return status;
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机读取数据
*PARAMETERS:reg_addr:寄存器地址  data_head：内存地址  len：读取的字节数量
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_READ_DATA(I2C_HandleTypeDef hi2c ,uint16_t slave_addr , uint8_t reg_addr , uint8_t* data , uint16_t len)
{
	HAL_StatusTypeDef status = HAL_OK;
	
	status=HAL_I2C_Mem_Read(&hi2c,slave_addr,reg_addr, I2C_MEMADD_SIZE_8BIT, data, len,1000);

	return status;
}
#else
/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机寄存器写入多个字节
*PARAMETERS:reg_addr：寄存器地址  reg_dat：内存地址  len：写入字节数量
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_WRITE_DATA(uint16_t slave_addr , uint8_t reg_addr , uint8_t* reg_dat ,uint16_t len)
{
	uint16_t i;
//	IIC_Stop();									
	IIC_Start();
	IIC_SendOneByte(slave_addr | IIC_W);	//此处是写指令 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_SendOneByte(reg_addr);					//写入寄存器地址 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	for(i = 0 ; i < len ; i++)
	{
		IIC_SendOneByte(reg_dat[i]);			//写入数据

		if (IIC_WaitAck() != 0)
		{
		  goto cmd_fail;						//未收到应答信号
		}
	}

	IIC_Stop();
	return 0;
cmd_fail:
	IIC_Stop();
	return 1;
}

/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机寄存器写入1个字节
*PARAMETERS:reg_addr：寄存器地址  reg_dat：写入数据
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_WRITE_Onebyte(uint16_t slave_addr , uint8_t reg_addr , uint8_t reg_dat)
{
//	IIC_Stop();									
	IIC_Start();
	IIC_SendOneByte(slave_addr | IIC_W);	//此处是写指令 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_SendOneByte(reg_addr);					//写入寄存器地址 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_SendOneByte(reg_dat);					//写入数据

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_Stop();
	return 0;
cmd_fail:
	IIC_Stop();
	return 1;
}


/*==============================================================
*FUNCTION NAME:
*DISCRIPTION:iic向从机读取数据
*PARAMETERS:reg_addr:寄存器地址  data_head：内存地址  len：读取的字节数量
*RETURN:0：成功 1：失败
*N/A
*NOTES:
*HISTORY:
*==============================================================*/
uint32_t IIC_READ_DATA(uint16_t slave_addr , uint8_t reg_addr , uint8_t* data_head , uint16_t len)
{
//	IIC_Stop();									
	IIC_Start();
	IIC_SendOneByte(slave_addr | IIC_W);	//此处是写指令 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_SendOneByte(reg_addr);					//写入寄存器地址 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	IIC_Stop();		
	IIC_Start();

	IIC_SendOneByte(slave_addr | IIC_R);	//此处是读指令 

	if (IIC_WaitAck() != 0)
	{
	  goto cmd_fail;							//未收到应答信号
	}

	while(len)
	{
		*data_head = IIC_ReadOneByte();
		data_head++;
		if(len == 1)
		{
			IIC_NAck();
		}
		else		
		{
			IIC_Ack();
		}
		len--;
	}

	IIC_Stop();
	return 0;
cmd_fail:
	IIC_Stop();
	return 1;
}
#endif



/***************************硬件IIC***************************/























