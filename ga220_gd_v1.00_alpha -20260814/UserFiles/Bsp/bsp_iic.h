#ifndef __BSP_IIC_H
#define __BSP_IIC_H
#include "Common/base_inc.h"
#include "bsp_gpio.h"
/* Includes ------------------------------------------------------------------*/
#if defined (STM32H743xx)||(STM32H7A3xx)
#include "i2c.h"
#endif
#define IIC_DEVICE_ADDR	0xD2

#define IIC_W	0	//iic写操作位	
#define IIC_R	1	//iic读操作位	


#define SDA_PORT	GPIOB
#define SDA_PIN		gpioPin7

#define SCL_PORT	GPIOB
#define SCL_PIN     gpioPin6


#define SCL_HIGH()  SYS_GPIO_PIN_Set(SCL_PORT, SCL_PIN , 1)	    
#define SCL_LOW()   SYS_GPIO_PIN_Set(SCL_PORT, SCL_PIN , 0)	
                                                        
#define SDA_HIGH()  SYS_GPIO_PIN_Set(SDA_PORT, SDA_PIN , 1)	    
#define SDA_LOW()   SYS_GPIO_PIN_Set(SDA_PORT, SDA_PIN , 0) 	

#define SDA_READ()  SYS_GPIO_PIN_Get(SDA_PORT, SDA_PIN)		//读SDA口线状态 

//#define hardware_iic_mode    0x01							//启动硬件IIC宏


#if hardware_iic_mode

uint32_t IIC_WRITE_DATA(I2C_HandleTypeDef hi2c , uint16_t slave_addr , uint8_t reg_addr , uint8_t* reg_dat ,uint16_t len);	 //向从机写入多个字节     /************** 硬件iic ******************/
uint32_t IIC_WRITE_Onebyte(I2C_HandleTypeDef hi2c , uint16_t slave_addr , uint8_t reg_addr , uint8_t *reg_dat);				 //向从机写入一个字节	  /************** 硬件iic ******************/
uint32_t IIC_READ_DATA(I2C_HandleTypeDef hi2c , uint16_t slave_addr , uint8_t reg_addr , uint8_t* data_head , uint16_t len); //向从机读取数据         /************** 硬件iic ******************/
#else
uint32_t IIC_WRITE_DATA(uint16_t slave_addr , uint8_t reg_addr , uint8_t* reg_dat ,uint16_t len);	 //向从机写入多个字节     /************** 软件iic ******************/
uint32_t IIC_WRITE_Onebyte(uint16_t slave_addr , uint8_t reg_addr , uint8_t reg_dat);				 //向从机写入一个字节	  /************** 软件iic ******************/
uint32_t IIC_READ_DATA(uint16_t slave_addr , uint8_t reg_addr , uint8_t* data_head , uint16_t len);//向从机读取数据         /************** 软件iic ******************/
#endif





void delayUs(void);



#endif

