#ifndef __ICM42688P_H
#define __ICM42688P_H

#include "main.h"
#include "stm32f4xx_hal.h"

//***************************    根据SPI对应的接口修改     ************************//
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

//***************************     预处理片选管脚的使能和禁能     ************************//
//需要修改为对应GPIO的管脚
#define SPI1_CS_en  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)
#define SPI1_CS_dis HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)

//***************************    预处理ICM-42688-P的寄存器地址     ************************//
#define ICM42688P_DEVICE_CONFIG_reg 0x11
#define ICM42688P_INT_CONFIG_reg 0x14
#define ICM42688P_TEMP_DATA1_reg 0x1D
#define ICM42688P_TEMP_DATA0_reg 0x1E
#define ICM42688P_ACCEL_DATA_X1_reg 0x1F
#define ICM42688P_ACCEL_DATA_X0_reg 0x20
#define ICM42688P_ACCEL_DATA_Y1_reg 0x21
#define ICM42688P_ACCEL_DATA_Y0_reg 0x22
#define ICM42688P_ACCEL_DATA_Z1_reg 0x23
#define ICM42688P_ACCEL_DATA_Z0_reg 0x24
#define ICM42688P_GYRO_DATA_X1_reg 0x25
#define ICM42688P_GYRO_DATA_X0_reg 0x26
#define ICM42688P_GYRO_DATA_Y1_reg 0x27
#define ICM42688P_GYRO_DATA_Y0_reg 0x28
#define ICM42688P_GYRO_DATA_Z1_reg 0x29
#define ICM42688P_GYRO_DATA_Z0_reg 0x2A
#define ICM42688P_GYRO_CONFIG0_reg 0x4F
#define ICM42688P_ACCEL_CONFIG0_reg 0x50
#define ICM42688P_GYRO_CONFIG1_reg 0x51
#define ICM42688P_GYRO_ACCEL_CONFIG0_reg 0x52
#define ICM42688P_ACCEL_CONFIG1_reg 0x53
#define ICM42688P_WHO_AM_I_reg 0x75
#define ICM42688P_GYRO_CONFIG_STATIC2_reg 0x0B
#define ICM42688P_ACCEL_CONFIG_STATIC2_reg 0x03
#define ICM42688P_PWR_MGMT0_reg 0x4E

//***************************   接收的数据结构体定义    ************************//
typedef struct
{
    int8_t  Temperature;
    int16_t ACCEL_DATA_X;
    int16_t ACCEL_DATA_Y;
    int16_t ACCEL_DATA_Z;
    int16_t GYRO_DATA_X;
    int16_t GYRO_DATA_Y;
    int16_t GYRO_DATA_Z;

} Icm42688_DataTypeDef;
extern Icm42688_DataTypeDef Icm42688_Data;

extern void ICM42688P_Init ( void );
extern void ICM42688P_Reg_Write ( uint8_t reg_addr, uint8_t *reg_val, uint8_t size );
extern void ICM42688P_Reg_Read ( uint8_t reg_addr, uint8_t *reg_val, uint8_t size );
extern void Receive_icm_42688 ( void );

#endif
