#include  "icm_42688_p.h"

//***************************   采集ICM-42688-p陀螺   ***********************//
Icm42688_DataTypeDef Icm42688_Data;

uint8_t ICM42688P_Init_flg = 1;
uint8_t ICM42688P_ReadReg_databuf[32];
uint8_t ICM42688P_WriteReg_databuf[32];

//***************************   陀螺设置初始化    ************************//
void ICM42688P_Init ( void )
{
    //		if(ICM42688P_Init_flg)
    //		{
    static uint8_t ICM42688P_Reg_op = 0;
    switch ( ICM42688P_Reg_op )
    {
        case 0:
            ICM42688P_Reg_Read ( ICM42688P_WHO_AM_I_reg, ICM42688P_ReadReg_databuf, 1 ); //
            if ( ICM42688P_ReadReg_databuf[1] == 0x47 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 1:
            ICM42688P_WriteReg_databuf[0] = 0x44;
            ICM42688P_Reg_Write ( ICM42688P_GYRO_CONFIG0_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_GYRO_CONFIG0_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0x44 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 2:
            ICM42688P_WriteReg_databuf[0] = 0x24;
            ICM42688P_Reg_Write ( ICM42688P_ACCEL_CONFIG0_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_ACCEL_CONFIG0_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0x24 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 3:
            ICM42688P_WriteReg_databuf[0] = 0xE0;
            ICM42688P_Reg_Write ( ICM42688P_GYRO_CONFIG1_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_GYRO_CONFIG1_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0xE0 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 4:
            ICM42688P_WriteReg_databuf[0] = 0x77;
            ICM42688P_Reg_Write ( ICM42688P_GYRO_ACCEL_CONFIG0_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_GYRO_ACCEL_CONFIG0_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0x77 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 5:
            ICM42688P_WriteReg_databuf[0] = 0x00;
            ICM42688P_Reg_Write ( ICM42688P_ACCEL_CONFIG1_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_ACCEL_CONFIG1_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0x00 )
            {
                ICM42688P_Reg_op ++;
            }
            break;
        case 6:
            ICM42688P_WriteReg_databuf[0] = 0x03;
            ICM42688P_Reg_Write ( ICM42688P_GYRO_CONFIG_STATIC2_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Reg_Read ( ICM42688P_GYRO_CONFIG_STATIC2_reg, ICM42688P_ReadReg_databuf, 1 );
            if ( ICM42688P_ReadReg_databuf[1] == 0x03 )
            {
                ICM42688P_Reg_op ++;
            }
        case 7:
            //					ICM42688P_WriteReg_databuf[0] = 0x31;
            //					ICM42688P_Reg_Write(ICM42688P_ACCEL_CONFIG_STATIC2_reg,ICM42688P_WriteReg_databuf,1);
            //					ICM42688P_Reg_Read(ICM42688P_ACCEL_CONFIG_STATIC2_reg,ICM42688P_ReadReg_databuf,1);
            //					if(ICM42688P_ReadReg_databuf[1] == 0x31)
            //					{
            ICM42688P_Reg_op ++;
            //					}
            break;
        default:
            ICM42688P_WriteReg_databuf[0] = 0x1F;
            ICM42688P_Reg_Write ( ICM42688P_PWR_MGMT0_reg, ICM42688P_WriteReg_databuf, 1 );
            ICM42688P_Init_flg = 0;
            break;
    }
    //		}
}


//***************************   读取芯片寄存器    ************************//
//  reg_addr:是寄存器的地址
//  reg_buf: 写寄存器的内容
//  size:    内容的大小
void ICM42688P_Reg_Read ( uint8_t reg_addr, uint8_t *reg_buf, uint8_t size )
{
    uint8_t read_opbuf[size + 1];

    read_opbuf[0] = reg_addr;
    read_opbuf[0] |= 0x80;

    if ( size < 3 )
    {
        SPI1_CS_en;
        HAL_SPI_TransmitReceive ( &hspi1, read_opbuf, reg_buf, size + 1, 5 );
        SPI1_CS_dis;
    }
    else
    {
        SPI1_CS_dis;
        SPI1_CS_en;
        HAL_SPI_TransmitReceive_DMA ( &hspi1, read_opbuf, reg_buf, size + 1 );
    }
}


//***************************    写芯片寄存器    ************************//
//  reg_addr:是寄存器的地址
//  reg_buf: 写寄存器的内容
//  size:    内容的大小
void ICM42688P_Reg_Write ( uint8_t reg_addr, uint8_t *reg_buf, uint8_t size )
{
    uint8_t write_opbuf[size + 1];

    write_opbuf[0] = reg_addr;
    write_opbuf[0] &= 0x7F;
    for ( uint8_t i = 0; i < size; i++ )
    {
        write_opbuf[i + 1] = reg_buf[i];
    }

    if ( size < 3 )
    {
        SPI1_CS_en;
        HAL_SPI_Transmit ( &hspi1, write_opbuf, size + 1, 5 );
        SPI1_CS_dis;
    }
    else
    {
        SPI1_CS_dis;
        SPI1_CS_en;
        HAL_SPI_Transmit_DMA ( &hspi1, write_opbuf, size + 1 );
    }
}

//***************************    接收函数    ************************//
/*
 * @brief   数据获取
 * @param   无
 * @retval  无
 * @warning 不能写在定时器里
*/
void Receive_icm_42688 ( void )
{
    if ( ICM42688P_Init_flg )
    {
        //		HAL_Delay(10);
        ICM42688P_Init();
    }
    else
    {
        ICM42688P_Reg_Read ( ICM42688P_TEMP_DATA1_reg, ICM42688P_ReadReg_databuf, 14 );
    }
}

//***************************    数据接收回调函数    ************************//
//  输入接收到的数组和大小
//  返回温度、陀螺X、Y和Z轴的角速度和角加速度

void HAL_SPI_TxRxCpltCallback ( SPI_HandleTypeDef *hspi )
{
    Icm42688_Data.Temperature = ( int8_t ) ( ( ( ICM42688P_ReadReg_databuf[1] << 8 | ICM42688P_ReadReg_databuf[2] ) / 132.48 ) + 25 );
    Icm42688_Data.ACCEL_DATA_X = ( int16_t ) ( ICM42688P_ReadReg_databuf[3] << 8 | ICM42688P_ReadReg_databuf[4] );
    Icm42688_Data.ACCEL_DATA_Y = ( int16_t ) ( ICM42688P_ReadReg_databuf[5] << 8 | ICM42688P_ReadReg_databuf[6] );
    Icm42688_Data.ACCEL_DATA_Z = ( int16_t ) ( ICM42688P_ReadReg_databuf[7] << 8 | ICM42688P_ReadReg_databuf[8] );
    Icm42688_Data.GYRO_DATA_X = ( int16_t ) ( ICM42688P_ReadReg_databuf[9] << 8 | ICM42688P_ReadReg_databuf[10] );
    Icm42688_Data.GYRO_DATA_Y = ( int16_t ) ( ICM42688P_ReadReg_databuf[11] << 8 | ICM42688P_ReadReg_databuf[12] );
    Icm42688_Data.GYRO_DATA_Z = ( int16_t ) ( ICM42688P_ReadReg_databuf[13] << 8 | ICM42688P_ReadReg_databuf[14] );

}
