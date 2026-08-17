#include "../gyro/gyro_xv7011.h"
#include "Common/base_inc.h"
#include "Common/config.h"
#include "Common/utl_math.h"
#include "Bsp/bsp_uart.h"

extern SPI_HandleTypeDef hspi1;

uint8_t TempRd_gy1[] = {0x88, 0x00, 0x00, 0x00};
uint8_t RdRateData_gy1[] = {0x8A, 0x00, 0x00, 0x00};
uint8_t WrRateData_gy1[] = {0x0B, 0x05};
uint8_t WrDspSet1_gy1[] = {0x01, 0x01};
uint8_t WrDspSet2_gy1[] = {0x02, 0x0D};//// 0x06 100hz  0x09 200hz 0x0D 500hz 
uint8_t WrDspSet3_gy1[] = {0x03, 0x40};//40//0x42
uint8_t TempData_gy1[8] = {0x00};
uint8_t TempData1_gy1[8] = {0x00};
uint8_t Wr_Serial_Data_gy1[] = {0x1F, 0x00};
uint8_t Wr_Tformat_Data_gy1[] = {0x1C, 0x4B};

uint8_t TempRd_gy2[] = {0x88, 0x00, 0x00, 0x00};
uint8_t RdRateData_gy2[] = {0x8A, 0x00, 0x00, 0x00};
uint8_t WrRateData_gy2[] = {0x0B, 0x05};
uint8_t WrDspSet1_gy2[] = {0x01, 0x01};
uint8_t WrDspSet2_gy2[] = {0x02, 0x0D};
uint8_t WrDspSet3_gy2[] = {0x03, 0x40};
uint8_t TempData_gy2[8] = {0x00};
uint8_t TempData1_gy2[8] = {0x00};
uint8_t Wr_Serial_Data_gy2[] = {0x1F, 0x00};
uint8_t Wr_Tformat_Data_gy2[] = {0x1C, 0x4B};


uint8_t TempRd_gy3[] = {0x88, 0x00, 0x00, 0x00};
uint8_t RdRateData_gy3[] = {0x8A, 0x00, 0x00, 0x00};
uint8_t WrRateData_gy3[] = {0x0B, 0x05};
uint8_t WrDspSet1_gy3[] = {0x01, 0x01};
uint8_t WrDspSet2_gy3[] = {0x02, 0x0D};
uint8_t WrDspSet3_gy3[] = {0x03, 0x40};
uint8_t TempData_gy3[8] = {0x00};
uint8_t TempData1_gy3[8] = {0x00};
uint8_t Wr_Serial_Data_gy3[] = {0x1F, 0x00};
uint8_t Wr_Tformat_Data_gy3[] = {0x1C, 0x4B};

uint8_t Gyro_Status = 0;

uint8_t GetGyroFlag;

int16_t  GyroX_tmpr, GyroY_tmpr, GyroZ_tmpr;
int32_t GyroX, GyroY, GyroZ;

uint8_t loop_flag = 1;//while循环标志
uint8_t Data_collect = 0xff;

float groy_7011_Z1 = 0;
float groy_7011_Z1_old = 0;
float groy_7011_Z1_cha = 0;
float groy_7011_T1 = 0;
float groy_7011_Z1_sum = 0;
float test_cnt_1s1 = 0;
float groy_7011_Z1_sum_1s = 0;
float groy_7011_Z1_sum_1spj = 0;
float groy_7011_Z1_Bias = 0;
float groy_7011_T1_1s = 0;

float groy_7011_Z2 = 0;
float groy_7011_Z2_old = 0;
float groy_7011_Z2_cha = 0;
float groy_7011_T2 = 0;
float groy_7011_Z2_sum = 0;
float test_cnt_1s2 = 0;
float groy_7011_Z2_sum_1s = 0;
float groy_7011_Z2_sum_1spj = 0;
float groy_7011_Z2_Bias = 0.0;
float groy_7011_T2_1s = 0;

float groy_7011_Z3 = 0;
float groy_7011_Z3_old = 0;
float groy_7011_Z3_cha = 0;
float groy_7011_T3 = 0;
float groy_7011_Z3_sum = 0;
float test_cnt_1s3 = 0;
float groy_7011_Z3_sum_1s = 0;
float groy_7011_Z3_sum_1spj = 0;
float groy_7011_Z3_Bias = 0;
float groy_7011_T3_1s = 0;


uint8_t  flag_spi1_trx_cs1_dma_start1 = 0, flag_spi1_trx_cs1_dma_start2 = 0;
uint8_t  flag_spi1_trx_cs2_dma_start1 = 0, flag_spi1_trx_cs2_dma_start2 = 0;
uint8_t  flag_spi1_trx_cs3_dma_start1 = 0, flag_spi1_trx_cs3_dma_start2 = 0;


int16_t     GyroZ1_tmpr;
int32_t   GyroZ1;
int16_t     GyroZ2_tmpr;
int32_t GyroZ2;
int16_t     GyroZ3_tmpr;
int32_t GyroZ3;

uint8_t  flag_spi1_rx1_cs1 = 0, flag_spi1_rx2_cs1 = 0;
uint8_t  flag_spi1_rx1_cs2 = 0, flag_spi1_rx2_cs2 = 0;
uint8_t  flag_spi1_rx1_cs3 = 0, flag_spi1_rx2_cs3 = 0;
uint8_t  flag_fwzktx = 0;
uint8_t ConfigFlag = 1;
uint8_t  spi1_trx_Callback_flag = 0;

uint32_t SPI1_RX_cnt = 0;
uint32_t SPI1_RX_gy_cnt = 0;
uint32_t SPI1_RX_t_cnt = 0;

void Init_groy_XV7011BB()//初始化
{

        SPI1_CS1_disable;
        SPI1_CS2_disable;
        SPI1_CS3_disable;
        groy_7011_Z1_Bias = 0.0;
        groy_7011_Z2_Bias = 0.0;
        groy_7011_Z3_Bias = 0.0;
        HAL_Delay ( 1 );
        HAL_Delay ( 10 );
        HAL_Delay ( 1 );
        SPI1_CS2_enable;
        HAL_SPI_TransmitReceive ( &hspi1, Wr_Serial_Data_gy2, TempData_gy2, sizeof ( Wr_Serial_Data_gy2 ), 5 );
        SPI1_CS2_disable;
}

void Init_groy_XV7011BB_config()
{
    if ( GetGyroFlag )
    {
        if ( ConfigFlag )
        {
            ConfigFlag = 0;

            //陀螺1 配置
            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Serial_Data_gy1, TempData_gy1, sizeof ( Wr_Serial_Data_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Tformat_Data_gy1, TempData_gy1, sizeof ( Wr_Tformat_Data_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrRateData_gy1, TempData_gy1, sizeof ( WrRateData_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet1_gy1, TempData_gy1, sizeof ( WrDspSet1_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet2_gy1, TempData_gy1, sizeof ( WrDspSet2_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet3_gy1, TempData_gy1, sizeof ( WrDspSet3_gy1 ), 5 );
            SPI1_CS1_disable;

            HAL_Delay ( 1 );
            //

            //陀螺2 配置
            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Serial_Data_gy2, TempData_gy2, sizeof ( Wr_Serial_Data_gy2 ), 5 );
            SPI1_CS2_disable;

            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Tformat_Data_gy2, TempData_gy2, sizeof ( Wr_Tformat_Data_gy2 ), 5 );
            SPI1_CS2_disable;



            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrRateData_gy2, TempData_gy2, sizeof ( WrRateData_gy2 ), 5 );
            SPI1_CS2_disable;

            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet1_gy2, TempData_gy2, sizeof ( WrDspSet1_gy2 ), 5 );
            SPI1_CS2_disable;

            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet2_gy2, TempData_gy2, sizeof ( WrDspSet2_gy2 ), 5 );
            SPI1_CS2_disable;

            HAL_Delay ( 1 );
            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet3_gy2, TempData_gy2, sizeof ( WrDspSet3_gy2 ), 5 );
            SPI1_CS2_disable;

            HAL_Delay ( 1 );
            //
            //
            //陀螺3 配置
            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Serial_Data_gy3, TempData_gy3, sizeof ( Wr_Serial_Data_gy3 ), 5 );
            SPI1_CS3_disable;

            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, Wr_Tformat_Data_gy3, TempData_gy3, sizeof ( Wr_Tformat_Data_gy3 ), 5 );
            SPI1_CS3_disable;



            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrRateData_gy3, TempData_gy3, sizeof ( WrRateData_gy3 ), 5 );
            SPI1_CS3_disable;

            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet1_gy3, TempData_gy3, sizeof ( WrDspSet1_gy3 ), 5 );
            SPI1_CS3_disable;

            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet2_gy3, TempData_gy3, sizeof ( WrDspSet2_gy3 ), 5 );
            SPI1_CS3_disable;

            HAL_Delay ( 1 );
            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive ( &hspi1, WrDspSet3_gy3, TempData_gy3, sizeof ( WrDspSet3_gy3 ), 5 );
            SPI1_CS3_disable;

            HAL_Delay ( 1 );


        }
        else
        {
            SPI1_CS1_disable;
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, RdRateData_gy1, TempData_gy1, sizeof ( RdRateData_gy1 ) );
            flag_spi1_trx_cs1_dma_start1 = 1;
					
        }

        GetGyroFlag = 0;
    }
}
void Init_groy_XV7011BB_read()
{
        if ( flag_spi1_trx_cs1_dma_start1&&spi1_trx_Callback_flag)
        {
           spi1_trx_Callback_flag = 0;
            SPI1_RX_gy_cnt++;
            flag_spi1_rx1_cs1 = 1;
            flag_spi1_trx_cs1_dma_start1 = 0;
            SPI1_CS1_disable;
            if ( TempData_gy1[1] & 0x80 )
            {
                GyroZ1 = 0xFF000000 | TempData_gy1[1] << 16 | TempData_gy1[2] << 8 | TempData_gy1[3];
            }
            else
            {
                GyroZ1 = TempData_gy1[1] << 16 | TempData_gy1[2] << 8 | TempData_gy1[3];
            }
            groy_7011_Z1_old = groy_7011_Z1;
            groy_7011_Z1 = ( GyroZ1 / 71680.0 ) * 4 - groy_7011_Z1_Bias; //0.0568
            groy_7011_Z1_sum    = groy_7011_Z1_sum + groy_7011_Z1 * 0.001;
            groy_7011_Z1_cha = ( groy_7011_Z1 - groy_7011_Z1_old ) * 1000.0;

            groy_7011_Z1_sum_1s = groy_7011_Z1_sum_1s + groy_7011_Z1;
            test_cnt_1s1++;

            if ( test_cnt_1s1 == 1000.0 )
            {
                groy_7011_Z1_sum_1spj = groy_7011_Z1_sum_1s * 0.001;
                groy_7011_Z1_sum_1s = 0;
                test_cnt_1s1 = 0;
                groy_7011_T1_1s = groy_7011_T1;
            }
            SPI1_CS1_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, TempRd_gy1, TempData1_gy1, sizeof ( TempRd_gy1 ) );
            flag_spi1_trx_cs1_dma_start2 = 1;
        }
        else if ( flag_spi1_trx_cs1_dma_start2 &&spi1_trx_Callback_flag)
        {
		        spi1_trx_Callback_flag = 0;
            SPI1_RX_t_cnt++;
            flag_spi1_rx2_cs1 = 1;
            flag_spi1_trx_cs1_dma_start2 = 0;
            SPI1_CS1_disable;
            if ( TempData1_gy1[1] & 0x80 )
            {
                GyroZ1_tmpr = 0xF000 | TempData1_gy1[1] << 4 | ( TempData1_gy1[2] >> 4 );
            }
            else
            {
                GyroZ1_tmpr = TempData1_gy1[1] << 4 | ( TempData1_gy1[2] >> 4 );
            }
            groy_7011_T1 = GyroZ1_tmpr / 16.0;

            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, RdRateData_gy2, TempData_gy2, sizeof ( RdRateData_gy2 ) );
            flag_spi1_trx_cs2_dma_start1 = 1;
        }
        else if ( flag_spi1_trx_cs2_dma_start1 &&spi1_trx_Callback_flag)
        {
					  spi1_trx_Callback_flag = 0;
            SPI1_RX_gy_cnt++;
            flag_spi1_rx1_cs2 = 1;
            flag_spi1_trx_cs2_dma_start1 = 0;
            SPI1_CS2_disable;
            if ( TempData_gy2[1] & 0x80 )
            {
                GyroZ2 = 0xFF000000 | TempData_gy2[1] << 16 | TempData_gy2[2] << 8 | TempData_gy2[3];
            }
            else
            {
                GyroZ2 = TempData_gy2[1] << 16 | TempData_gy2[2] << 8 | TempData_gy2[3];
            }
            groy_7011_Z2_old = groy_7011_Z2;
            groy_7011_Z2 = ( GyroZ2 / 71680.0 ) * 4 - groy_7011_Z2_Bias; //0.0568
//            groy_7011_Z2 = ( GyroZ2 / 71680.0 ) * 4 - 0.25; //hh
            groy_7011_Z2_sum    = groy_7011_Z2_sum + groy_7011_Z2 * 0.001;
            groy_7011_Z2_cha = ( groy_7011_Z2 - groy_7011_Z2_old ) * 1000.0;

            groy_7011_Z2_sum_1s = groy_7011_Z2_sum_1s + groy_7011_Z2;
            test_cnt_1s2++;

            if ( test_cnt_1s2 == 1000.0 )
            {
                groy_7011_Z2_sum_1spj = groy_7011_Z2_sum_1s * 0.001;
                groy_7011_Z2_sum_1s = 0;
                test_cnt_1s2 = 0;
                groy_7011_T2_1s = groy_7011_T2;
            }

            SPI1_CS2_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, TempRd_gy2, TempData1_gy2, sizeof ( TempRd_gy2 ) );
            flag_spi1_trx_cs2_dma_start2 = 1;
        }
        else if ( flag_spi1_trx_cs2_dma_start2 &&spi1_trx_Callback_flag)
        {
			spi1_trx_Callback_flag = 0;
            SPI1_RX_t_cnt++;
            flag_spi1_rx2_cs2 = 1;
            flag_spi1_trx_cs2_dma_start2 = 0;
            SPI1_CS2_disable;
            if ( TempData1_gy2[1] & 0x80 )
            {
                GyroZ2_tmpr = 0xF000 | TempData1_gy2[1] << 4 | ( TempData1_gy2[2] >> 4 );
            }
            else
            {
                GyroZ2_tmpr = TempData1_gy2[1] << 4 | ( TempData1_gy2[2] >> 4 );
            }
            groy_7011_T2 = GyroZ2_tmpr / 16.0;

            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, RdRateData_gy3, TempData_gy3, sizeof ( RdRateData_gy3 ) );
            flag_spi1_trx_cs3_dma_start1 = 1;

        }
        else if ( flag_spi1_trx_cs3_dma_start1 &&spi1_trx_Callback_flag)
        {
			      spi1_trx_Callback_flag = 0;
            SPI1_RX_gy_cnt++;
            flag_spi1_rx1_cs3 = 1;
            flag_spi1_trx_cs3_dma_start1 = 0;
            SPI1_CS3_disable;
            if ( TempData_gy3[1] & 0x80 )
            {
                GyroZ3 = 0xFF000000 | TempData_gy3[1] << 16 | TempData_gy3[2] << 8 | TempData_gy3[3];
            }
            else
            {
                GyroZ3 = TempData_gy3[1] << 16 | TempData_gy3[2] << 8 | TempData_gy3[3];
            }
            groy_7011_Z3_old = groy_7011_Z3;
            groy_7011_Z3 = ( GyroZ3 / 71680.0 ) * 4 - groy_7011_Z3_Bias; //0.0568
            groy_7011_Z3_sum    = groy_7011_Z3_sum + groy_7011_Z3 * 0.001;
            groy_7011_Z3_cha = ( groy_7011_Z3 - groy_7011_Z3_old ) * 1000.0;

            groy_7011_Z3_sum_1s = groy_7011_Z3_sum_1s + groy_7011_Z3;
            test_cnt_1s3++;

            if ( test_cnt_1s3 == 1000.0 )
            {
                groy_7011_Z3_sum_1spj = groy_7011_Z3_sum_1s * 0.001;
                groy_7011_Z3_sum_1s = 0;
                test_cnt_1s3 = 0;
                groy_7011_T3_1s = groy_7011_T3;
            }

            SPI1_CS3_enable;
            HAL_SPI_TransmitReceive_DMA ( &hspi1, TempRd_gy3, TempData1_gy3, sizeof ( TempRd_gy3 ) );
            flag_spi1_trx_cs3_dma_start2 = 1;

        }
        else if ( flag_spi1_trx_cs3_dma_start2 &&spi1_trx_Callback_flag)
        {
			spi1_trx_Callback_flag = 0;
            SPI1_RX_t_cnt++;
            flag_spi1_rx2_cs3 = 1;
            flag_spi1_trx_cs3_dma_start2 = 0;
            SPI1_CS3_disable;
            if ( TempData1_gy3[1] & 0x80 )
            {
                GyroZ3_tmpr = 0xF000 | TempData1_gy3[1] << 4 | ( TempData1_gy3[2] >> 4 );
            }
            else
            {
                GyroZ3_tmpr = TempData1_gy3[1] << 4 | ( TempData1_gy3[2] >> 4 );
            }
            groy_7011_T3 = GyroZ3_tmpr / 16.0;
            flag_fwzktx = 1;//

        }
}
