#ifndef __BSP_STATUS_H
#define __BSP_STATUS_H

typedef enum
{
    /* Generic error codes */
	/*!< Generic operation success status */
    STATUS_SUCCESS                         = 0x000U,    
	/*!< Generic operation failure status */
    STATUS_ERROR                           = 0x001U,    
	/*!< Generic operation busy status */
    STATUS_BUSY                            = 0x002U,    
	/*!< Generic operation timeout status */
    STATUS_TIMEOUT                         = 0x003U,    
	/*!< Generic operation unsupported status */
    STATUS_UNSUPPORTED                     = 0x004U,   
	
    /* I2C specific error codes */
	/*!< NACK signal received  */
    STATUS_I2C_RECEIVED_NACK               = 0x200U,  
	/*!< TX underrun error */
    STATUS_I2C_TX_UNDERRUN                 = 0x201U,  
	/*!< RX overrun error */
    STATUS_I2C_RX_OVERRUN                  = 0x202U,  
	/*!< Arbitration lost */
    STATUS_I2C_ARBITRATION_LOST            = 0x203U,  
	/*!< A transfer was aborted */
    STATUS_I2C_ABORTED                     = 0x204U,  
	/*!< I2C bus is busy, cannot start transfer */
    STATUS_I2C_BUS_BUSY                    = 0x205U,  
	
    /* CAN specific error codes */
	/*!< The specified MB index is out of the configurable range */
    STATUS_CAN_BUFF_OUT_OF_RANGE           = 0x300U,  
	/*!< There is no transmission or reception in progress */
    STATUS_CAN_NO_TRANSFER_IN_PROGRESS     = 0x301U,  
	
    /* SPI specific error codes */
	/*!< TX underrun error */
    STATUS_SPI_TX_UNDERRUN                 = 0x500U, 
	/*!< RX overrun error */	
    STATUS_SPI_RX_OVERRUN                  = 0x501U, 
	/*!< A transfer was aborted */
    STATUS_SPI_ABORTED                     = 0x502U,  
	
    /* UART specific error codes */
	/*!< TX underrun error */
    STATUS_UART_TX_UNDERRUN                = 0x600U,  
	/*!< RX overrun error */
    STATUS_UART_RX_OVERRUN                 = 0x601U,  
	/*!< A transfer was aborted */
    STATUS_UART_ABORTED                    = 0x602U,  
	/*!< Framing error */
	STATUS_UART_FRAMING_ERROR              = 0x603U,  
	/*!< Parity error */
	STATUS_UART_PARITY_ERROR               = 0x604U, 
	/*!< Noise error */
	STATUS_UART_NOISE_ERROR                = 0x605U,  
	
    /* FLASH specific error codes */
    STATUS_FLASH_OK                        = 0x900U,
	/*!< It's impossible to enable an operation */
    STATUS_FLASH_ERROR_ENABLE              = 0x901U, 
	/*!< No blocks have been enabled for Array Integrity check */
    STATUS_FLASH_ERROR_NO_BLOCK            = 0x902U, 
	/*!< InProgress status */
    STATUS_FLASH_INPROGRESS                = 0x903U, 
    /*解锁失败*/
    STATUS_FLASH_ERR_UNLOCK                = 0x904U,
    /*擦除flash错误*/
    STATUS_FLASH_ERR_ERASE                 = 0x905U,
    /*写错误*/
    STATUS_FLASH_ERR_WRITE                 = 0x906U,
    /*无效的地址*/
    STATUS_FLASH_ERR_INV_ADDR              = 0x907U,
	
    /* ENET specific error codes */
	/*!< There is no available frame in the receive queue */
    STATUS_ENET_RX_QUEUE_EMPTY             = 0xA01U, 
	/*!< There is no available space for the frame in the transmit queue */
    STATUS_ENET_TX_QUEUE_FULL              = 0xA02U, 
	/*!< The specified buffer was not found in the queue */
    STATUS_ENET_BUFF_NOT_FOUND             = 0xA03U, 

} status_t;

#endif /*__BSP_STATUS_H*/
