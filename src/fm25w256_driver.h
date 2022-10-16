/**
 * @file      fm25w256_driver.h
 * @authors   Ricard Molins
 * @copyright Closed
 *
 * @brief Driver for 256-Kbit (32K × 8) Serial (SPI) F-RAM for the STM32F767 HAL
 *
 */

#ifndef __FM25W256_H_
#define __FM25W256_H_

/*******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

/*******************************************************************************
* Defines
*******************************************************************************/


#define FRAM_WREN  0b00000110 /* Set write enable latch */
#define FRAM_WRDI  0b00000100 /* Write disable */
#define FRAM_RDSR  0b00000101 /* Read Status Register */
#define FRAM_WRSR  0b00000001 /* Write Status Register */
#define FRAM_READ  0b00000011 /* Read memory data */
#define FRAM_WRITE 0b00000010 /* Write memory data */


#define STATUS_REGISTER_NOT_AVAILABLE 0xFF
#define FRAM_ADDR_MASK 0x7FFF


/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/
typedef union
{
  struct
  {
    unsigned char DC_0 : 1; /* Do not care. This bit is non-writable and always returns ‘0’ upon read. */
    unsigned char WEL : 1; /* Write Enable Latch. WEL indicates if the device is write enabled. This bit defaults to ‘0’ (disabled) on power-up.
                                WEL = ‘1’ --> Write enabled
                                WEL = ‘0’ --> Write disabled*/
    unsigned char BP0 : 1;
    unsigned char BP1 : 1;
    unsigned char DC_1 : 1; /* Do not care. These bits are non-writable and always return ‘0’ upon read. */
    unsigned char DC_2 : 1; /* Do not care. These bits are non-writable and always return ‘0’ upon read. */
    unsigned char DC_3 : 1; /* Do not care. These bits are non-writable and always return ‘0’ upon read. */
    unsigned char WPEN : 1; /*  Write Protect Enable bit.  Used to enable the function of Write Protect Pin (WP).*/
  }sr_register;
  unsigned char sr_byte;
}StatusRegister;

typedef enum{
    FRAM_NO_INIT,
    FRAM_READY_IDLE,
    FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION,
    FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION_WAIT,
    FRAM_WRITE_REGISTER_WRITE,
    FRAM_WRITE_REGISTER_WRITE_WAIT,
    FRAM_READ_REGISTER,
    FRAM_READ_REGISTER_WAIT,
    FRAM_READ_STATUS,
    FRAM_READ_STATUS_WAIT,
    FRAM_WRITE_STATUS_REQUEST_WRITE_PERMISSION,
    FRAM_WRITE_STATUS_WRITE
} FSMFram;

typedef enum {
    FRAM_FUNCTION_NONE = 0,
    FRAM_FUNCTION_READ_STATUS_REGISTER,
    FRAM_FUNCTION_WRITE_STATUS_REGISTER,
    FRAM_FUNCTION_WRITE_REGISTERS,
    FRAM_FUNCTION_READ_REGISTERS
}FramFunctions;

/*******************************************************************************
* Variables
*******************************************************************************/

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void FM25W256_Init(void);
void FM25W256_RegisterCS(GPIO_TypeDef *gpio_x, uint16_t gpio_pin);
void FM25W256_RegisterSPI(SPI_HandleTypeDef* spi_handler);
void FM25W256_RegisterWriteCompleteCbk(void (*fun)());
void FM25W256_RegisterReadCompleteCbk(void (*fun)());

FSMFram FM25W256_GetStatus(void);
StatusRegister FM25W256_GetStatusRegister(void);
void FM25W256_GetMemoryData(uint8_t * data, uint16_t data_length);

void Task_FM25W256_Driver(void);
void FM25W256_SPI_ErrorCallback_IRQ(void);
void FM25W256_SPI_TxRxCpltCallback_IRQ(void);

bool FM25W256_ReadStatusRegister(void);
bool FM25W256_WriteMemoryData(uint16_t addr, uint8_t * data, uint16_t data_length);
bool FM25W256_ReadMemoryData(uint16_t addr, uint16_t data_length);
/*******************************************************************************
* Functions
*******************************************************************************/


#endif /* __FM25W256_H_ */
