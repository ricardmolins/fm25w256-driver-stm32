/**
 * @file      fm25w256_driver.c
 * @authors   Ricard Molins
 * @copyright Closed
 *
 * @brief Driver for 256-Kbit (32K × 8) Serial (SPI) F-RAM for the STM32F767 HAL
 *
 */

/*******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "fm25w256_driver.h"
#include "stm32f7xx_hal.h"



/*******************************************************************************
* Defines
*******************************************************************************/
#define TX_BUFFER_SIZE 100
#define RX_BUFFER_SIZE 100

#define OPCODE_CMD_WRITE_HEADER_SIZE 3
#define OPCODE_CMD_READ_HEADER_SIZE 3
#define OPCODE_CMD_WRITE_ENABLE_SIZE 1
#define OPCODE_CMD_READ_STATUS_SIZE 2
#define OPCODE_CMD_CODE_INDEX 0
#define OPCODE_CMD_MSB_ADDR 1
#define OPCODE_CMD_LSB_ADDR 2


/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/*******************************************************************************
* Variables
*******************************************************************************/
static FSMFram fsm_fram = FRAM_NO_INIT;
/* GPIO handlers */
static GPIO_TypeDef *cs_gpio_port = 0;
static uint16_t cs_gpio_pin;
/* SPI handlers */
static SPI_HandleTypeDef *fram_spi = 0;
/* Buffers */
static StatusRegister status_register;
static uint8_t tx_buffer[TX_BUFFER_SIZE] = {0};
static uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
static FramFunctions fram_function = FRAM_FUNCTION_NONE;
static uint16_t req_address = 0;
static uint16_t req_length = 0;
static bool fram_busy =false;

static void (*WriteCompleteCbk)(void) = 0;
static void (*ReadCompleteCbk)(void) = 0;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/*******************************************************************************
* Functions
*******************************************************************************/
/**
 * @brief Initializes the FM25W256 Ferromagnetic driver module
 *
 * @pre Module shall have GPIO and SPI ports registered using FM25W256_RegisterCS
 * and FM25W256_RegisterSPI
 */
void FM25W256_Init(void)
{
    if ((cs_gpio_port == 0) || (fram_spi == 0)){
        /* do nothing */
    } else {
        HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_SET);
        status_register.sr_byte = STATUS_REGISTER_NOT_AVAILABLE;
        fsm_fram = FRAM_READY_IDLE;
    }
}

/**
 *  @brief Registers a GPIO to be used as CS
 *
 *  @param gpio_x GPIO port
 *  @param gpio_pin GPIO pin
 */
void FM25W256_RegisterCS(GPIO_TypeDef *gpio_x, uint16_t gpio_pin)
{
    cs_gpio_port = gpio_x;
    cs_gpio_pin = gpio_pin;
}

/**
 *  @brief Registers the SPI that the module should use to communicate
 *
 *  @param spi_handler SPI configuration pointer
 */
void FM25W256_RegisterSPI(SPI_HandleTypeDef* spi_handler)
{
    fram_spi = spi_handler;
}

/**
 *  @brief Registers the Write complete callback
 *
 *  @param fun callback that will be called on completion (within interrupt context)
 */
void FM25W256_RegisterWriteCompleteCbk(void (*fun)())
{
    WriteCompleteCbk = fun;
}

/**
 *  @brief Registers the Read complete callback
 *
 *  @param fun callback that will be called on completion (within interrupt context)
 */
void FM25W256_RegisterReadCompleteCbk(void (*fun)())
{
    ReadCompleteCbk = fun;
}

/**
 *  @brief Get status of drivers
 *
 *  @return FSMFram Current status of the FSM from the driver
 */
FSMFram FM25W256_GetStatus(void)
{
    return fsm_fram;
}

/**
 *  @brief Get status register value.
 *
 *  @pre FM25W256_ReadStatusRegister shall be called before getting the value
 *
 *  @return StatusRegister Last known value of status register
 */
StatusRegister FM25W256_GetStatusRegister(void)
{
    return status_register;
}


/**
 *  @brief Get the values of the last Received memory positions.
 *
 *  @pre FM25W256_ReadMemoryData shall be called with at least a >= data length
 *  before FM25W256_GetMemoryData
 *
 *  @param *data buffer to be filled with data
 *  @param data_length amount of bytes to be copied to "data"
 *
 */
void FM25W256_GetMemoryData(uint8_t * data, uint16_t data_length)
{
    uint16_t counter = 0;

    for(counter = 0 ; counter <data_length; counter++){
        data[counter] = rx_buffer[OPCODE_CMD_READ_HEADER_SIZE + counter];
    }
}

/**
 * @brief FSM of the driver. The drivers stays in idle waiting for requests to be made.
 * Once a request is done no more requests are accepted until the driver is in IDLE state
 * again.
 */
void Task_FM25W256_Driver(void)
{
    uint8_t clock_pulses = 0;

    switch (fsm_fram) {
    case FRAM_NO_INIT:
        /* driver not initialized */
        break;
    case FRAM_READY_IDLE:
        /* Check if there is any request */
        switch (fram_function) {
        case FRAM_FUNCTION_READ_STATUS_REGISTER:
            /* Status register read */
            fsm_fram = FRAM_READ_STATUS;
            break;
        case FRAM_FUNCTION_WRITE_STATUS_REGISTER:
            /* Write status register */
            fsm_fram = FRAM_WRITE_STATUS_REQUEST_WRITE_PERMISSION;
            break;
        case FRAM_FUNCTION_WRITE_REGISTERS:
            /* Write multiple registers */
            fsm_fram = FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION;
            break;
        case FRAM_FUNCTION_READ_REGISTERS:
            /* Read multiple registers*/
            fsm_fram = FRAM_READ_REGISTER;
            break;
        default:
            break;
        }
        fram_function = FRAM_FUNCTION_NONE;
        break;
    case FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION:
        HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_RESET);
        tx_buffer[0] = FRAM_WREN;

        HAL_SPI_TransmitReceive_IT(fram_spi, &tx_buffer[0], &rx_buffer[0], OPCODE_CMD_WRITE_ENABLE_SIZE) ;

        fsm_fram = FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION_WAIT;
        break;
    case FRAM_WRITE_REGISTER_REQUEST_WRITE_PERMISSION_WAIT:
        if (fram_busy == false) {
            fsm_fram = FRAM_WRITE_REGISTER_WRITE;
        } else {
            /* do nothing */
        }
        break;
    case FRAM_WRITE_REGISTER_WRITE:
        HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_RESET);
        tx_buffer[OPCODE_CMD_CODE_INDEX] = FRAM_WRITE;
        tx_buffer[OPCODE_CMD_MSB_ADDR] = ((req_address & FRAM_ADDR_MASK) >> 8) & 0xFF;
        tx_buffer[OPCODE_CMD_LSB_ADDR] = ((req_address & FRAM_ADDR_MASK) >> 0) & 0xFF;
        clock_pulses = req_length + OPCODE_CMD_WRITE_HEADER_SIZE;

        HAL_SPI_TransmitReceive_IT(fram_spi, &tx_buffer[0], &rx_buffer[0], clock_pulses);

        fsm_fram = FRAM_WRITE_REGISTER_WRITE_WAIT;
        break;
    case FRAM_WRITE_REGISTER_WRITE_WAIT:
        if (fram_busy == false) {
            fsm_fram = FRAM_READY_IDLE;
        } else {
            /* do nothing */
        }
        break;
    case FRAM_READ_REGISTER:
        HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_RESET);
        /* Update Tx buffer with OP code and address*/
        tx_buffer[OPCODE_CMD_CODE_INDEX] = FRAM_READ;
        tx_buffer[OPCODE_CMD_MSB_ADDR] = ((req_address & FRAM_ADDR_MASK) >> 8) & 0xFF;
        tx_buffer[OPCODE_CMD_LSB_ADDR] = ((req_address & FRAM_ADDR_MASK) >> 0) & 0xFF;
        clock_pulses = req_length + OPCODE_CMD_READ_HEADER_SIZE;

        HAL_SPI_TransmitReceive_IT(fram_spi, &tx_buffer[0], &rx_buffer[0], clock_pulses);

        fsm_fram = FRAM_READ_REGISTER_WAIT;
        break;
    case FRAM_READ_REGISTER_WAIT:
        if (fram_busy == false) {
            fsm_fram = FRAM_READY_IDLE;
        } else {
            /* do nothing */
        }
        break;
    case FRAM_READ_STATUS:
        HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_RESET);
        tx_buffer[0] = FRAM_RDSR;

        HAL_SPI_TransmitReceive_IT(fram_spi, &tx_buffer[0], &rx_buffer[0], OPCODE_CMD_READ_STATUS_SIZE);

        fsm_fram = FRAM_READ_STATUS_WAIT;
        break;
    case FRAM_READ_STATUS_WAIT:
        if (fram_busy == false) {
            fsm_fram = FRAM_READY_IDLE;
        } else {
            /* do nothing */
        }
        break;

    case FRAM_WRITE_STATUS_REQUEST_WRITE_PERMISSION:
        /* TODO */
        break;

    case FRAM_WRITE_STATUS_WRITE:
        /* TODO */
        break;

    default:
        /* do nothing */
        break;
    }
}


/**
 * @brief Request a status register read to the FRAM
 *
 * @return true if the request was accepted, false otherwise.
 */
bool FM25W256_ReadStatusRegister(void)
{
  bool request_accepted = false;
  if ( fram_busy == true){
      /* do nothing */
      request_accepted = false;
  } else {
      fram_busy = true;
      request_accepted = true;
      fram_function = FRAM_FUNCTION_READ_STATUS_REGISTER ;
  }
  return request_accepted;
}

/**
 * @brief Request a Write of DATA_LENGTH bytes from DATA to the address ADDR from the FRAM
 *
 * @param addr address of the FRAM
 * @param *data buffer containing data
 * @param data_length number of bytes to copy
 *
 * @return true if the request was accepted, false otherwise.
 */
bool FM25W256_WriteMemoryData(uint16_t addr, uint8_t * data, uint16_t data_length)
{
    uint8_t buffer_iter;
    bool request_accepted = false;
    if (fram_busy == true) {
        /* do nothing */
        request_accepted = false;
    } else {
        fram_busy = true;
        request_accepted = true;

        fram_function = FRAM_FUNCTION_WRITE_REGISTERS;
        req_address = addr;
        /* Sanitizing inputs */
        if (req_length > (TX_BUFFER_SIZE)) {
            req_length = TX_BUFFER_SIZE;
        } else {
            req_length = data_length;
        }

        /* copying into transmission buffer */
        for (buffer_iter = 0; buffer_iter < data_length; buffer_iter++) {
            tx_buffer[OPCODE_CMD_WRITE_HEADER_SIZE + buffer_iter] =
                    data[buffer_iter];
        }
    }

    return request_accepted;
}


/**
 * @brief Request a read of DATA_LENGTH bytes starting at address ADDR of the FRAM.
 *
 * @param addr address of the FRAM
 * @param data_length number of bytes to read
 *
 * @return true if the request was accepted, false otherwise.
 */
bool FM25W256_ReadMemoryData(uint16_t addr, uint16_t data_length)
{
    bool request_accepted = false;
    if (fram_busy == true) {
        /* do nothing */
        request_accepted = false;
    } else {
        fram_busy = true;
        request_accepted = true;
        fram_function = FRAM_FUNCTION_READ_REGISTERS;
        req_address = addr;
        req_length = data_length;
    }

    return request_accepted;
}


/**
  * @brief  TxRx Transfer completed callback.
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void FM25W256_SPI_TxRxCpltCallback_IRQ(void)
{
    HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_SET);
    fram_busy = false;


    switch (fsm_fram) {
    case FRAM_READ_STATUS_WAIT:
        /* Store the status register */
        status_register.sr_byte = rx_buffer[1];
        break;
    case FRAM_READ_REGISTER_WAIT:
        /* Call Read register complete callback*/
        if (ReadCompleteCbk != 0) {
            ReadCompleteCbk();
        }
        break;
    case FRAM_WRITE_REGISTER_WRITE_WAIT:
        /* Call Write register complete callback*/
        if (WriteCompleteCbk != 0) {
            WriteCompleteCbk();
        }
        break;
    default:
        break;
    }
}


/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void FM25W256_SPI_ErrorCallback_IRQ(void)
{
    HAL_GPIO_WritePin(cs_gpio_port, cs_gpio_pin, GPIO_PIN_SET);
    fram_busy = false;
}
