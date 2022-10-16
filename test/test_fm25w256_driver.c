#include "unity.h"
#include <stdint.h>

#include "fm25w256_driver.h"

#include "mock_stm32f7xx_hal.h"


GPIO_TypeDef cs_gpio_port = {0};
SPI_HandleTypeDef fram_spi = {0};
uint16_t cs_gpio_pin = 0;

void setUp(void)
{
    HAL_GPIO_WritePin_Ignore();
    FM25W256_RegisterCS(&cs_gpio_port, cs_gpio_pin);
    FM25W256_RegisterSPI(&fram_spi);
    FM25W256_Init();
    HAL_GPIO_WritePin_StopIgnore();
}

void tearDown(void)
{

}

void test_ReadStatusRegister(void)
{

    /* Expected message to be send */
    uint8_t expected_spi_tx[1] = {0b00000101};
    uint8_t expected_spi_rx[1] = {0};
    uint8_t expected_spi_message_length = 2;

    /* Chip select goes to 0*/
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 0);

    /* We expect to transmit a message of length 2 (the status register + the status received */
    HAL_SPI_TransmitReceive_IT_ExpectWithArrayAndReturn(&fram_spi,1,&expected_spi_tx[0],1,&expected_spi_rx[0], 1,expected_spi_message_length,0);
    HAL_SPI_TransmitReceive_IT_ReturnArrayThruPtr_pRxData(&expected_spi_rx[0],1);

    /* After transmission the chip select goes to 1 again */
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 1);

    /* Making the actual call*/
    FM25W256_ReadStatusRegister();

    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    /* Faking the interrupt */
    FM25W256_SPI_TxRxCpltCallback_IRQ();
    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();

}

void test_Read_5_registers_at_address_0x3344(void)
{

    /* Expected message to be send */
    uint8_t expected_spi_tx[3] = {0b00000011, 0x33,0x44};
    uint8_t expected_spi_rx[1] = {0};
    uint8_t expected_spi_message_length = 5 + 3; /* data_length + heater size */

    uint16_t start_addres = 0x3344;
    uint16_t data_length = 5;

    /* Chip select goes to 0*/
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 0);

    /* We expect to transmit a message of length 2 (the status register + the status received */
    HAL_SPI_TransmitReceive_IT_ExpectWithArrayAndReturn(&fram_spi,1,&expected_spi_tx[0],3,&expected_spi_rx[0], 1,expected_spi_message_length,0);
    HAL_SPI_TransmitReceive_IT_ReturnArrayThruPtr_pRxData(&expected_spi_rx[0],1);

    /* After transmission the chip select goes to 1 again */
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 1);

    /* Making the actual call*/
    FM25W256_ReadMemoryData(start_addres,data_length);

    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    /* Faking the interrupt */
    FM25W256_SPI_TxRxCpltCallback_IRQ();
    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();

}

void test_Write_3_registers_at_address_0x1278(void)
{
    /* This tests expects two messages,
     * 1st message is a "write enable"
     * 2nd message is the "write itself"
     */
    /* Expected message 1 to be send */
    uint8_t expected_spi_tx[1] = {0b00000110};
    uint8_t expected_spi_rx[1] = {0};
    uint8_t expected_spi_message_length = 1; /* data_length + heater size */

    /* Expected message 1 to be send */
    uint8_t expected_spi_tx_2[6] = {0b00000010,0x12,0x78,0x2,0x6,0x7};
    uint8_t expected_spi_rx_2[1] = {0};
    uint8_t expected_spi_message_length_2 = 3 + 3; /* data_length + heater size */

    uint16_t start_addres = 0x1278;
    uint8_t data_to_send[3] = {2,6,7};
    uint16_t data_length = 3;

    /* Chip select goes to 0*/
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 0);

    /* We expect to transmit a message of length 2 (the status register + the status received */
    HAL_SPI_TransmitReceive_IT_ExpectWithArrayAndReturn(&fram_spi,1,&expected_spi_tx[0],1,&expected_spi_rx[0], 1,expected_spi_message_length,0);
    HAL_SPI_TransmitReceive_IT_ReturnArrayThruPtr_pRxData(&expected_spi_rx[0],1);

    /* After transmission the chip select goes to 1 again */
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 1);


    /* Chip select goes to 0*/
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 0);

    /* We expect to transmit a message of length 2 (the status register + the status received */
    HAL_SPI_TransmitReceive_IT_ExpectWithArrayAndReturn(&fram_spi,1,&expected_spi_tx_2[0],6,&expected_spi_rx_2[0], 1,expected_spi_message_length_2,0);
    HAL_SPI_TransmitReceive_IT_ReturnArrayThruPtr_pRxData(&expected_spi_rx[0],1);

    /* After transmission the chip select goes to 1 again */
    HAL_GPIO_WritePin_Expect(&cs_gpio_port, cs_gpio_pin, 1);

    /* Making the actual call*/
    FM25W256_WriteMemoryData(start_addres, &data_to_send[0], data_length);

    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    /* Faking the interrupt MSG 1*/
    FM25W256_SPI_TxRxCpltCallback_IRQ();
    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
    /* Faking the interrupt MSG 2*/
    FM25W256_SPI_TxRxCpltCallback_IRQ();
    /* Faking some time */
    Task_FM25W256_Driver();
    Task_FM25W256_Driver();
}




