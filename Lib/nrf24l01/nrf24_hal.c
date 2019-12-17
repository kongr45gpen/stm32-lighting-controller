#include "nrf24_hal.h"


// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void)
{
    // No need to initialise SPI, as this is handled by STM32CubeMX
    nRF24_CE_L();
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
/*uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_TXE) == RESET);
	// Send byte to SPI (TXE cleared)
	SPI_I2S_SendData(nRF24_SPI_PORT, data);
	// Wait while receive buffer is empty
	while (SPI_I2S_GetFlagStatus(nRF24_SPI_PORT, SPI_I2S_FLAG_RXNE) == RESET);

	// Return received byte
	return (uint8_t)SPI_I2S_ReceiveData(nRF24_SPI_PORT);
}*/

uint8_t nRF24_LL_RW(uint8_t data)
{
    static uint8_t receiveBuffer;

    HAL_SPI_TransmitReceive(&hspi3, &data, &receiveBuffer, 1, 0);
}
