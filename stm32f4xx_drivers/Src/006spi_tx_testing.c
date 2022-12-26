/*
 * 006spi_tx_testing.c
 *
 *  Created on: Dec 23, 2022
 *      Author: Dungpk
 */


/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB23 --> SP2_CLK
 * PB12 --> SP2_NSS
 * ALT function mode : 5
 */

#include "stdint.h"
#include "string.h"
#include "stm32f401xx_gpio_drivers.h"
#include "stm32f401xx_spi_driver.h"

void SPI_GPIOInit(void);
void SPI2_Init(void);

int main(void)
{
	char user_data[] = "Hello World";
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInit();

	// This function is used to initialize the SPI2 peripheral parameter
	SPI2_Init();

	// enable spi peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	SPI_SSIConfig(SPI2,ENABLE);

	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	// Lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	// Disable spi peripheral
	SPI_PeripheralControl(SPI2,DISABLE);


	while(1);
	return 0;
}

void SPI_GPIOInit(void)
{

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


}

void SPI2_Init(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_Busconfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // software slave management enabled for NSS Pin

	SPI_Init(&SPI2handle);
}
