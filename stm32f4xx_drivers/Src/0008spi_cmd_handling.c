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


//command codes
#define COMMAND_LED_CTRL           0x50
#define COMMAND_SENSOR_READ         0x51
#define COMMAND_LED_READ           0x52
#define COMMAND_PRINT              0x53
#define COMMAND_ID_READ            0x54

// led state
#define LED_ON                     1
#define LED_OFF                    0


//arduino analog pin
#define ANALOG_PIN_0			   0
#define ANALOG_PIN_1			   1
#define ANALOG_PIN_2			   2
#define ANALOG_PIN_3			   3
#define ANALOG_PIN_4			   4


// arduino led
#define LED_PIN                    9

void GPIO_ButtonInit(void);
void SPI_GPIOInit(void);
void SPI2_Init(void);
void delay(void);
uint8_t SPI_VerifyResponse(uint8_t ackByte);

int main(void)
{
	char user_data[] = "Hello World";

	uint8_t dummy_read = 0xFF;
	uint8_t dummy_write;
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI_GPIOInit();

	// This function is used to initialize the SPI2 peripheral parameter
	SPI2_Init();


	/*
	 * making SSOE 1 does NSS output enable
	 * The NSS Pin is automatically managed by hardware.
	 * i.e When SPE = 1, NSS pulled to Low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	// enable spi peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	// first send length information
	uint8_t dataLength = strlen(user_data);
	SPI_SendData(SPI2,&dataLength,1);

	// Lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	// Lets confirm SPI is not busy
	while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

	// Disable spi peripheral
	SPI_PeripheralControl(SPI2,DISABLE);

	while(1)
	{
		//wait button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouning relate issuse 200ms delay
		delay();

		// enable the SPI2 Peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTR
		uint8_t commandCode = COMMAND_LED_CTRL;
		uint8_t ackByte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandCode,1);

		//send some dummy bits (1 byte) fetch the respone from the slave
		SPI_SendData(SPI2, &dummy_read,1);

		//send some dummy bits (1 byte) fetch the respone from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve Data
		SPI_ReceiveData(SPI2, &ackByte, 1);

		// verify response
		if (SPI_VerifyResponse(ackByte))
		{
			//send arguments
			args[0] = ANALOG_PIN_0;
			SPI_SendData(SPI2,args,1);
		}

		//enable of COMMAND_LED_CONTROL

		//2. CMD_SENSOR_READ < analog pin number(1) >

		//wait till button is pressed

		while(! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

		// to avoid button de-boucing related issuse 200ms of delay
		delay();

		commandCode = COMMAND_SENSOR_READ;

		//send command read to clear off the RXNE
		SPI_SendData(SPI2,&commandCode,1);

		//insert some delay so that slave can ready with the data
		delay();

		//send some dummy bits (1 byte) fetch the respone from the slave
		SPI_SendData(SPI2, &dummy_read,1);

		//send some dummy bits (1 byte) fetch the respone from the slave
		SPI_SendData(SPI2, &dummy_write, 1);

		// Recieve Data
		SPI_ReceiveData(SPI2, &ackByte, 1);

		// verify response
		if (SPI_VerifyResponse(ackByte))
		{
			//send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2,args,2);
		}

		// Lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG));

		// Disable spi peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}
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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generates sclk of 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardward slave management enabled for NSS Pin

	SPI_Init(&SPI2handle);
}

void GPIO_ButtonInit(void)
{
	// this is gpio button init
	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GPIOBtn);
}

void delay(void)
{
	for(uint32_t i=0;i<50000/2;i++);
}

uint8_t SPI_VerifyResponse(uint8_t ackByte)
{
	if(ackByte == 0xF5)
	{
		return 1;
	}
	return 0;
}
