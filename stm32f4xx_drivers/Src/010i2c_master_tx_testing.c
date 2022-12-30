/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Dec 28, 2022
 *      Author: Dungpk
 */

#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_gpio_drivers.h"
#include <string.h>
#include <stdio.h>


#define MY_ADD      0x61
#define SLAVE_ADD   0x68

// some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

I2C_Handle_t I2C1Handle;

void delay(void);
void I2C1_Init(void);
void I2C1_GPIOInit(void);
void GPIO_ButtonInit(void);

int main()
{
	GPIO_ButtonInit();

	I2C1_GPIOInit();

	I2C1_Init();

	// enable the clock peripheral j
	I2C_PeriClockControl(I2C1,ENABLE);



	while(1)
	{
	//wait button is pressed
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		//to avoid button de-bouning relate issuse 200ms delay
		delay();

		//send some data to slave
		I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADD);
	}
	return 0;
}

void delay(void)
{
	for(uint32_t i=0;i<50000/2;i++);
}

/*
 * PB6->SCL
 * PB9->SDA
 */
void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Init(void)
{


	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADD;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCL_Speed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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
