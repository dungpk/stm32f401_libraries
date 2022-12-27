/*
 * smt32f401xx_i2c_driver.c
 *
 *  Created on: Dec 27, 2022
 *      Author: Dungpk
 */


#include "stm32f401xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPLLOutputClock(void);

/**********************************************************************************
 * @fn           - I2C_PeriClockControl
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - value input pin of gpio
 *
 * @Note         - None
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2C,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2C == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2C == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2C == I2C3)
		{
			I2C3_PCLK_EN();
		}

	}else
	{
		if(pI2C == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2C == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2C == I2C3)
		{
			I2C3_PCLK_DI();
		}

	}
}


/**********************************************************************************
 * @fn           - I2C_Init
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - value input pin of gpio
 *
 * @Note         - None
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	//1. ACK control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;

	//2.Configure the FREG field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	//3. Program the Device own address
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= (1<<14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//4. CCR caculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GetPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCL_Speed));
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle	<< 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCL_Speed));
		}else
		{
			ccr_value = (RCC_GetPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCL_Speed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;
}

/**********************************************************************************
 * @fn           - RCC_GetPCLK1Value
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - value input pin of gpio
 *
 * @Note         - None
 *
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;
	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = (RCC->CFGR >>2 )& 0x03;

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//ahb1
	temp = ( (RCC->CFGR >> 10) & 0x07);
	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[8-temp];
	}

	// apb1

	temp = ( (RCC->CFGR >> 10) & 0x07);
		if(temp < 4)
		{
			apb1p = 1;
		}else
		{
			apb1p = APB_PreScaler[temp-4];
		}

		pclk1 = (SystemClk/ahbp)/apb1p;

	return pclk1;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t value = 0;

	return value;
}
