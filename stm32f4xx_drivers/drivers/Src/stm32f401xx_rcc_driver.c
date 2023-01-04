/*
 * stm32f401xx_rcc_driver.c
 *
 *  Created on: Jan 3, 2023
 *      Author: Dungpk
 */

#include "stm32f401xx_rcc_driver.h"


uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = {2,4,8,16};

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

/**********************************************************************************
 * @fn           - RCC_GetPCLK2Value
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
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2,SystemClk;
	uint8_t clksrc,temp,ahbp,apb2p;

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
			apb2p = 1;
		}else
		{
			apb2p = APB_PreScaler[temp-4];
		}

		pclk2 = (SystemClk/ahbp)/apb2p;

	return pclk2;
}

uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t value = 0;

	return value;
}
