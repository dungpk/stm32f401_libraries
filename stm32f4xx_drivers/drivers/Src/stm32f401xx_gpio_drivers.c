/*
 * stm32f401xx_gpio_drivers.c
 *
 *  Created on: Dec 19, 2022
 *      Author: Dungpk
 */

#include "stm32f401xx_gpio_drivers.h"

/*
 * Peripheral Clock setup
 */

/**********************************************************************************
 * @fn           - GPIO_PeriClockControl
 *
 * @brief        - This function enable or disable peripheral clock for given GPIO port
 *
 * @param        - Base address of the gpio peripheral
 * @param        - Enable or Disable macros
 * @param        -
 *
 * @return       - None
 *
 * @Note         - None
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Imit and DeInit
 */

/**********************************************************************************
 * @fn           - GPIO_Init
 *
 * @brief        - GPIO Initial
 *
 * @param        - Base address of the gpio peripheral
 * @param        -
 * @param        -
 *
 * @return       - None
 *
 * @Note         - None
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)

{
	// Enable the peripheral Clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp=0;
	// 1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//non interupt mode '
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//interupt mode

		//1.config interupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// config the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// config the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// config both the FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Clear the corresponding RTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

			//2.configure the GPIO port selection  in SYSCFG_EXTICR

			uint8_t temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
			uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (4*temp2) ;

			//3.enable the exti interupt delivery using IMR
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. configure the speed
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	// 3. configure the pupd settings
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	// 4. configure output type
	temp = 0;
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. configure alt functionality
	temp = 0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// config alt function register
		uint32_t temp1,temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/8;
		temp2 =  (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= (0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/**********************************************************************************
 * @fn           - GPIO_DeInit
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - None
 *
 * @Note         - None
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/*
 * Data Read and Write
 */

/**********************************************************************************
 * @fn           - GPIO_ReadFromInputPin
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

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/**********************************************************************************
 * @fn           - GPIO_ReadFromInputPort
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - value input port
 *
 * @Note         - None
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;

	return value;
}

/**********************************************************************************
 * @fn           - GPIO_WriteToOutputPin
 *
 * @brief        - write value of gpio pin
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/**********************************************************************************
 * @fn           - GPIO_WriteToOutputPort
 *
 * @brief        - write value of gpio port
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/**********************************************************************************
 * @fn           - GPIO_ToggleOutputPin
 *
 * @brief        - Toggle output Pin
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^=  ( 1<< PinNumber);
}

/*
 * IRQ Configuration and Handling ISR
 */

/**********************************************************************************
 * @fn           - GPIO_IRQIneruptConfig
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */
void GPIO_IRQIneruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber <64 ) // 32 to 63
		{
			//program ISE1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
		{
			//program ISE2 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}else
	{
		if(IRQNumber <= 31)
				{
					//program ICER0 register
					*NVIC_ICER0 |= (1 << IRQNumber);

				}else if(IRQNumber > 31 && IRQNumber <64 ) // 32 to 63
				{
					//program ISE1 register
					*NVIC_ICER1 |= (1 << (IRQNumber % 32));

				}else if(IRQNumber >= 64 && IRQNumber < 96) //64 to 95
				{
					//program ISE2 register
					*NVIC_ICER3 |= (1 << (IRQNumber % 64));
				}
	}
}


/**********************************************************************************
 * @fn           - GPIO_IRQPriorityConfig
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// firsts lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}
/**********************************************************************************
 * @fn           - GPIO_IRQHandling
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       -
 *
 * @Note         - None
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the exti pr register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//clear
		EXTI->PR |= ( 1<< PinNumber);
	}
}
