/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: Dec 22, 2022
 *      Author: Dungpk
 */

#include "stm32f401xx_spi_driver.h"


static void spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**********************************************************************************
 * @fn           - SPI_PeriClockControl
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
void SPI_PeriClockControl(SPI_RegDef_t *pSPI,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_EN();
		}
//		else if(pSPI == SPI4)
//		{
//			SPI4_PCLK_EN();
//		}
	}else
	{
		if(pSPI == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPI == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPI == SPI3)
		{
			SPI3_PCLK_DI();
		}
//		else if(pSPI == SPI4)
//		{
//			SPI4_PCLK_DI();
//		}
	}
}

/*
 *  Init and DeInit
 */

/**********************************************************************************
 * @fn           - SPI_Init
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// first lets configue the SPI_CR1 register

	uint32_t tempreg = 0;

	//1.Configure the device mode

	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the Bus Configure
	if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_Busconfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RX must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	//3. Configure spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;
	//5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;


	pSPIHandle->pSPIx->CR1 = tempreg;
}

/**********************************************************************************
 * @fn           - SPI_DeInit
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
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/**********************************************************************************
 * @fn           - SPI_GetFlagStatus
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
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 *  Data Send or Recive
 */

/**********************************************************************************
 * @fn           - SPI_SendData
 *
 * @brief        -
 *
 * @param        -
 * @param        -
 * @param        -
 *
 * @return       - value input pin of gpio
 *
 * @Note         - This is blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG == FLAG_RESET));

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			// Load data in to the DR
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/**********************************************************************************
 * @fn           - SPI_ReceiveData
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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXE_FLAG == FLAG_RESET));

		//2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			// Load data from the DR to Rx buffer
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

/**********************************************************************************
 * @fn           - SPI_SendDataIT
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
 *+
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2. Mark the SPI state as busy in tranmission so that
		// no other code can take over same SPI peripheral until transmission over
		pSPIHandle->TxLen = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interupt whenever TXE flags is set  in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);

		//4. Data Transmission will be handled by the ISR code
	}
	return state;
}

/**********************************************************************************
 * @fn           - SPI_ReceiveDataIT
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
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		//2. Mark the SPI state as busy in tranmission so that
		// no other code can take over same SPI peripheral until transmission over
		pSPIHandle->RxLen = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interupt whenever TXE flags is set  in SR
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);

		//4. Data Transmission will be handled by the ISR code
	}
	return state;
}

/**********************************************************************************
 * @fn           - SPI_PeripheralControl
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
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else

	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/**********************************************************************************
 * @fn           - SPI_SSIConfig
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
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else

	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/**********************************************************************************
 * @fn           - SPI_SSOEConfig
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
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else

	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 *  IRQ Congiguration and ISR handling
 */

/**********************************************************************************
 * @fn           - SPI_IRQIneruptConfig
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
void SPI_IRQIneruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @fn           - SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// firsts lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/**********************************************************************************
 * @fn           - SPI_IRQHandling
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
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1,temp2;
	// first check for TXE

	temp1 = pHandle-> pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 =  pHandle-> pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interupt_handle(pHandle);
	}

	// sencond check for RXNE

	temp1 = pHandle-> pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 =  pHandle-> pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interupt_handle(pHandle);
	}

	//check ovr flag
	temp1 = pHandle-> pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 =  pHandle-> pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}

}


/*
 * some helper function implementations
 */
static void spi_txe_interupt_handle(SPI_Handle_t *pSPIHandle)
{

	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		// Load data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)(pSPIHandle->pTxBuffer));
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;
	}
	else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		// Tx Len is zero, so close the spi communication and inform the application that
		//Tx is over

		//This prevent interupts from setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxne_interupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		// Load data in to the DR
		*((uint16_t *)(pSPIHandle->pRxBuffer)) = pSPIHandle->pSPIx->DR ;
		pSPIHandle->RxLen -=2;
		(uint16_t *)pSPIHandle->pRxBuffer--;
	}
	else
	{
		//8 bit DFF
		 *(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer--;
	}

	if(! pSPIHandle->RxLen)
	{
		// Tx Len is zero, so close the spi communication and inform the application that
		//Tx is over

		//This prevent interupts from setting up of TXE flag
		SPI_CloseReception(pSPIHandle);

		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen  = 0;
	pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen  = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	// this is a weak implemention. The application may override this function.

}
