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

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRFLAG(I2C_Handle_t *pI2CHandle);
void I2C_MasterHandleTXEInterupt(I2C_Handle_t *pI2CHandle);
void I2C_MasterHandleRXNEInterupt(I2C_Handle_t *pI2CHandle);


static void I2C_ClearADDRFLAG(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	// check the device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		// device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//Clear the ADDR flag, read SR1, read SR2.
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else
		{
			//Clear the ADDR flag, read SR1, read SR2.
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}else
	{
		// device is in slave mode
		//Clear the ADDR flag, read SR1, read SR2.
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

/**********************************************************************************
 * @fn           - I2C_ExecuteAddressPhase
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
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}


/**********************************************************************************
 * @fn           - I2C_ExecuteAddressPhaseRead
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
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1 ;
	pI2Cx->DR = SlaveAddr;
}

/**********************************************************************************
 * @fn           - I2C_GenerateStartCondition
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
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/**********************************************************************************
 * @fn           - I2C_GenerateStopCondition
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
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

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
	// enable the clock for the i2cxx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

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

	//Trise Configuration
	if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;
	}else
	{
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Value() *300)/1000000000U) +1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
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

/**********************************************************************************
 * @fn           - I2C_MasterSendData
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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate  the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	///2. Confirm the start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled by LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the Slave with r/w bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. Clear the ADDR Flag according to its software squence
	// Note: Until ADDR is cleared SCL will be stretched (pulled by LOW)
	I2C_ClearADDRFLAG(pI2CHandle);

	//6. Send the data until len becomes 0

	while(Len > 0 )
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); // wait till TXE set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. When becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// Note: TXE = 1 and BTF = 1 that both SR and DR are empty and next tranmission should begin

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need to wait for the completion of stop condition
	if(Sr == I2C_DISABLE_SR)
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}

/**********************************************************************************
 * @fn           - I2C_MasterReceiveData
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
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flags in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));
	//3. Send the address of slave with r/w bit set to R(1) (total 8 bit)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,SlaveAddr);

	//procedure to read  only 1 byte from slave

	if(Len == 1)
	{
		// Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

		// Clear the ADDR flag
		I2C_ClearADDRFLAG(pI2CHandle);

		// wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// Generate Stop Condition
		if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// read Data in to buffer
		*pTxbuffer = pI2CHandle->pI2Cx->DR;

	}

	//procedure to read data from slave when Len > 1.
	if(Len > 1)
	{
		// clear the ADDR flag
		I2C_ClearADDRFLAG(pI2CHandle);

		//read Data until Len becomes zero
		for(uint32_t i = Len;i>0;i--)
		{
			// wait until RXNE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) // if last 2 bytes are remaining
			{
				// clear ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_DISABLE);

				// stop bit condition
				if(Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read data from data register in to the buffer
			*pTxbuffer = pI2CHandle->pI2Cx->DR;

			//incement the buffer address
			pTxbuffer++;
		}
	}

	//re- enable acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,I2C_ACK_ENABLE);
	}
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		//enable the Ack
		pI2Cx->CR1 |=  (1 << I2C_CR1_ACK);
	}else
	{
		//disable the ack
		pI2Cx->CR1 &=  ~(1 << I2C_CR1_ACK);
	}
}

void I2C_IRQIneruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	// firsts lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= ( IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

void I2C_MasterHandleTXEInterupt(I2C_Handle_t *pI2CHandle)
{
	// BTF, TXE = 1

	if(pI2CHandle->TxLen == 0)
	{
		//Generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Reset all the member elements of the handle structure.
		I2CCloseSendData(pI2CHandle);

		// Tranmistion complete
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_TX_CMPLT);
	}
}
void I2C_MasterHandleRXNEInterupt(I2C_Handle_t *pI2CHandle)
{
	// We have to do data reception
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxSize == 2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		// close the I2C dara reception and notify the application


		//1. generate the stop condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//2. close the i2c rx
		I2CCloseReceiveData(pI2CHandle);

		// notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interupt handling for both master and slave mode of a device

	uint32_t temp1,temp2,temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN );
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN);
 	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB );

	//1. Handle for interupt generated by SB event
 	if(temp1 && temp3)
 	{
 		// SB flag is set
 		// the block will be not executed in slave mode beacause for slave SB is always zero
 		// In this block lets executed the address phase
 		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
 		{
 			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
 		}else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
 			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx,pI2CHandle->DevAddr);
		}

 	}

	//2. Handle for interupt generate by ADDR event
 	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR );
 	if(temp1 && temp3)
 	{
 		// ADDR flag is set
 		I2C_ClearADDRFLAG(pI2CHandle);
 	}

	//3. Handle interupt generate by BTF (byte transfer finish) event
 	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF );
 	if(temp1 && temp3)
 	{
 		// BTS flag is set
 		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
 		{
 			// make sure that TXE is also set
 			if(pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE ))
 			{
 				I2C_MasterHandleTXEInterupt(pI2CHandle);
 			}

 		}else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
 			;
		}
 	}

	//4. handle interupt generate by Stopf event
 	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF );
 	if(temp1 && temp3)
 	{
 		// STOP flag is set

 		// Clear the Stop flag (i.e read SR1 2) Write to CR1)
 		 pI2CHandle->pI2Cx->CR1 |= 0x0000;

 		 // Notify the applocation that Stop is generated by the master
 		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
 	}

	//5.Handle interupt generate bt TXE event
 	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TXE );
 	if(temp1 && temp3 && temp2)
 	{
 		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
 		{
			// TXE flag is set

			//We have to do the data tranmision
			if(pI2CHandle->TxRxState == I2C_BUSY_TX)
			{
				if(pI2CHandle->TxLen > 0)
				{
					// Load data in to DR
					pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

					// Decrement the TxLen
					pI2CHandle->TxLen--;

					//Increment the buffer address
					pI2CHandle->pTxBuffer++;
				}
			}
 		}
 	}

	//6. Handle for interupt generated by RXNE event
	temp3 =  pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RXNE );
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		if(temp1 && temp3 && temp2)
		{
			// RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_RX)
			{
				I2C_MasterHandleRXNEInterupt(pI2CHandle);
			}

		}
	}

}


void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2CCloseSendData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2CCloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag

		//Implement the code to notify the application about the error

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag

		//Implement the code to notify the application about the error
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
	}

}

