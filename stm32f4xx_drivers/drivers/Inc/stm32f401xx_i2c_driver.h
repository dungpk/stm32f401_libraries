/*
 * stm32f402xx_i2c_driver.h
 *
 *  Created on: Dec 27, 2022
 *      Author: Dungpk
 */
#include "stm32f401xx.h"

#ifndef INC_STM32F402XX_I2C_DRIVER_H_
#define INC_STM32F402XX_I2C_DRIVER_H_

/*
 * Configuration structure for I2Cx Peripheral
 */
typedef struct
{
	uint32_t I2C_SCL_Speed; //@I2C_SCLSpeed
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_ACKControl; // @I2C_ACKControl
	uint16_t I2C_FMDutyCycle;  // @IC2_DutyCylce

}I2C_Config_t;


/*
 * Handle structure for I2Cx Peripheral
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t *pTxBuffer; /*< to store the app. Tx buffer address*/
	uint8_t *pRxBuffer; /*< to store the app. Rx buffer address*/
	uint32_t TxLen;     /*< to store Tx Length*/
	uint32_t RxLen;     /*< to store Rx Length*/
	uint8_t TxRxState;  /*< to store communication state*/
	uint8_t DevAddr;   /*< to store slave device address*/
	uint32_t RxSize;   /*< to store Rx size*/
	uint8_t Sr;        /*< to store Rx start value*/

}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM        100000
#define I2C_SCL_SPEED_FM4K      400000
#define I2C_SCL_SPEED_FM2K      200000


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE           1
#define I2C_ACK_DISABLE          0

/*
 * @IC2_DutyCylce
 */
#define I2C_FM_DUTY_2             0
#define I2C_FM_DUTY_16_9          1


/*
 * I2C related status flags definiton
 */
#define I2C_FLAG_SB                     (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR                   (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF                    (1 << I2C_SR1_BTF )
#define I2C_FLAG_ADDR                   (1 << I2C_SR1_ADDR)
#define I2C_FLAG_ADD10                  (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF                  (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE                   (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE                    (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR                   (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO                   (1 << I2C_SR1_ARLO)
#define I2C_FLAG_OVR                    (1 << I2C_SR1_OVR)


/*
 * I2C SR status definiton
 */
#define I2C_DISABLE_SR           RESET
#define I2C_ENABLE_SR            SET


/*
 * I2C application state
 */
#define I2C_READY                0
#define I2C_BUSY_TX              1
#define I2C_BUSY_RX              2

/*
 * I2C application event
 */
#define I2C_EV_TX_CMPLT          0
#define I2C_EV_RX_CMPLT          1
#define I2C_EV_STOP              2

/*
 * I2C error event
 */
#define I2C_ERROR_BERR            3
#define I2C_ERROR_ARLO            4
#define I2C_ERROR_AF              5
#define I2C_ERROR_OVR             6
#define I2C_ERROR_TIMEOUT         7

/************************************************************************************************************
 *                               APIs Supported by this driver
 *               For more information about the APIs check function definitions
************************************************************************************************************/

/*
 *  Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2C,uint8_t EnorDi);

/*
 *  Init and DeInit
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 *  Data Send or Recive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint8_t Len,uint8_t SlaveAddr,uint8_t Sr);

/*
 *  Data Send or Recive use interupt
 */

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer,uint32_t Len,uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

/*
 *  IRQ Congiguration and ISR handling
 */
void I2C_IRQIneruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

void I2CCloseSendData(I2C_Handle_t *pI2CHandle);
void I2CCloseReceiveData(I2C_Handle_t *pI2CHandle);

/*
 *  Other Peripheral Control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);


/*
 * Application callback
 */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F402XX_I2C_DRIVER_H_ */
