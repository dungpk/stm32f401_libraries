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
#define I2CP_ACK_DISABLE         0

/*
 * @IC2_DutyCylce
 */
#define I2C_FM_DUTY_2             0
#define I2C_FM_DUTY_16_9          1


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



/*
 *  IRQ Congiguration and ISR handling
 */
void I2C_IRQIneruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);


/*
 *  Other Peripheral Control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


/*
 * Application callback
 */
__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F402XX_I2C_DRIVER_H_ */
