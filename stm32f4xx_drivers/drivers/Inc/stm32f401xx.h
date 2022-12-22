/*
 * stm32f401xx.h
 *
 *  Created on: Dec 18, 2022
 *      Author: Dungpk
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define _vo volatile

/********************** START: Processor Specific Details ************************************
 *
 * ARM Cortex Mx Processor NVIC ISERx register Address
 */

#define NVIC_ISER0                                  ( (_vo uint32_t *)0xE000E100 )
#define NVIC_ISER1                                  ( (_vo uint32_t *)0xE000E104 )
#define NVIC_ISER2                                  ( (_vo uint32_t *)0xE000E108 )
#define NVIC_ISER3                                  ( (_vo uint32_t *)0xE000E10C )


/*
 *  ARM Cortex Mx Processor NVIC ISECx register Address
 */

#define NVIC_ICER0                                  ( (_vo uint32_t *)0xE000E180 )
#define NVIC_ICER1                                  ( (_vo uint32_t *)0xE000E184 )
#define NVIC_ICER2                                  ( (_vo uint32_t *)0xE000E188 )
#define NVIC_ICER3                                  ( (_vo uint32_t *)0xE000E18C )

/*
 *  ARM Cortex Mx Processor Priority register Address
 */
#define NVIC_PR_BASE_ADDR                           ( (_vo uint32_t *)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED                      4

/* base address of Flash and SRAM memories */

#define FLASH_BASEADDR                               0x08000000U
#define SRAM1_BASEADDR                               0x20000000U
#define SRAM2_BASEADDR                               0x20001C00U
#define ROM_BASEADDR                                 0x1FFF0000U
#define SRAM                                         SRAM1_BASEADDR

/*
 * define AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASE                                  0x40000000U
#define APB1PERIPH_BASE                              PERIPH_BASE
#define APB2PERIPH_BASE                              (PERIPH_BASE + 0x00010000)
#define AHB1PERIPH_BASE                              (PERIPH_BASE + 0x00020000)
#define AHB2PERIPH_BASE                              (PERIPH_BASE + 0x10000000)


/*
 * Base addresses of  peripherals which are hanging on AHB1 bus
 * TODO : Complete for all  other peripherals
 */

#define GPIOA_BASEADDR                                (AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR                                (AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR                                (AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR                                (AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR                                (AHB1PERIPH_BASE + 0x1000)
#define GPIOH_BASEADDR                                (AHB1PERIPH_BASE + 0x1C00)

#define RCC_BASEADDR                                  (AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of  peripherals which are hanging on APB1 bus
 * TODO : Complete for all  other peripherals
 */

#define I2C1_BASEADDR                                  (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR                                  (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR                                  (APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR                                  (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR                                  (APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADD							       (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADD							       (APB1PERIPH_BASE + 0x4800)
#define USART4_BASEADD							       (APB1PERIPH_BASE + 0x4C00)
#define USART5_BASEADD							       (APB1PERIPH_BASE + 0x5000)

/*
 * Base addresses of  peripherals which are hanging on APB2 bus
 * TODO : Complete for all  other peripherals
 */

#define EXTI_BASEADDR                                   (APB2PERIPH_BASE + 0x3C00)
#define SPI1_BASEADDR                                   (APB2PERIPH_BASE + 0x3000)
#define SYSCFG_BASEADDR                                 (APB2PERIPH_BASE + 0x3800)
#define USART1_BASEADD	                                (APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADD                                  (APB2PERIPH_BASE + 0x1400)

/*
 * Registor of a perioheral are sprecific  to MCU
 *
 */

typedef struct
{
	_vo uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	_vo uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	_vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	_vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	_vo uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	_vo uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	_vo uint32_t BSRR;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
	_vo uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	_vo uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 *
 */

typedef struct
{
	_vo uint32_t  IMR;  /* !< Give a short description ,                          Address offset : 0x00*/
	_vo uint32_t  EMR;  /* !< TODO,                                               Address offset : 0x04*/
	_vo uint32_t  RTSR; /* !< TODO,                                               Address offset : 0x08*/
	_vo uint32_t  FTSR; /* !< TODO,                                               Address offset : 0x0C*/
	_vo uint32_t  SWIER;/* !< TODO,                                               Address offset : 0x10*/
	_vo uint32_t  PR;   /* !< TODO,                                               Address offset : 0x14*/

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	_vo uint32_t  MEMRMP;    /* !< Give a short description ,                          Address offset : 0x00        */
	_vo uint32_t  PMC;       /* !< TODO,                                               Address offset : 0x04        */
	_vo uint32_t  EXTICR[4]; /* !< TODO,                                               Address offset : 0x08 - 0x14 */
	uint32_t  RESERVED1[2];  /* !< TODO,                                               Address offset : 0x08 - 0x1C */
	_vo uint32_t  CMPCR;     /* !< TODO,                                               Address offset : 0x20        */
	 uint32_t  RESERVED2[2]; /* !< TODO,                                               Address offset : 0x24 - 0x28 */
	_vo uint32_t  CFGR;      /* !< TODO,                                               Address offset : 0x2C        */


}SYSCFG_RegDef_t;

/*
 *  Peripheral definitions
 */

#define GPIOA                                              ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB                                              ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC                                              ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD                                              ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE                                              ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOH                                              ((GPIO_RegDef_t *) GPIOH_BASEADDR)

#define RCC                                                ((RCC_RegDef_t *) RCC_BASEADDR)

#define EXTI                                               ((EXTI_RegDef_t *) EXTI_BASEADDR)

#define SYSCFG                                             ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  _vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  _vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  _vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  _vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  _vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  _vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  _vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  _vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  _vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  _vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  _vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  _vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  _vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  _vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  _vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  _vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  _vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  _vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  _vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  _vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  _vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  _vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  _vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  _vo uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
  _vo uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
  _vo uint32_t CKGATENR;      /*!< RCC Clocks Gated Enable Register,                            Address offset: 0x90 */ /* Only for STM32F412xG, STM32413_423xx and STM32F446xx devices */
  _vo uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */ /* Only for STM32F410xx, STM32F412xG, STM32413_423xx and STM32F446xx devices */

} RCC_RegDef_t;

/*
 * Clock Enable Marcos for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 4) )
#define GPIOH_PCLK_EN()      ( RCC->AHB1ENR |= (1 << 7) )

/*
 * Clock Enable Marcos for i2Cx peripherals
 */

#define I2C1_PCLK_EN()      ( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Marcos for SPIx peripherals
 */

#define SPI1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()      ( RCC->APB1ENR |= (1 << 15) )

/*
 * Clock Enable Marcos for USARTx peripherals
 */

#define USART1_PCLK_EN()      ( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()      ( RCC->APB1ENR |= (1 << 17) )
#define USART6_PCLK_EN()      ( RCC->APB2ENR |= (1 << 5) )
/*
 * Clock Enable Marcos for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()      ( RCC->APB2ENR |= (1 << 14) )

/*
 * Clock Disable Marcos for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOH_PCLK_DI()      ( RCC->AHB1ENR &= ~(1 << 7) )

/*
 * Clock Disable Marcos for i2Cx peripherals
 */

#define I2C1_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock Disable Marcos for SPIx peripherals
 */

#define SPI1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 15) )

/*
 * Clock Disable Marcos for USARTx peripherals
 */

#define USART1_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()      ( RCC->APB1ENR &= ~(1 << 17) )
#define USART6_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 5) )
/*
 * Clock Disable Marcos for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()      ( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Marcos to reset  GPIOx peripherals
 */
#define GPIOA_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<0)); }while(0);
#define GPIOB_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<1)); }while(0);
#define GPIOC_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<2)); }while(0);
#define GPIOD_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<3)); }while(0);
#define GPIOE_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<4)); }while(0);
#define GPIOH_REG_RESET()       do{ (RCC->AHB1RSTR |= (1<<0));(RCC->AHB1RSTR &= ~(1<<5)); }while(0);

/*
 * Return port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)          ( (x == GPIOA)?0:\
                                            (x == GPIOB)?1:\
                                            (x == GPIOC)?2:\
                                            (x == GPIOD)?3:\
                                            (x == GPIOE)?4:\
                                            (x == GPIOH)?7:0 )

/*
 * IRQ (Interupt Request) Number of stm32f4x MCU
 * NOTE : update macros with valid values according to your MCU
 * TODO: you may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0	     6
#define IRQ_NO_EXTI1	     7
#define IRQ_NO_EXTI2	     8
#define IRQ_NO_EXTI3	     9
#define IRQ_NO_EXTI4	     10
#define IRQ_NO_EXTI9_5	     23
#define IRQ_NO_EXTI15_11	 40

/*
 * Marcro for all the possible priority levels
 */
#define NVIC_IRQ_PRI0           0
#define NVIC_IRQ_PRI1           1
#define NVIC_IRQ_PRI2           3
#define NVIC_IRQ_PRI3           4
#define NVIC_IRQ_PRI4           5
#define NVIC_IRQ_PRI5           6
#define NVIC_IRQ_PRI6           7
#define NVIC_IRQ_PRI7           8
#define NVIC_IRQ_PRI8           9
#define NVIC_IRQ_PRI9           10
#define NVIC_IRQ_PRI10          11
#define NVIC_IRQ_PRI11          12
#define NVIC_IRQ_PRI12          13
#define NVIC_IRQ_PRI13          14
#define NVIC_IRQ_PRI15          15

/*
 * Some Generic marcros
 */
#define ENABLE           1
#define DISNABLE         0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET


#endif /* INC_STM32F401XX_H_ */
