/*
 * stm32f407xx.h
 *
 *  Created on: Aug 16, 2024
 *      Author: suraj
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_
#include <stdint.h>

/*
 * ARM Cortex MX Processor NVIC ISERx register addresses.
 *
 *
 * */
#define NVIC_ISER0     (volatile uint32_t *)0xE000E180
#define NVIC_ISER1     (volatile uint32_t *)0xE000E184
#define NVIC_ISER2     (volatile uint32_t *)0xE000E188
#define NVIC_ISER3     (volatile uint32_t *)0xE000E18C

/*
 * ARM Cortex MX Processor Priority Register Address Calculation
 *
 * */
#define NVIC_PR_BASE_ADDR   ((volatile uint32_t*)0xE00E400)

#define NO_PR_BITS_IMPLEMENTED   4

/*
 * Base Addresses for the SRAM & Flash Memories
 *
 * */
#define FLASH_BASEADDRESS 0x08000000U
#define SRAM1_BASEADDRESS 0x20000000U
#define SRAM2_BASEADDRESS 0x20001C00U
#define ROM_BASEADDRESS   0x1FFF0000U
#define OTP_BASEADDRESS   0x1FFF7800U

/*
 * APB and AHB Peripheral Base Addresses
 *
 * */
#define PERIPHERALBASE 0x40000000U
#define APB1_PERIPHERALBASE PERIPHERALBASE
#define APB2_PERIPHERALBASE 0x40010000U
#define AHB1_PERIPHERALBASE 0x40020000U
#define AHB2_PERIPHERALBASE 0x50000000U


/*
 * Base Addresses for the Peripherals on AHB1 bus.
 *
 * */
#define RCC_BASEADDR (APB1_PERIPHERALBASE + 0x3800)
#define GPIOA_BASEADDR (APB1_PERIPHERALBASE + 0x0000)
#define GPIOB_BASEADDR (APB2_PERIPHERALBASE + 0x0400)
#define GPIOC_BASEADDR (APB2_PERIPHERALBASE + 0x0800)
#define GPIOD_BASEADDR (APB2_PERIPHERALBASE + 0x0C00)
#define GPIOE_BASEADDR (APB2_PERIPHERALBASE + 0x1000)
#define GPIOF_BASEADDR (APB2_PERIPHERALBASE + 0x1400)
#define GPIOG_BASEADDR (APB2_PERIPHERALBASE + 0x1800)
#define GPIOH_BASEADDR (APB2_PERIPHERALBASE + 0x1C00)
#define GPIOI_BASEADDR (APB2_PERIPHERALBASE + 0x2000)

/*
 * Base Addresses for the Peripherals on APB1 bus.
 *
 * */
#define I2C1_BASEADDR (APB1_PERIPHERALBASE + 0x5400)
#define I2C2_BASEADDR (APB1_PERIPHERALBASE + 0x5800)
#define I2C3_BASEADDR (APB1_PERIPHERALBASE + 0x5C00)
#define SPI2_BASEADDR (APB1_PERIPHERALBASE + 0x3800)
#define SPI3_BASEADDR (APB1_PERIPHERALBASE + 0x3C00)
#define USART2_BASEADDR (APB1_PERIPHERALBASE + 0x4400)
#define USART3_BASEADDR (APB1_PERIPHERALBASE + 0x4800)
#define UART4_BASEADDR (APB1_PERIPHERALBASE + 0x4C00)
#define UART5_BASEADDR (APB1_PERIPHERALBASE + 0x5000)

/*
 * Base Addresses for the Peripherals on APB2 bus.
 *
 * */
#define EXTI_BASEADDR (APB2_PERIPHERALBASE + 0x3C00)
#define SPI1_BASEADDR (APB2_PERIPHERALBASE + 0x3000)
#define USART1_BASEADDR (APB2_PERIPHERALBASE + 0x1000)
#define USART6_BASEADDR (APB2_PERIPHERALBASE + 0x1400)
#define SYSCFG_BASEADDR (APB2_PERIPHERALBASE + 0x3800)


/*
 * Peripheral Structure Definition for RCC.
 *
 * */
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	     uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
		 uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
		 uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
		 uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	     uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6;
	uint32_t RESERVED7;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED8;
	uint32_t RESERVED9;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;


/*
 * Peripheral Structure Definition for EXTI.
 *
 * */
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral Structure Definition for SYSCFG.
 *
 * */
typedef struct {
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
			 uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
			 uint32_t RESERVED2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;

/*
 * Peripheral Structure Definition for GPIO.
 *
 * */
typedef struct {
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;

/*
 * Peripheral Structure Definition for SPI.
 *
 * */
typedef struct {
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;

}SPI_RegDef_t;



#define GPIOA   ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG   ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH   ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI   ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define RCC   	((RCC_RegDef_t *)RCC_BASEADDR)
#define SYSCFG  ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t *)SPI3_BASEADDR)
/*
 * Clock Enable Macros for GPIOx Peripherals.
 * */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() (RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx Peripherals.
 * */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx Peripherals.
 * */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx/UARTx Peripherals.
 * */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG Peripheral.
 * */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))



/*
 * Clock Disable Macros for GPIOx Peripherals.
 * */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 8))

#define EXTI ((EXTI_RegDef_t *)EXTI_BASEADDR)


/*
 * Clock Disable Macros for I2Cx Peripherals.
 * */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable Macros for SPIx Peripherals.
 * */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock Disable Macros for USARTx/UARTx Peripherals.
 * */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macros for SYSCFG Peripheral.
 * */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))

/*
 * Macros to reset GPIOx peripherals.
 * */
#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)


/*
 * Macro Returns a PORT code for a given GPIO Base Address.
 * This Macro returns a code between 0-8 for a given GPIO Base Address.
 *
 * */
#define GPIO_BASEADDR_TO_CODE(x) ((x == GPIOA)?0:\
								  (x == GPIOB)?1:\
								  (x == GPIOC)?2:\
								  (x == GPIOD)?3:\
								  (x == GPIOE)?4:\
								  (x == GPIOF)?5:\
								  (x == GPIOG)?6:\
								  (x == GPIOH)?7:\
								  (x == GPIOI)?8:0\
										  )
#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40




/* Generic Macros */
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#endif /* INC_STM32F407XX_H_ */
