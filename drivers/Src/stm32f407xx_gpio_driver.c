/*
 * stm32f4xx_gpio_driver.c
 *
 *  Created on: Aug 17, 2024
 *      Author: suraj
 */
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include <stdint.h>

/*
 *@brief: API to Enable/Disable GPIO Peripheral Clock Control.
 *@param1: Pointer to the GPIO Base Address.
 *@param2: An unsigned int macro to enable or disable the GPIO Peripheral Clock.
 *@return: None.
 * */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if(pGPIOx == GPIOA) {

			GPIOA_PCLK_EN();

		} else if (pGPIOx == GPIOB) {

			GPIOB_PCLK_EN();

		} else if (pGPIOx == GPIOC) {

			GPIOC_PCLK_EN();

		} else if (pGPIOx == GPIOD) {

			GPIOD_PCLK_EN();

		} else if (pGPIOx == GPIOD) {

			GPIOD_PCLK_EN();

		} else if (pGPIOx == GPIOE) {

			GPIOE_PCLK_EN();

		} else if (pGPIOx == GPIOF) {

			GPIOF_PCLK_EN();

		} else if (pGPIOx == GPIOG) {

			GPIOG_PCLK_EN();

		} else if (pGPIOx == GPIOH) {

			GPIOH_PCLK_EN();

		} else if (pGPIOx == GPIOI) {

			GPIOI_PCLK_EN();

		} else {

			if(pGPIOx == GPIOA) {

				GPIOA_PCLK_DI();

			} else if (pGPIOx == GPIOB) {

				GPIOB_PCLK_DI();

			} else if (pGPIOx == GPIOC) {

				GPIOC_PCLK_DI();

			} else if (pGPIOx == GPIOD) {

				GPIOD_PCLK_DI();

			} else if (pGPIOx == GPIOD) {

				GPIOD_PCLK_DI();

			} else if (pGPIOx == GPIOE) {

				GPIOE_PCLK_DI();

			} else if (pGPIOx == GPIOF) {

				GPIOF_PCLK_DI();

			} else if (pGPIOx == GPIOG) {

				GPIOG_PCLK_DI();

			} else if (pGPIOx == GPIOH) {

				GPIOH_PCLK_DI();

			} else if (pGPIOx == GPIOI) {

				GPIOI_PCLK_DI();

			}

		}

	}
}

/*
 *@brief: API to Initialize the GPIO Peripheral.
 *@param1: Pointer to the GPIO Handle.
 *@return: None.
 * */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
		uint32_t temp = 0;

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {

			/* Non-interrupt mode(ref. GPIO Pin Mode #defines in the header)*/
			/* Configure the mode of the GPIO Pin */
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* Clearing */
			pGPIOHandle->pGPIOx->MODER |= temp; /* Setting */
			temp = 0;

		} else {

			/*Interrupt Mode */
			if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {

				/* Configure the FTSR */
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				/* Clear the corresponding RTSR Bit */
				EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

				/* Configure the RTSR */
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				/* Clear the corresponding FTSR Bit */
				EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


			} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RTFT){
				/* Configure the RTSR & FTSR */
				EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			/* Configure Port Selection in SYSCFG_EXTICR */
			uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
			uint8_t temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
			uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
			SYSCFG_PCLK_EN();
			SYSCFG->EXTICR[temp1] = portcode << (temp2 << 4);

			/* Enable the interrupt delivery using the interrupt mask register(IMR). */
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		temp = 0;
		/* Configure the speed of the GPIO Pin */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* Clearing */
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;

		/* Configure the PuPd settings */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); /* Clearing */
		pGPIOHandle->pGPIOx->PUPDR |= temp;
		temp = 0;

		/* Configure the OPtype */
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;
		temp = 0;

		/* Configure Alternate Function Mode */
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
			uint8_t temp1 = 0, temp2 = 0;

			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
		}




}

/*
 *@brief: API to De-Initialize the GPIO Peripheral.
 *@param1: Pointer to the GPIO Base Address.
 *@return: None.
 * */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

			if(pGPIOx == GPIOA) {

				GPIOA_REG_RESET();

			} else if (pGPIOx == GPIOB) {

				GPIOB_REG_RESET();

			} else if (pGPIOx == GPIOC) {

				GPIOC_REG_RESET();

			} else if (pGPIOx == GPIOD) {

				GPIOD_REG_RESET();

			} else if (pGPIOx == GPIOE) {

				GPIOE_REG_RESET();

			} else if (pGPIOx == GPIOF) {

				GPIOF_REG_RESET();

			} else if (pGPIOx == GPIOG) {

				GPIOG_REG_RESET();

			} else if (pGPIOx == GPIOH) {

				GPIOH_REG_RESET();

			} else if (pGPIOx == GPIOI) {

				GPIOI_REG_RESET();

			}
}

/*
 *@brief: API to Read from GPIO Input Pin.
 *@param1: Pointer to the GPIO Base Address.
 *@param2: An unsigned int macro specifying the pin number.
 *@return: Read uint8_t value.
 * */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/*
 *@brief: API to Read from GPIO Input Port.
 *@param1: Pointer to the GPIO Base Address.
 *@return: An uint16_t parameter containing the read port value.
 * */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

}


/*
 *@brief: API to write to GPIO Output Pin.
 *@param1: Pointer to the GPIO Base Address.
 *@param2: An uint8_t parameter specifying the pin number.
 *@param3: An uint8_t macro specifying the value to be written.
 *@return: None.
 * */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET) {

		pGPIOx->ODR |= (1 << PinNumber);

	} else {

		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

/*
 *@brief: API to Write to GPIO Output Port.
 *@param1: Pointer to the GPIO Base Address.
 *@param2: An uint16_t parameter specifying the value to be written.
 *@return: None.
 * */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value;
}

/*
 *@brief: API to Toggle a GPIO output pin.
 *@param1: Pointer to the GPIO Base Address.
 *@param2: An uint16_t parameter specifying the pin number.
 *@return: None.
 * */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *@brief: API to Configure the GPIO Interrupts.
 *@param1: An uint16_t value specifying the IRQ.
 *@param2: An uint16_t parameter specifying the Interrupt Priority.
 *@param3: An uint8_t macro specifying the value to be set.
 *@return: None.
 * */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if (IRQNumber <= 31) {
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//Program ISER2 register
			*NVIC_ISER3 |= (1 << IRQNumber % 64);
		}
	} else {
		if (IRQNumber <= 31) {
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			//Program ISER2 register
			*NVIC_ISER3 |= (1 << IRQNumber % 64);

		}
	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx * 4 )) |= (IRQPriority << shift_amount);
}

/*
 *@brief: ISR Routine to Handle GPIO Interrupts.
 *@brief: An uint8_t specifying the GPIO Pin Number.
 *@return: None.
 * */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}
