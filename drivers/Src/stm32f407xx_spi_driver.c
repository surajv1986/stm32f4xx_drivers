/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Oct 12, 2024
 *      Author: Dell
 */

#include "stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"


void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
			if(pSPIx == SPI1) {

				SPI1_PCLK_EN();

			} else if (pSPIx == SPI2) {

				SPI2_PCLK_EN();

			} else if (pSPIx == SPI3) {

				SPI3_PCLK_EN();

			}

		} else {


		}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

void SPI_sendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

void SPI_receiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}
