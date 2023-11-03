/*
 * stm32f4xx_spi_driver.c
 *
 *  Created on: Oct 29, 2023
 *      Author: Admin
 */
#include "stm32f4xx.h"
#include "stm32f4xx_spi_driver.h"
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp_reg = 0;
	//1. config the device mode
	temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;
	pSPIHandle->pSPIx->CR1 &= ~(0x1 << 2);
	pSPIHandle->pSPIx->CR1 |= temp_reg;

	//2. config the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		temp_reg &= ~(1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		temp_reg |= (1 << 15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		temp_reg &= ~(1 << 15);
		temp_reg |= (1 << 10);
	}

	//3. config the spi serial clock speed (baudrate)
	temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << 3;

	//4. config the DFF
	temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	//5. config the CPOL
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	//6. config the CPHA
	temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	//pSPIHandle->pSPIx->CR1 &= ~(0xFF);
	pSPIHandle->pSPIx->CR1 = temp_reg;
}

//
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{

	}
}
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	else
	{
		return FLAG_RESET;
	}
}
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TX buff is empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. CHECK THE DFF BIT IN CRC1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 BIT DFF
			//load data
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			 // 8 bit dff
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}
