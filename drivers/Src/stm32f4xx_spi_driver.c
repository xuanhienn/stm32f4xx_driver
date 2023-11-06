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
	// ENABLE SPI PERIPHERAL CLOCK
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
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
//
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
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
	//1. check the len of data
	while(Len > 0)
	{
		//2. wait the TxFlag
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);
		//3. check the DFF bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);
		// check dff bit
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		}
		else
		{
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			Len--;
		}

	}
}
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	//save the buffer address and len to a global variable
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;
	// mark spi state as busy in tramission
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	//ENABLE TXEIE CONTROL BIT TO GET INTERRUPT WHEN TXE FLAG IS SET
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
void SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
}
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// check for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2){
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		spi_rxe_interrupt_handle(pHandle);
	}
	// check for overrun
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else
	{
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		// clear txeie
//		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
//		pSPIHandle->pTxBuffer = NULL;
//		pSPIHandle->TxLen = 0;
//		pSPIHandle->TxState = SPI_READY;
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}
	else
	{
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}
	if(!pSPIHandle->RxLen)
	{
//		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
//		pSPIHandle->pRxBuffer = NULL;
//		pSPIHandle->RxLen = 0;
//		pSPIHandle->RxState = SPI_READY;
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);
