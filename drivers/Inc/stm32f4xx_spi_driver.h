/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Oct 29, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_
#include "stm32f4xx.h"

//SPI DeviceMode, BIT 2 IN SPI_CR1 REG
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

//SPI bus config, BIT 15-14 IN SPI_CR1(BIDIMODE & BIDIOE)
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

//SPI CLOCK SPEED
#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPPED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

//SPI DFF DATASIZE
#define SPI_DFF_8BITS					0
#define SPI_DFF_16BITS					1

//SPI CPOL
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

//SPI CPHA
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

//SPI SOFTWARE SLAVE MANAGEMENT
#define SPI_SSM_EN						1
#define SPI_SSM_DI						0

//SPI STATUS FLAG DEFINITION
#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;
/*
 * Peripheral clock setup
 * */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * Init and DeInit
 * */
 void SPI_Init(SPI_Handle_t *pSPIHandle);
 void SPI_DeInit(SPI_RegDef_t *pSPIx);
 // Data send and receive
 void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
 void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

 //IRQ configuration and ISR handling
 void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
 void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
 void SPI_IRQHandling(SPI_Handle_t *pHandle);
#endif /* INC_STM32F4XX_SPI_DRIVER_H_ */
