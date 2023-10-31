/*
 * stm32f4xx_spi_driver.h
 *
 *  Created on: Oct 29, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F4XX_SPI_DRIVER_H_
#define INC_STM32F4XX_SPI_DRIVER_H_
#include "stm32f4xx.h"
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
