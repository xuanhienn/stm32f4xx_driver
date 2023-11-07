/*
 * stm32f4xx_i2c_drivers.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Admin
 */
#include "stm32f4xx.h"
#include "stm32f4xx_i2c_drivers.h"
uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescaler[4] = {2, 4, 8, 16};
void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2CHandle->pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2CHandle->pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2CHandle->pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}
}
uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t SystemClk, pclk1;
	uint8_t clksrc, ahb_prsclr,apb_prsclr, temp;
	clksrc = ((RCC->CFGR >> 2) & 0X3);
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}else if(clksrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahb_prsclr = 1;
	}
	else
	{
		ahb_prsclr = AHB_Prescaler[temp - 8];
	}
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb_prsclr = 1;
	}
	else
	{
		apb_prsclr = APB_Prescaler[temp - 4];
	}
	pclk1 = (SystemClk / ahb_prsclr)/apb_prsclr;
	return pclk1;
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp = 0;
	// ack control bit
	temp |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->CR1 = temp;

	// configure the FREQ field of CR2
	temp = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x1F);
	//1. configure the mode

	//2. config the speed of serial clock

	//3. configure the device address

	//4. enable the ACK

	//5. configure for the rise time for I2C time
}
