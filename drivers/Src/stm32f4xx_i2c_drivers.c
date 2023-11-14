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
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
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
	temp = 0;
	temp = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (temp & 0x1F);

	temp = 0;
	temp |= pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_7BIT_ADDR;
	temp |= (1 << 14); //14th bit should always be kept at 1 by software
	pI2CHandle->pI2Cx->OAR1 = temp;
	//1. configure the mode
	temp = 0;
	uint16_t ccr_value = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM)
	{
		ccr_value = RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		temp |= (ccr_value & 0xFFF);
	}
	else
	{
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			temp |= (ccr_value & 0xFFF);
		}
		else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
			temp |= (ccr_value & 0xFFF);
		}
	}
	pI2CHandle->pI2Cx->CCR = temp;
	//2. configure the speed of serial clock

	//3. configure the device address

	//4. enable the ACK

	//5. configure for the rise time for I2C time
	uint32_t rise_time = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM)
	{
		rise_time = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		rise_time = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (rise_time & 0x3F);
}
uint8_t I2C_GetFlagStatus(I2C_Handle_t *pI2CHandle, uint32_t FlagName)
{
	if(pI2CHandle->pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}
void static I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. Generate the Start condition
	//pI2CHandle->pI2Cx->CR1 |= I2C_CR1_START;
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2. checking SB flag
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_SB) == FLAG_RESET);
	//3. SEND THE ADDRESS OF THE SLAVE
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);
	//4. CONFIEM THAT ADDRESS PHASE IS COMPLETED BY CHECKING THE ADDR FLAG IN THE SR1
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR));
	//5. CLEAR THE ADDR FLAG ACCORDING TO ITS SOFTWARE SEQUENCE
	I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
	//6. send the data until the length becomes 0
	while(Len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}
	//7. when Len becomes 0, wait for TxE=1 and BTF=1 before generating the STOP condition.
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_TXE));
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_BTF));

	//8. generate STOP condition
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}



