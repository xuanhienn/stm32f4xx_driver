/*
 * stm32f4xx_i2c_drivers.c
 *
 *  Created on: Nov 7, 2023
 *      Author: Admin
 */
#include "stm32f4xx.h"
#include "stm32f4xx_i2c_drivers.h"
#include "stdio.h"
uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB_Prescaler[4] = {2, 4, 8, 16};
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearAddrFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);

//void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t EventCode);
/* ******************************************************************************
 * @fn					-  I2C_MasterHandleTXEInterrupt
 *
 * @brief				-  Receive data over I2C bus from master
 *
 * @param[in]			-  Base address of the I2C Handle
 *
 * @return				-  None
 *
 * @note				-  None
 *
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}
static void I2C_MasterHandleRXNEInterrutp(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}
	if(pI2CHandle->RxLen == 0)
	{

		if(pI2CHandle->Sr == DISABLE)
		{
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		I2C_CloseReceiveData(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_CMPLT);
	}

}
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// disable ITBUFEN to prevent generation of more RXNE or TXE interrupts
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
//	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}
//static void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
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
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(1); //SLAVE ADDRESS + R/NW BIT = 0;
	pI2Cx->DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1;
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
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
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

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	//1. generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2. confirm that start generation is completed by checking SB flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_SB));
	//3. send address of the slave with r/nw bit to set to R(1)
	//pI2CHandle->pI2Cx->DR = (SlaveAddr << 1) | 0;
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until  address phase is completed by checking the ADDR flag in SR1
	while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_ADDR));
	if(Len == 1)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		//generate stop condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//clear addr flag
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		//WAIT UNTIL RXNE BECOME 1
		while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_RXNE));

		//read data in buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
		return;

	}
	if(Len > 1)
	{
		//clear addr flag
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
		//read the data until len becomes 0
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until RNXE becomes 1
			while(! I2C_GetFlagStatus(pI2CHandle, I2C_FLAG_RXNE));
			if(i == 2)
			{
				// clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				//generate stop condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//READ THE DATA TO THE BUFFER
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}

	}
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	if((busystate != I2C_BUSY_IN_TX) | (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->Sr = Sr;

		//implement code to generate start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
		//IMLEMENT CODE TO ENABLE ITBUFEN CONTROL BIT
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// IMPLEMENT THE CODE TO ENABLE ITEVTEN control bit;
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
		// implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}
	return busystate;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	//
	if(temp1 && temp3)
	{
		//sb flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		I2C_ClearAddrFlag(pI2CHandle->pI2Cx);
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF is set
		//check the state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
				if(pI2CHandle->TxLen == 0)
				{
					//BTF = 1, TXE = 1
					//1. generate stop condition
					if(pI2CHandle->Sr == DISABLE)
					{
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//2. reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);
					//3. notify the application about the transmission complete
					I2C_ApplicationEventCallback(pI2CHandle,I2C_EVENT_TX_CMPLT);
				}
			}
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;// this line is intended to be blank
		}
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
//		uint8_t dummyRead = pI2CHandle->pI2Cx->SR1;
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		//notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_STOP);
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RxNE);
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_MasterHandleRXNEInterrutp(pI2CHandle);
		}
		else
		{

			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_RX_CMPLT);
			}
		}
	}
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 &= (1 << I2C_SR2_MSL))
		{
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EVENT_TX_CMPLT);
			}
		}
	}

}

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint8_t temp1, temp2;
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN);
	/********************* check for the bus error****************/
	temp1 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		// NOTIFY THE APPLICATION
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/********************* check for the ACK failure error****************/
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		// clear AF flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		//NOTIFY THE APPLICATION
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_APLO);
	if(temp1 && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_APLO);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_APLO);
	}
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR);
	if(temp1 && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}
	temp2 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t EventCode)
{
	switch(EventCode)
	{
	case I2C_EVENT_RX_CMPLT:
		printf("Receive data completed \n");
		break;
	case I2C_EVENT_STOP:
		printf("stop\n");
		break;
	case I2C_EVENT_TX_CMPLT:
		printf("I2C_EVENT_TX_CMPLT\n");
		break;
	case I2C_ERROR_OVR:
		printf("I2C_ERROR_OVR\n");
		break;
	case I2C_ERROR_AF:
		printf("I2C_ERROR_AF\n");
		break;
	case I2C_ERROR_APLO:
		printf("I2C_ERROR_APLO\n");
		break;
	case I2C_ERROR_BERR:
		printf("I2C_ERROR_BERR\n");
		break;
	case I2C_ERROR_TIMEOUT:
		printf("TIMEOUT\n");
		break;
	}
}

