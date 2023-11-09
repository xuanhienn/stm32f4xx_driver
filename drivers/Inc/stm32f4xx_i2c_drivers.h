/*
 * stm32f4xx_i2c_drivers.h
 *
 *  Created on: Nov 7, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F4XX_I2C_DRIVERS_H_
#define INC_STM32F4XX_I2C_DRIVERS_H_
#include "stm32f4xx.h"
//I2C MASTER MODE
#define I2C_SCL_SPEED_SM			100000
#define I2C_SCL_SPEED_FM2K			200000
#define I2C_SCL_SPEED_FM4K

#define I2C_FM_DUTY_2				0
#define I2C_FM_DUTY_16_9			1


typedef struct
{
	uint32_t	I2C_SCLSpeed;
	uint8_t		I2C_DeviceAddress;
	uint8_t		I2C_AckControl;
	uint16_t	I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;
void I2C_PeriClockControl(I2C_Handle_t *pI2CHandle, uint8_t EnorDi);
void I2C_Init(I2C_Handle_t *pI2CHandle);

#endif /* INC_STM32F4XX_I2C_DRIVERS_H_ */
