/*
 * stm32f4xx.h
 *
 *  Created on: Oct 22, 2023
 *      Author: Admin
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo		volatile
#define __weak		__attribute__((weak))
//
/*
 Arm cortex Mx Processor NVIC ISERx register Address
 */
#define NVIC_ISER0				((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1				((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2				((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3				((__vo uint32_t*)0xE000E10C)

//ARM CORTEX Mx PROCESSOR NVIC ICERx register address
#define NVIC_ICER0				((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1				((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2				((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3				((__vo uint32_t*)0xE000E18C)
// arm cortex m4x nvic IPRx register address
#define NVIC_IPR_BASEADDR		((__vo uint32_t*)0xE000E400)
//
#define NO_IPR_BITS_IMPLEMENTED	4
//base addresses of flash and SRAM memories
#define FLASH_BASEADDR			0x08000000UL
#define SRAM1_BASEADDR			0x20000000UL // 112KB = 114688 Byte => Hex: 1C000
#define SRAM2_BASEADDR			0x2001C000UL // SRAM2_BASEADDR = SRAM1_BASEADDR + 1C000
#define ROM						0x1FFF0000UL
#define SRAM					SRAM1_BASEADDR
//Advanced High-performance Bus(AHBx) and Advanced Peripheral Bus (APBx) base address
#define PERIPH_BASEADDR			0x40000000UL
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000UL
#define AHB1PERIPH_BASEADDR		0x40020000UL
#define AHB2PERIPH_BASEADDR		0x50000000UL

// Base addresses of peripherals which are hanging on AHB1_bus
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)	//OFFSET 0x0000
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)
// base addresses of peripherals which are hanging on APB1 bus
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

// base addresses of peripherals which are hanging on APB2 bus
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400)
//
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)
//

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRRL;
	__vo uint32_t BSRRH;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;
//
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
		 uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
		 uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	     uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	     uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
		 uint32_t RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    	 uint32_t RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    	 uint32_t RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;
//
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;
//
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
		 uint32_t RESERVED1[2];
	__vo uint32_t EXTICR[4];
		 uint32_t RESERVED2[2];
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

//
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;
//
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
} I2C_RegDef_t;
//peripheral base addresses typecasted to xxx_RegDef_t
#define GPIOA			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG 			((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
//
#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)

// GPIO PERIPHERAL CLOCK ENABLE
#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))
//GPIO peripheral clock disable
#define GPIO_PLCK_DI()		(RCC->AHB1ENR &=~(1 << 0))
// I2C CLOCK ENABLE
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))
// peripheral definition macros
#define I2C1				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*)I2C3_BASEADDR)
// I2C CR1 BIT POSITION
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15
// I2C CR2 REGISTER BIT POSITION
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12
// I2C SR1 REGISTER BIT POSITION
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define	I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RxNE		6
#define I2C_SR1_TxNE		7
#define I2C_SR1_BERR		8
#define I2C_SR1_APLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15
//
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

// Bit definition I2C_CCR

#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

// bit position OAR reg
#define I2C_OAR1_7BIT_ADDR	1
//SPI
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))


// UART CLOCK ENABLE
#define USART1_PCCK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN()	(RCC->APB1ENR |= (1 << 5))

// CLOCK ENABLE MACROS FOR SYSCFG PERIPHERAL
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

//CLOCK DISBALE MACROS FOR GPIO PERIPHERAL
#define GPIO_PCLK_DI()
//SOME GENERIC MACROS
#define ENABLE			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define FLAG_SET		SET
#define FLAG_RESET		RESET
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

// GPIO peripheral reset
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)
//
#define GPIO_BASEADDR_TO_CODE(x)	(x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : \
									(x == GPIOG) ? 6 : \
									(x == GPIOH) ? 7 : \
									(x == GPIOI) ? 8 : 0

// Interrupt Request numbers of stm32f407x MCU
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * BIT POSITION DEFINITIONS OF SPI PERIPHERAL SPI_CR1
 * */
#define SPI_CR1_CPHA		0	//CLOCK PHASE
#define SPI_CR1_CPOL		1	//CLOCK POLARITY
#define SPI_CR1_MSTR		2	//MASTER SELECTION
#define SPI_CR1_BR			3	//BAUDRATE CONTROL
#define SPI_CR1_SPE			6	//SPI ENABLE
#define SPI_CR1_LSDFIRST	7	//FRAME FORMAT
#define SPI_CR1_SSI			8	//INTERNAL SLAVE SELECT
#define SPI_CR1_SSM			9	//SOFTWARE SLAVE MANAGEMENT
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11	//DATA FRAME FORMAT
#define SPI_CR1_CRCNEXT		12	//CRC TRANSFER NETXT
#define SPI_CR1_CRCEN		13	//CRC CALCULATION ENABLE
#define SPI_CR1_BIDIOE		14	//OUTPUT ENABLE IN BIDIRECTIONAL MODE
#define SPI_CR1_BIDIMODE	15	//BIDIRECTIONAL DATA MODE ENABLE

// BIT POSITION DEFINITIONS OF SPI PERIPHERAL SPI_CR2
#define SPI_CR2_RXDMAEN		0	//RX BUFFER DMA ENABLE
#define SPI_CR2_TXDMAEN		1	//TX BUFFER DMA ENABLE
#define SPI_CR2_SSOE		2	//SS OUTPUT ENABLE
#define SPI_CR2_FRF			3	//FRAME FORMAT
#define SPI_CR2_ERRIE		4	//ERROR INTERRUPT ENABLE
#define SPI_CR2_RXNEIE		5	//RX BUFFER NOT EMPTY INTERRUPT ENABLE
#define SPI_CR2_TXEIE		6	//TX BUFFER EMPTY INTERRUPT ENABLE

// BIT POSITION OF SPI_SR (STATUS REGISTER)
#define SPI_SR_RXNE			0	//RECEIVE BUFFER NOT EMPTY
#define SPI_SR_TXE			1	//TX BUFFER EMPTY
#define SPI_SR_CHSIDE		2	//CHANNEL SIDE
#define SPI_SR_UDR			3	//UNDERRUN FLAG
#define SPI_SR_CRCERR		4	//CRCR ERROR FLAG
#define SPI_SR_MODF			5	//MODE FAULT
#define SPI_SR_OVR			6	//OVERRUN FLAG
#define SPI_SR_BSY			7	//BUSY FLAG
#define SPI_SR_FRE			8	//FRAME FORMAT ERROR
#endif /* INC_STM32F4XX_H_ */
