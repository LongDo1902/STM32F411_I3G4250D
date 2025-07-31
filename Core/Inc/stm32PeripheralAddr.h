/*
 * @file	stm32PeripheralAddr.h
 * @brief	Peripheral memory-map and register-layout definitionos for **STM32F411xx**
 *
 * This header provides:
 * 		**Base-address constants** for on-chip peripherals
 * 		**Register-layout structure** that mirror the ref manual bitfield order
 * 		**Typed pointers** that cast a base address to its structure type
 * 		**Helper macros** ('GPIOA_GET_REG(MODER)') that yield the address of an individual register
 *
 *  Created on: Jul 16, 2025
 *      Author: dobao
 */

#ifndef INC_STM32PERIPHERALADDR_H_
#define INC_STM32PERIPHERALADDR_H_

#include <stdint.h>
#include <stdio.h>

/*
 * ----------------------------------------------------
 * Peripheral's Base Address Declarations
 * ----------------------------------------------------
 */
/* AHB1 Peripheral */
#define DMA2_BASE_ADDR	0x40026400U
#define DMA1_BASE_ADDR	0x40026000U
#define FLASH_BASE_ADDR	0x40023C00U
#define RCC_BASE_ADDR	0x40023800U
#define CRC_BASE_ADDR	0x40023000U
#define GPIOH_BASE_ADDR	0x40021C00U
#define GPIOE_BASE_ADDR	0x40021000U
#define GPIOD_BASE_ADDR	0x40020C00U
#define GPIOC_BASE_ADDR	0x40020800U
#define GPIOB_BASE_ADDR	0x40020400U
#define GPIOA_BASE_ADDR	0x40020000U

/* APB2 peripherals */
#define UART1_BASE_ADDR	0x40011000U
#define UART6_BASE_ADDR	0x40011400U
#define TIM1_BASE_ADDR	0x40010000U
#define TIM9_BASE_ADDR	0x40014000U
#define TIM10_BASE_ADDR	0x40014400U
#define TIM11_BASE_ADDR	0x40014800U
#define EXTI_BASE_ADDR		  0x40013C00U
#define ADC1_BASE_ADDR		  0x40012000U
#define ADC1_COMMON_BASE_ADDR 0x40012300U
#define SPI1_BASE_ADDR	0x40013000U
#define	SPI4_BASE_ADDR	0x40013400U
#define SPI5_BASE_ADDR	0x40015000U

/* APB1 peripherals */
#define UART2_BASE_ADDR	0x40004400U
#define TIM2_BASE_ADDR	0x40000000U
#define	TIM3_BASE_ADDR	0x40000400U
#define TIM4_BASE_ADDR	0x40000800U
#define TIM5_BASE_ADDR	0x40000C00U
#define SPI2_BASE_ADDR	0x40003800U
#define SPI3_BASE_ADDR	0x40003C00U

#define NVIC_BASE_ADDR 	0xE000E100U
#define VTOR_BASE_ADDR 0xE000ED08UL


/*
 * ----------------------------------------------------
 * Peripheral Register Address Offset Structures
 * ----------------------------------------------------
 */
/* @brief	General-purpose I/O register map */
typedef struct{
	volatile uint32_t MODER;	//GPIO port mode register
	volatile uint32_t OTYPER;	//GPIO port output type register
	volatile uint32_t OSPEEDR;	//GPIO port output speed register
	volatile uint32_t PUPDR;	//GPIO port pull-up/pull-down register
	volatile uint32_t IDR;		//GPIO port input data register
	volatile uint32_t ODR;		//GPIO port output data register
	volatile uint32_t BSRR;		//GPIO port bit set/reset register
	volatile uint32_t LCKR;		//GPIO port configuration lock register
	volatile uint32_t AFRL;		//GPIO alternate function low register
	volatile uint32_t AFRH;		//GPIO alternate function high register
}gpioRegOffset_t;

/* @brief	Universal synchronous/asynchronouse receiver transmitter */
typedef struct{
	volatile uint32_t UART_SR;		//Status Reg
	volatile uint32_t UART_DR;		//Data Reg
	volatile uint32_t UART_BRR;		//Baud-rate Reg
	volatile uint32_t UART_CR1;		//Control Reg 1
	volatile uint32_t UART_CR2;		//Control Reg 2
	volatile uint32_t UART_CR3;		//Control Reg 3
	volatile uint32_t UART_GTPR;	//Guard-time & prescaler Reg
}uartRegOffset_t;

/* @brief	General/advanced-purpose timer register map */
typedef struct{
	volatile uint32_t TIM_CR1;		//Control Reg 1
	volatile uint32_t TIM_CR2;		//Control Reg 2
	volatile uint32_t TIM_SMCR;		//Slave-mode Control Reg
	volatile uint32_t TIM_DIER;		//DMA/Interrupt Enable Reg

	volatile uint32_t TIM_SR;		//Status Reg
	volatile uint32_t TIM_EGR;		//Event-generation Reg
	volatile uint32_t TIM_CCMR1;	//Capture/Compare Mode Reg 1
	volatile uint32_t TIM_CCMR2;	//Capture/Compare Mode Reg 2

	volatile uint32_t TIM_CCER;		//Capture/Compare Enable Reg
	volatile uint32_t TIM_CNT;		//Counter
	volatile uint32_t TIM_PSC;		//Prescaler
	volatile uint32_t TIM_ARR;		//Auto-reload Reg

	volatile uint32_t TIM_RCR;		//Repetition Counter Reg
	volatile uint32_t TIM_CCR1;		//Capture/Compare Reg 1
	volatile uint32_t TIM_CCR2;		//Capture/Compare Reg 2
	volatile uint32_t TIM_CCR3;		//Capture/Compare Reg 3

	volatile uint32_t TIM_CCR4;		//Capture/Compare Reg 4
	volatile uint32_t TIM_BDTR;		//Break and Dead-time Reg
	volatile uint32_t TIM_DCR;		//DMA Control Reg
	volatile uint32_t TIM_DMAR;		//DMA Address for Full Transfer

	union{
		volatile uint32_t TIM2_OR;	//Option Reg TIM2
		volatile uint32_t TIM5_OR;	//Option Reg TIM5
		volatile uint32_t TIM11_OR;	//Option Reg TIM11
	};
}timerRegOffset_t;

/* @brief	Reset and Clock Control Register Map */
typedef struct{
	volatile uint32_t RCC_CR;			//0x00 (RCC Clock Control Reg)
	volatile uint32_t RCC_PLL_CFGR;		//0x04 (RCC PLL Config Reg)
	volatile uint32_t RCC_CFGR;			//0x08 (RCC Clock Config Reg)
	volatile uint32_t RCC_CIR;			//0x0C (RCC Clock Interrupt Reg)

	volatile uint32_t RCC_AHB1_RSTR;	//0x10 (RCC AHB1 Reset Reg)
	volatile uint32_t RCC_AHB2_RSTR;	//0x14 (RCC AHB2 Reset Reg)

	volatile uint32_t RESERVED0;
	volatile uint32_t RESERVED1;

	volatile uint32_t RCC_APB1_RSTR;	//0x20 (RCC APB1 Reset Reg)
	volatile uint32_t RCC_APB2_RSTR;	//0x24 (RCC APB2 Reset Reg)

	volatile uint32_t RESERVED2;
	volatile uint32_t RESERVED3;

	volatile uint32_t RCC_AHB1_ENR;		//0x30 (RCC AHB1 Clock Enable)
	volatile uint32_t RCC_AHB2_ENR;		//0x34 (RCC AHB2 Clock Enable)

	volatile uint32_t RESERVED4;
	volatile uint32_t RESERVED5;

	volatile uint32_t RCC_APB1_ENR;		//0x40 (RCC APB1 Clock Enable)
	volatile uint32_t RCC_APB2_ENR;		//0x44 (RCC APB2 Clock Enable)

	volatile uint32_t RESERVED6;
	volatile uint32_t RESERVED7;

	volatile uint32_t RCC_AHB1_LP_ENR;	//0x50 (RCC AHB1 Low Power Mode Enable)
	volatile uint32_t RCC_AHB2_LP_ENR;	//0x54 (RCC AHB2 Low Power Mode Enable)

	volatile uint32_t RESERVED8;
	volatile uint32_t RESERVED9;

	volatile uint32_t RCC_APB1_LP_ENR;	//0x60 (RCC APB1 Low Power Mode Enable)
	volatile uint32_t RCC_APB2_LP_ENR;	//0x64 (RCC APB2 Low Power Mode Enable)

	volatile uint32_t RESERVED10;
	volatile uint32_t RESERVED11;

	volatile uint32_t RCC_BDCR;			//0x70 (RCC Backup Domain Control Reg)
	volatile uint32_t RCC_CSR;			//0x74 (RCC Clock Control & Status Reg)

	volatile uint32_t RESERVED12;
	volatile uint32_t RESERVED13;

	volatile uint32_t RCC_SSCGR;		//0x80 (RCC Spread Spectrum Clock Generation Reg)
	volatile uint32_t RCC_PLL_I2S_CFGR;	//0x84 (RCC PLLI2S Config Reg)

	volatile uint32_t RESERVED14;

	volatile uint32_t RCC_DCK_CFGR;		//0x8C (RCC Dedicated Clocks Config Reg)
}rccRegOffset_t;

/* @brief	ADC Register Map (single-conversion ADC1) */
typedef struct{
	volatile uint32_t ADC_SR;		//Status Reg
	volatile uint32_t ADC_CR1;		//Control Reg 1
	volatile uint32_t ADC_CR2;		//Control Reg 2

	volatile uint32_t ADC_SMPR1;	//Sample Time Reg 1
	volatile uint32_t ADC_SMPR2;	//Sample Time Reg 2

	volatile uint32_t ADC_JOFR1;	//Injected Channel Data Offset Reg 1
	volatile uint32_t ADC_JOFR2;	//Injected Channel Data Offset Reg 2
	volatile uint32_t ADC_JOFR3;	//Injected Channel Data Offset Reg 3
	volatile uint32_t ADC_JOFR4;	//Injected Channel Data Offset Reg 4

	volatile uint32_t ADC_HTR;		//Watchdog Higher Threshold Register
	volatile uint32_t ADC_LTR;		//Watchdog Lower Threshold Register

	volatile uint32_t ADC_SQR1;		//Regular Sequence Reg 1
	volatile uint32_t ADC_SQR2;		//Reqular Sequence Reg 2
	volatile uint32_t ADC_SQR3;		//Regular Sequence Reg 3

	volatile uint32_t ADC_JSQR;		//Injected Sequence Reg

	volatile uint32_t ADC_JDR1;		//Injected Data Reg 1
	volatile uint32_t ADC_JDR2;		//Injected Data Reg 2
	volatile uint32_t ADC_JDR3;		//Injected Data Reg 3
	volatile uint32_t ADC_JDR4;		//Injected Data Reg 4

	volatile uint32_t ADC_DR;		//Regular Data Reg
}adcRegOffset_t;


typedef struct{
	volatile uint32_t ADC_CSR;	//Common Status Reg
	volatile uint32_t ADC_CCR;	//Common Control Reg
	volatile uint32_t ADC_CDR;	//Common Regular Data Reg
}adcCommonRegOffset_t;


typedef struct{
	volatile uint32_t DMA_LISR;		//Low Interrupt Status Reg
	volatile uint32_t DMA_HISR;		//High Interrupt Status Reg
	volatile uint32_t DMA_LIFCR;	//Low Interrupt Flag Clear Reg
	volatile uint32_t DMA_HIFCR;	//High Interurpt Flag Clear Reg

	volatile uint32_t DMA_S0CR;		//Stream 0 Config Reg
	volatile uint32_t DMA_S0NDTR;	//Stream 0 No. of Data Reg
	volatile uint32_t DMA_S0PAR;	//Stream 0 Peripheral Address Register
	volatile uint32_t DMA_S0M0AR;	//Stream 0 Memory 0 address Register
	volatile uint32_t DMA_S0M1AR;	//Stream 0 Memory 1 address Register
	volatile uint32_t DMA_S0FCR;	//Stream 0 FIFO Control Register

	/* Streams 1-7 follow the same layout */
	volatile uint32_t DMA_S1CR;
	volatile uint32_t DMA_S1NDTR;
	volatile uint32_t DMA_S1PAR;
	volatile uint32_t DMA_S1M0AR;
	volatile uint32_t DMA_S1M1AR;
	volatile uint32_t DMA_S1FCR;

	volatile uint32_t DMA_S2CR;
	volatile uint32_t DMA_S2NDTR;
	volatile uint32_t DMA_S2PAR;
	volatile uint32_t DMA_S2M0AR;
	volatile uint32_t DMA_S2M1AR;
	volatile uint32_t DMA_S2FCR;

	volatile uint32_t DMA_S3CR;
	volatile uint32_t DMA_S3NDTR;
	volatile uint32_t DMA_S3PAR;
	volatile uint32_t DMA_S3M0AR;
	volatile uint32_t DMA_S3M1AR;
	volatile uint32_t DMA_S3FCR;

	volatile uint32_t DMA_S4CR;
	volatile uint32_t DMA_S4NDTR;
	volatile uint32_t DMA_S4PAR;
	volatile uint32_t DMA_S4M0AR;
	volatile uint32_t DMA_S4M1AR;
	volatile uint32_t DMA_S4FCR;

	volatile uint32_t DMA_S5CR;
	volatile uint32_t DMA_S5NDTR;
	volatile uint32_t DMA_S5PAR;
	volatile uint32_t DMA_S5M0AR;
	volatile uint32_t DMA_S5M1AR;
	volatile uint32_t DMA_S5FCR;

	volatile uint32_t DMA_S6CR;
	volatile uint32_t DMA_S6NDTR;
	volatile uint32_t DMA_S6PAR;
	volatile uint32_t DMA_S6M0AR;
	volatile uint32_t DMA_S6M1AR;
	volatile uint32_t DMA_S6FCR;

	volatile uint32_t DMA_S7CR;
	volatile uint32_t DMA_S7NDTR;
	volatile uint32_t DMA_S7PAR;
	volatile uint32_t DMA_S7M0AR;
	volatile uint32_t DMA_S7M1AR;
	volatile uint32_t DMA_S7FCR;
}dmaRegOffset_t;

/* @brief	FLASH interface register map */
typedef struct{
	volatile uint32_t FLASH_ACR;
	volatile uint32_t FLASH_KEYR;
	volatile uint32_t FLASH_OPTKEYR;
	volatile uint32_t FLASH_SR;
	volatile uint32_t FLASH_CR;
	volatile uint32_t FLASH_OPTCR;
}flashRegOffset_t;

/* @brief	External interrupt/event controller register map */
typedef struct{
	volatile uint32_t IMR; 		//0x00 (Interrupt Mask Reg)
	volatile uint32_t EMR; 		//0x04 (Event Mask Reg)
	volatile uint32_t RTSR;		//0x08 (Rising Trigger Selection Reg)
	volatile uint32_t FTSR;		//0x0C (Falling Trigger Selection Reg)
	volatile uint32_t SWIER; 	//0x10 (Software Interrupt Event Reg)
	volatile uint32_t PR;		//0x14 (Pending Reg)
}extiRegOffset_t;

/* @brief	NVIC register map (subset: ISER/ICER/ISPR/ICPR/IABR/IPR) */
typedef struct{
	volatile uint32_t _ISER[8];		//0xE000E100 (Interrupt Set Enable Reg)
	uint32_t RESERVED0[24]; 		//Reserved gap (0xE000E120 to 0xE000E17F)
	volatile uint32_t _ICER[8]; 	//0xE000E180 (Interrupt Clear-Enable Reg)
	uint32_t RESERVED1[24];
	volatile uint32_t _ISPR[8]; 	//0xE000E200 (Interrupt Set Pending Reg)
	uint32_t RESERVED2[24];
	volatile uint32_t _ICPR[8]; 	//0xE000E280 (Interrupt Clear-Pending Reg)
	uint32_t RESERVED3[24];
	volatile uint32_t _IABR[8]; 	//0xE000E300 (Interrupt Active Bit Reg)
	uint32_t RESERVED4[56];
	volatile uint8_t _IPR[240]; 	//0xE000E400 (Interrupt Priority Reg 1 byte each)
}NVIC_t;

/*@brief	SPI Peripheral Register Map */
typedef struct{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t SPI_SR;
	volatile uint32_t SPI_DR;
	volatile uint32_t SPI_CRC_PR;
	volatile uint32_t SPI_RX_CRCR;
	volatile uint32_t SPI_TX_CRCR;
	volatile uint32_t SPI_I2S_CFGR;
	volatile uint32_t SPI_I2S_PR;
}spiRegOffset_t;

/*
 * ----------------------------------------------------
 * Peripheral Base-Address Cast Macros
 * ----------------------------------------------------
 */
#define GPIOA_REG ((volatile gpioRegOffset_t*)GPIOA_BASE_ADDR)
#define GPIOB_REG ((volatile gpioRegOffset_t*)GPIOB_BASE_ADDR)
#define GPIOC_REG ((volatile gpioRegOffset_t*)GPIOC_BASE_ADDR)
#define GPIOD_REG ((volatile gpioRegOffset_t*)GPIOD_BASE_ADDR)
#define GPIOE_REG ((volatile gpioRegOffset_t*)GPIOE_BASE_ADDR)
#define GPIOH_REG ((volatile gpioRegOffset_t*)GPIOH_BASE_ADDR)

#define UART1_REG ((volatile uartRegOffset_t*)UART1_BASE_ADDR)
#define UART2_REG ((volatile uartRegOffset_t*)UART2_BASE_ADDR)
#define UART6_REG ((volatile uartRegOffset_t*)UART6_BASE_ADDR)

#define TIM1_REG 	((volatile timerRegOffset_t*)TIM1_BASE_ADDR)
#define TIM2_REG 	((volatile timerRegOffset_t*)TIM2_BASE_ADDR)
#define TIM3_REG 	((volatile timerRegOffset_t*)TIM3_BASE_ADDR)
#define TIM4_REG 	((volatile timerRegOffset_t*)TIM4_BASE_ADDR)
#define TIM5_REG 	((volatile timerRegOffset_t*)TIM5_BASE_ADDR)
#define TIM9_REG 	((volatile timerRegOffset_t*)TIM9_BASE_ADDR)
#define TIM10_REG 	((volatile timerRegOffset_t*)TIM10_BASE_ADDR)
#define TIM11_REG 	((volatile timerRegOffset_t*)TIM11_BASE_ADDR)

#define DMA2_REG	((volatile dmaRegOffset_t*)DMA2_BASE_ADDR)
#define DMA1_REG	((volatile dmaRegOffset_t*)DMA1_BASE_ADDR)

#define FLASH_REG		((volatile flashRegOffset_t*)FLASH_BASE_ADDR)
#define RCC_REG 		((volatile rccRegOffset_t*)RCC_BASE_ADDR)
#define ADC1_REG		((volatile adcRegOffset_t*)ADC1_BASE_ADDR)
#define ADC1_COMMON_REG	((volatile adcCommonRegOffset_t*)ADC1_COMMON_BASE_ADDR)
#define	EXTI_REG		((volatile extiRegOffset_t*)EXTI_BASE_ADDR)
#define NVIC_REG		((volatile NVIC_t*)NVIC_BASE_ADDR)

#define SPI1_REG	((volatile spiRegOffset_t*)SPI1_BASE_ADDR)
#define SPI2_REG	((volatile spiRegOffset_t*)SPI2_BASE_ADDR)
#define SPI3_REG	((volatile spiRegOffset_t*)SPI3_BASE_ADDR)
#define SPI4_REG	((volatile spiRegOffset_t*)SPI4_BASE_ADDR)
#define SPI5_REG	((volatile spiRegOffset_t*)SPI5_BASE_ADDR)

/*
 * ----------------------------------------------------
 * Single-field Address Helpers
 * ----------------------------------------------------
 */
/*
 * GET_REG() helpers return the **address** (not the value) of a specific
 * register inside a peripheral structure. This is handy when a driver
 * needs a pointer to a register for DMA/bit-bading or when passing the
 * address to an inline assembly instruction
 */
#define GPIOA_GET_REG(regOffset) (&(GPIOA_REG -> regOffset))
#define GPIOB_GET_REG(regOffset) (&(GPIOB_REG -> regOffset))
#define GPIOC_GET_REG(regOffset) (&(GPIOC_REG -> regOffset))
#define GPIOD_GET_REG(regOffset) (&(GPIOD_REG -> regOffset))
#define GPIOE_GET_REG(regOffset) (&(GPIOE_REG -> regOffset))
#define GPIOH_GET_REG(regOffset) (&(GPIOH_REG -> regOffset))

#define UART1_GET_REG(regOffset) (&(UART1_REG -> regOffset))
#define UART2_GET_REG(regOffset) (&(UART2_REG -> regOffset))
#define UART6_GET_REG(regOffset) (&(UART6_REG -> regOffset))

#define TIM1_GET_REG(regOffset)  (&(TIM1_REG -> regOffset))
#define TIM2_GET_REG(regOffset)  (&(TIM2_REG -> regOffset))
#define TIM3_GET_REG(regOffset)  (&(TIM3_REG -> regOffset))
#define TIM4_GET_REG(regOffset)  (&(TIM4_REG -> regOffset))
#define TIM5_GET_REG(regOffset)  (&(TIM5_REG -> regOffset))
#define TIM9_GET_REG(regOffset)  (&(TIM9_REG -> regOffset))
#define TIM10_GET_REG(regOffset) (&(TIM10_REG -> regOffset))
#define TIM11_GET_REG(regOffset) (&(TIM11_REG -> regOffset))

#define DMA2_GET_REG(regOffset) (&(DMA2_REG -> regOffset))
#define DMA1_GET_REG(regOffset) (&(DMA1_REG -> regOffset))

#define FLASH_GET_REG(regOffset)		(&(FLASH_REG -> regOffset))
#define RCC_GET_REG(regOffset)			(&(RCC_REG -> regOffset))
#define ADC1_GET_REG(regOffset) 		(&(ADC1_REG -> regOffset))
#define ADC1_COMMON_GET_REG(regOffset) 	(&(ADC1_COMMON_REG -> regOffset))

#define SPI1_GET_REG(regOffset)	(&(SPI1_REG -> regOffset))
#define SPI2_GET_REG(regOffset) (&(SPI2_REG -> regOffset))
#define SPI3_GET_REG(regOffset) (&(SPI3_REG -> regOffset))
#define SPI4_GET_REG(regOffset) (&(SPI4_REG -> regOffset))
#define SPI5_GET_REG(regOffset) (&(SPI5_REG -> regOffset))

#endif /* INC_STM32PERIPHERALADDR_H_ */
