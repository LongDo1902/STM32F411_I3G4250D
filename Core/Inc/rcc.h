/*
 * @file	RCC.h
 * @brief	Reset and Clock Control (RCC) interface for STM32F411VET6
 *
 *  Created on: July 18, 2025
 *      Author: dobao
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

/*
 * -----------------------------------------
 * Standard & Device Headers
 * -----------------------------------------
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx.h"
#include "stm32PeripheralAddr.h"
#include "flash.h"


/*
 *------------------------------------------
 * Global Constants & Helper Macros
 *------------------------------------------
 */
#define HSERDY_TIMEOUT	0x2000U //Max polling loops while waiting for the HSE oscillator
#define PLLRDY_TIMEOUT	0x4000U //Max pollong loops while waiting for the main PLL
#define	SWS_TIMEOUT		0x4000U //Max polling loops while verifying SYSCLK switch

/*
 * ------------------------------------------
 * Enumeration
 * ------------------------------------------
 *
 * @enum	RCC_Name_t
 * @brief	Human-readable indices for RCC registers.
 *
 * These indices are used with ::writeRCC and ::readRCC helpers.
 * The final member, #RCC_REG_COUNT, always equals the number of real registers and used to size loopkup table
 */
typedef enum{
	/* Core Clock Registers */
	RCC_CR,			//RCC Clock Control Reg
	RCC_PLL_CFGR,	//RCC PLL Config Reg
	RCC_CFGR,		//RCC Clk Ctrl Reg
	RCC_CIR,		//RCC Clk Interrupt Reg

	/* Reset AHB and APB Registers */
	RCC_AHB1_RSTR,
	RCC_AHB2_RSTR,
	RCC_APB1_RSTR,
	RCC_APB2_RSTR,

	/* Clock Enable AHB and APB Registers */
	RCC_AHB1_ENR,
	RCC_AHB2_ENR,
	RCC_APB1_ENR,
	RCC_APB2_ENR,

	/* Low-power Clock Enable Registers */
	RCC_AHB1_LP_ENR,
	RCC_AHB2_LP_ENR,
	RCC_APB1_LP_ENR,
	RCC_APB2_LP_ENR,

	/* Backup and Misc Registers */
	RCC_BDCR,
	RCC_CSR,
	RCC_SSCGR,
	RCC_PLL_I2S_CFGR,
	RCC_DCK_CFGR,

	RCC_REG_COUNT	//Total num of RCC registers
}RCC_Mode_t;


/*
 * -------------------------------------------
 * RCC Core Helps
 * --------------------------------------------
 */

/*
 * @brief	Initialize clocks (HSE -> PLL -> 100MHz SYSCLK)
 */
void RCC_init(void);
void writeRCC(uint8_t bitPosition, RCC_Mode_t mode, uint32_t value);
uint32_t readRCC(uint8_t bitPosition, RCC_Mode_t mode);

/*
 * ----------------------------------------
 * Peripheral Clock Control - TIM
 * ----------------------------------------
 */
void my_RCC_TIM1_CLK_ENABLE();
void my_RCC_TIM1_CLK_DISABLE();

void my_RCC_TIM2_CLK_ENABLE();
void my_RCC_TIM2_CLK_DISABLE();

void my_RCC_TIM3_CLK_ENABLE();
void my_RCC_TIM3_CLK_DISABLE();

void my_RCC_TIM4_CLK_ENABLE();
void my_RCC_TIM4_CLK_DISABLE();

void my_RCC_TIM5_CLK_ENABLE();
void my_RCC_TIM5_CLK_DISABLE();

void my_RCC_TIM9_CLK_ENABLE();
void my_RCC_TIM9_CLK_DISABLE();

void my_RCC_TIM10_CLK_ENABLE();
void my_RCC_TIM10_CLK_DISABLE();

void my_RCC_TIM11_CLK_ENABLE();
void my_RCC_TIM11_CLK_DISABLE();


/*
 * ----------------------------------------
 * Peripheral Clock Control - GPIO
 * ----------------------------------------
 */
#define my_RCC_GPIOA_CLK_ENABLE()			\
	do{										\
		writeRCC(0, RCC_AHB1_ENR, SET);		\
		(void)readRCC(0, RCC_AHB1_ENR);		\
	    __DSB();							\
	} while(0)

#define my_RCC_GPIOA_CLK_DISABLE()			\
	do{										\
		writeRCC(0, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(0, RCC_AHB1_ENR);		\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_GPIOB_CLK_ENABLE()			\
	do{										\
		writeRCC(1, RCC_AHB1_ENR, SET);		\
		(void)readRCC(1, RCC_AHB1_ENR);		\
	} while(0)

#define my_RCC_GPIOB_CLK_DISABLE()			\
	do{										\
		writeRCC(1, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(1, RCC_AHB1_ENR);		\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_GPIOC_CLK_ENABLE()			\
	do{										\
		writeRCC(2, RCC_AHB1_ENR, SET);		\
		(void)readRCC(2, RCC_AHB1_ENR);		\
	} while(0)

#define my_RCC_GPIOC_CLK_DISABLE()			\
	do{										\
		writeRCC(2, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(2, RCC_AHB1_ENR);		\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_GPIOD_CLK_ENABLE()			\
	do{										\
		writeRCC(3, RCC_AHB1_ENR, SET);		\
		(void)readRCC(3, RCC_AHB1_ENR);		\
	} while(0)

#define my_RCC_GPIOD_CLK_DISABLE()			\
	do{										\
		writeRCC(3, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(3, RCC_AHB1_ENR);		\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_GPIOE_CLK_ENABLE()			\
	do{										\
		writeRCC(4, RCC_AHB1_ENR, SET);		\
		(void)readRCC(4, RCC_AHB1_ENR);		\
	    __DSB();							\
	} while(0)

#define my_RCC_GPIOE_CLK_DISABLE()			\
	do{										\
		writeRCC(4, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(4, RCC_AHB1_ENR);		\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_GPIOH_CLK_ENABLE()			\
	do{										\
		writeRCC(7, RCC_AHB1_ENR, SET);		\
		(void)readRCC(7, RCC_AHB1_ENR);		\
	} while(0)

#define my_RCC_GPIOH_CLK_DISABLE()			\
	do{										\
		writeRCC(7, RCC_AHB1_ENR, RESET);	\
		(void)readRCC(7, RCC_AHB1_ENR);		\
	} while(0)


/*
 * ----------------------------------------
 * Peripheral Clock Control - SPI
 * ----------------------------------------
 */
#define my_RCC_SPI1_CLK_ENABLE()			\
	do {									\
		writeRCC(12, RCC_APB2_ENR, SET);	\
		(void)readRCC(12, RCC_APB2_ENR);	\
	    __DSB();							\
	} while (0)

#define my_RCC_SPI1_CLK_DISABLE()			\
	do{										\
		writeRCC(12, RCC_APB2_ENR, RESET);	\
		(void)readRCC(12, RCC_APB2_ENR);	\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_SPI2_CLK_ENABLE()			\
	do {									\
		writeRCC(14, RCC_APB1_ENR, SET);	\
		(void)readRCC(14, RCC_APB1_ENR);	\
	} while (0)

#define my_RCC_SPI2_CLK_DISABLE()			\
	do{										\
		writeRCC(14, RCC_APB1_ENR, RESET);	\
		(void)readRCC(14, RCC_APB1_ENR);	\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_SPI3_CLK_ENABLE()			\
	do {									\
		writeRCC(15, RCC_APB1_ENR, SET);	\
		(void)readRCC(15, RCC_APB1_ENR);	\
	} while (0)

#define my_RCC_SPI3_CLK_DISABLE()			\
	do{										\
		writeRCC(15, RCC_APB1_ENR, RESET);	\
		(void)readRCC(15, RCC_APB1_ENR);	\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_SPI4_CLK_ENABLE()			\
	do {									\
		writeRCC(13, RCC_APB2_ENR, SET);	\
		(void)readRCC(13, RCC_APB2_ENR);	\
	} while (0)

#define my_RCC_SPI4_CLK_DISABLE()			\
	do{										\
		writeRCC(13, RCC_APB2_ENR, RESET);	\
		(void)readRCC(13, RCC_APB2_ENR);	\
	} while(0)

/*--------------------------------------------*/

#define my_RCC_SPI5_CLK_ENABLE()			\
	do {									\
		writeRCC(20, RCC_APB2_ENR, SET);	\
		(void)readRCC(20, RCC_APB2_ENR);	\
	} while (0)

#define my_RCC_SPI5_CLK_DISABLE()			\
	do{										\
		writeRCC(20, RCC_APB2_ENR, RESET);	\
		(void)readRCC(20, RCC_APB2_ENR);	\
	} while(0)

/*--------------------------------------------*/


/*
 * ----------------------------------------
 * Peripheral Clock Control - USART
 * ----------------------------------------
 */
void my_RCC_USART1_CLK_ENABLE();
void my_RCC_USART1_CLK_DISABLE();

void my_RCC_USART2_CLK_ENABLE();
void my_RCC_USART2_CLK_DISABLE();

void my_RCC_USART6_CLK_ENABLE();
void my_RCC_USART6_CLK_DISABLE();


/*
 * ----------------------------------------
 * Peripheral Clock Control - I2C
 * ----------------------------------------
 */
void my_RCC_I2C1_CLK_ENABLE();
void my_RCC_I2C1_CLK_DISABLE();

void my_RCC_I2C2_CLK_ENABLE();
void my_RCC_I2C2_CLK_DISABLE();

void my_RCC_I2C3_CLK_ENABLE();
void my_RCC_I2C3_CLK_DISABLE();

/*
 * ----------------------------------------
 * Peripheral Clock Control - ADC
 * ----------------------------------------
 */
void my_RCC_ADC1_CLK_ENABLE();
void my_RCC_ADC1_CLK_DISABLE();



#endif /* INC_RCC_H_ */
