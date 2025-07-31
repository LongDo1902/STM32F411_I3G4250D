/*
 * @file	timer.h
 * @brief	Basic TIMER utilities for STM32F411VET6 (TIM1 - TIM11)
 * 			Provide PSC/ARR calculations and safe register access helpers.
 *
 *  Created on: July 18, 2025
 *      Author: dobao
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

/*
 * -----------------------------------------------
 * Standard & Device Headers
 * -----------------------------------------------
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32PeripheralAddr.h"
#include "exti.h"
#include "stm32f4xx.h"
#include "rcc.h"

/*
 * --------------------------------------------------
 * Clock Parameters Definitions
 * --------------------------------------------------
 */
#define SYSCLK_FREQ_16M		16000000UL	//16MHz SYSCLK Freq
#define SYSCLK_FREQ_100M	100000000UL 	//100MHz SYSCLK Freq

#define _1MS_PER_TICK	0.001	//1msec per tick
#define _5MS_PER_TICK	0.005	//5msec per tick
#define _1S_PER_TICK	1		//1sec per tick

#define TICK_FREQ_1000Hz	1/_1MS_PER_TICK //1ms per period
#define TICK_FREQ_200Hz		1/_5MS_PER_TICK //5ms per period


/*
 * ---------------------------------------------------
 * Enumerations
 * ---------------------------------------------------
 */

/*
 * @enum	TIM_Name_t
 * @brief	Logical indices for the timers we use
 */
typedef enum{
	my_TIM1, //0
	my_TIM2,
	my_TIM3,
	my_TIM4,
	my_TIM5,
	my_TIM9,
	my_TIM10,
	my_TIM11 //10
}TIM_Name_t;

/*
 * @enum	TIM_Mode_t
 * @brief	Individual timer registers
 */
typedef enum{
	TIM_CR1,
	TIM_CR2,
	TIM_SMCR,
	TIM_DIER,
	TIM_SR,

	TIM_EGR,
	TIM_CCMR1,
	TIM_CCMR2,
	TIM_CCER,
	TIM_CNT,

	TIM_PSC,
	TIM_ARR,
	TIM_RCR,

	TIM_CCR1,
	TIM_CCR2,
	TIM_CCR3,
	TIM_CCR4,

	TIM_BDTR,

	TIM_DCR,
	TIM_DMAR,

	TIM2_OR,
	TIM5_OR,
	TIM11_OR
}TIM_RegName_t;

/*
 * ------------------------------------------------------
 * PSC/ARR Calculation Helper
 * ------------------------------------------------------
 */

/*
 * @struct	TIM_Cal_t
 * @brief	Result of timer prescaler/auto-reload computation
 */
typedef struct{
	uint32_t psc;	//Prescaler value
	uint32_t arr;	//Auto-reload Value
	uint32_t actualHz;	//Achieved frequency
	uint32_t error; //An error status
}TIM_Cal_t;


/*
 * Timer Bit Status
 */
typedef enum{
	TIM_INVALID_BIT,
	TIM_VALID_BIT,
	TIM_INVALID_REG
}TIM_isValidBit_t;


/*
 * --------------------------------------------------------
 * Public API
 * --------------------------------------------------------
 */
void initTimer(TIM_Name_t userTIMx);
void delay(int msec);

void TIM1_UP_TIM10_IRQHandler();

void writeTimer(uint8_t bitPosition, TIM_Name_t userTIMx, TIM_RegName_t mode, uint32_t value);

uint32_t readTimer (uint8_t bitPosiion, TIM_Name_t userTIMx, TIM_RegName_t mode);

#endif /* INC_TIMER_H_ */
