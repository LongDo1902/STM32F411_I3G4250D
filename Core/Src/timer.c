/*
 * @file	timer.c
 * @brief	Timer utilities  for STM32F4 (TIM1 to TIM11)
 * 			Dynamic PSC/ARR calculation to hit target update rates
 * 			Safe bit-field access with validity checks
 * 			1kHz system tick using TIM1 + IRQ 25(TIM1_UP_TIM10)
 *
 *  Created on: Jun 20, 2025
 *  Updated on: Jul 18, 2025
 *  	Improve readability
 *
 *      Author: dobao
 */

#include "timer.h"

/*
 * ------------------------------------------------------------
 * Globals
 * ------------------------------------------------------------
 */
static volatile int timeCnt = 0; //Millisecond counter


/*
 * ------------------------------------------------------------
 * Validation Helper
 * ------------------------------------------------------------
 */

/*
 * @brief	check if a bit position in a specific TIMER's register is valid
 *
 * @param	bitPosition		Bit location (index) to check (0-31)
 * @param	userTIMx 		my_TIM1 to my_TIM11 (timer instance)
 * @param	mode			Target timer register (e.g., TIM_CR1, TIM_CR2, etc.)
 *
 * @return	true if the bit position is valid for the given timer and register, else returns false
 */
static inline TIM_isValidBit_t isValidTimerBit(uint8_t bitPosition, TIM_Name_t userTIMx, TIM_RegName_t mode){
	switch(mode){
		/*
		 * TIMx_CR1:
		 * 		TIM1 to TIM5: valid bits are [0:9]
		 * 		TIM9 to TIM11: valid bits are [0:9] excluding [4:6]
		 */
		case TIM_CR1:
			if(((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition < 10) ||
			   ((userTIMx >= my_TIM9 && userTIMx <= my_TIM11) && bitPosition < 10 && !(bitPosition >= 4 && bitPosition <= 6))){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CR2:
		 * 		TIM1: reserved bits at 1 and 15
		 * 		TIM2 to TIM5: valid bits at [3:7]
		 * 		TIM9 to TIM11 has no CR2
		 */
		case TIM_CR2:
			if((userTIMx == my_TIM1 && bitPosition != 1 && bitPosition != 15) ||
					   ((userTIMx >= my_TIM2 && userTIMx <= my_TIM5) && (bitPosition >= 3 && bitPosition <= 7))){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_SMCR
		 * 		TIM1 to TIM5: reserved bits at 3
		 * 		TIM9: reserved at bits 3 and [8:15]
		 * 		TIM10 to TIM11 does not have TIMx_SMCR
		 */
		case TIM_SMCR:
			if(((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition != 3) ||
					(userTIMx == my_TIM9 && bitPosition < 8 && bitPosition != 3)){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_DIER
		 * 		TIM1: reserved bit at 15
		 * 		TIM2 to TIM5: reserved bits at 15, 13, 7, 5
		 * 		TIM9: reserved bits within [3:5] and [7:15]
		 * 		TIM10 to TIM11: valid bits at 0 and 1
		 */
		case TIM_DIER:
			if((userTIMx == my_TIM1 && bitPosition != 15) ||
			  ((userTIMx >= my_TIM2 && userTIMx <= my_TIM5) && bitPosition != 15 && bitPosition != 13 && bitPosition != 7 && bitPosition != 5) ||
			   (userTIMx == my_TIM9 && !(bitPosition >= 3 && bitPosition <= 5) && !(bitPosition >= 7 && bitPosition <= 15)) ||
			  ((userTIMx == my_TIM10 || userTIMx == my_TIM11) && (bitPosition == 0 || bitPosition == 1))){
				return TIM_VALID_BIT;
			} return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_SR
		 * 		TIM1: reserved bits at [13:15] and 8
		 * 		TIM2 to TIM5: reserved bits at [13:15], [7:8], and 5
		 * 		TIM9: reserved bits at [11:15], [7:8] and [3:5]
		 * 		TIM10 to TIM11: valid bits at 0, 1, 9
		 */
		case TIM_SR:
			if((userTIMx == my_TIM1 && !(bitPosition >= 13 && bitPosition <= 15) && bitPosition != 8) ||
			  ((userTIMx >= my_TIM2 && userTIMx <= my_TIM5) && !(bitPosition >= 13 && bitPosition <= 15) && bitPosition != 8 && bitPosition != 7 && bitPosition != 5) ||
			   (userTIMx == my_TIM9 && !(bitPosition >= 11 && bitPosition <= 15) && bitPosition != 8 && bitPosition != 7 && !(bitPosition >= 3 && bitPosition <= 5)) ||
			  ((userTIMx == my_TIM10 || userTIMx == my_TIM11) && (bitPosition == 9 || bitPosition == 1 || bitPosition == 0))){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_EGR
		 * 		TIM1: valid bits [0:7]
		 * 		TIM2 to TIM5: valid bits [0:6] excluding bit 5
		 * 		TIM9: valid bits at 6 and [0:2]
		 * 		TIM10 to TIM11: valid bits at [0:1]
		 */
		case TIM_EGR:
			if((userTIMx == my_TIM1 && (bitPosition >= 0 && bitPosition <= 7)) ||
			  ((userTIMx >= my_TIM2 && userTIMx <= my_TIM5) && (bitPosition >= 0 && bitPosition <= 6) && bitPosition != 5) ||
			   (userTIMx == my_TIM9 && (bitPosition == 6 || (bitPosition >= 0 && bitPosition <= 2))) ||
			  ((userTIMx == my_TIM10 || userTIMx == my_TIM11) && (bitPosition == 1 || bitPosition == 0))){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT ;
		break;

		/*
		 * TIMx_CCER
		 * 		TIM1: valid bits [0:13]
		 * 		TIM2 to TIM5: reserved bits at 14, 10, 6, 2
		 * 		TIM9: reserved bits at [8:15], 6, 2
		 * 		TIM10 to TIM11: valid bits at 3, 1, 0
		 */
		case TIM_CCER:
			if((userTIMx == my_TIM1 && (bitPosition >= 0 && bitPosition <= 13)) ||
			  ((userTIMx >= my_TIM2 && userTIMx <= my_TIM5) && bitPosition != 14 && bitPosition != 10 && bitPosition != 6 && bitPosition != 2) ||
			   (userTIMx == my_TIM9 && bitPosition < 8 && bitPosition != 6 && bitPosition != 2) ||
			  ((userTIMx == my_TIM10 || userTIMx == my_TIM11) && (bitPosition == 3 || bitPosition == 1 || bitPosition == 0))){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CNT
		 */
		case TIM_CNT:
			if(bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_PSC
		 */
		case TIM_PSC:
			if(bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_ARR
		 */
		case TIM_ARR:
			if(bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_RCR
		 * 		Only available in TIM1
		 * 		TIM1: valid bits [0:7]
		 */
		case TIM_RCR:
			if(userTIMx == my_TIM1 && bitPosition <= 7){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CCR1 is available for all timers
		 */
		case TIM_CCR1:
			if(bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CCR2 is not available on TIM10 and TIM11
		 */
		case TIM_CCR2:
			if((userTIMx >= my_TIM1 && userTIMx <= my_TIM9) && bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CCR3 is only available on TIM1 to TIM5
		 */
		case TIM_CCR3:
			if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_CCR4 is only available on TIM1 to TIM5
		 */
		case TIM_CCR4:
			if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_BDTR is only available on TIM1
		 */
		case TIM_BDTR:
			if(userTIMx == my_TIM1){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_DCR
		 * 		Only available on TIM1 to TIM5
		 * 		All have reserved bits in [5:7] and [13:15]
		 */
		case TIM_DCR:
			if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && !(bitPosition >= 5 && bitPosition <= 7) && !(bitPosition >= 13 && bitPosition <= 15)){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIMx_DMAR
		 * 		Only available in TIM1 to TIM5
		 */
		case TIM_DMAR:
			if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIM2_OR
		 * 		TIM2: valid bits at 11 and 10
		 */
		case TIM2_OR:
			if(userTIMx == my_TIM2 && bitPosition == 10){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIM5_OR
		 * 		TIM5: valid bits at 7 and 6
		 */
		case TIM5_OR:
			if(userTIMx == my_TIM5 && bitPosition == 6){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		/*
		 * TIM11_OR
		 * 		TIM11: valid bits at 1 and 0
		 */
		case TIM11_OR:
			if(userTIMx == my_TIM11 && bitPosition == 0){
				return TIM_VALID_BIT;
			} else return TIM_INVALID_BIT;
		break;

		default: return TIM_INVALID_REG;
	}
}


/*
 * @brief	Read a field of 'bitWidth' bits from a register starting at 'bitPosition'
 *
 * @param	reg (pointer) to the register
 * @param	bitPosition		Starting bit position (0-31)
 * @param	bitWidth		Number of bits needed to read (1 to 32)
 *
 * @return	Extracted value
 */
uint32_t readBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	if(bitWidth == 32){
		return (*reg >> bitPosition); //Full-word; no mask needed
	}
	uint32_t mask = (1U << bitWidth) - 1U;
	return (*reg >> bitPosition) & mask;
}


/*
 * @brief	Writes a bit field into a specific TIM register at a given position,
 * 			only if the register and bit are valid for the selected TIM peripheral
 *
 * @param	bitPosition		The starting bit position in the target register
 * @param	userTIMx		The TIM peripheral (e.g., my_TIM1, my_TIM5, etc.)
 * @param	mode			The specific register being written (e.g., TIM_CR1, TIM_EGR, etc.)
 * @param	value			The value to write into the bit field
 */
void writeTimer(uint8_t bitPosition, TIM_Name_t userTIMx, TIM_RegName_t mode, uint32_t value){
	if(bitPosition > 31) return;

	//Due to the nature of CCMR register, which has input (capture mode) and output (compare mode)
	//Same bits but different mode(input/output) will have different functions
	if(mode == TIM_CCMR1 || mode == TIM_CCMR2){
		printf("Use writeCCMR() instead.\n");
		return;
	}

	volatile timerRegOffset_t* TIMx_p; //Timer Pointer

	switch(userTIMx){
		case my_TIM1: TIMx_p = TIM1_REG; break;
		case my_TIM2: TIMx_p = TIM2_REG; break;
		case my_TIM3: TIMx_p = TIM3_REG; break;
		case my_TIM4: TIMx_p = TIM4_REG; break;
		case my_TIM5: TIMx_p = TIM5_REG; break;
		case my_TIM9: TIMx_p = TIM9_REG; break;
		case my_TIM10: TIMx_p = TIM10_REG; break;
		case my_TIM11: TIMx_p = TIM11_REG; break;
		default: return; //Invalid TIMER
	}

	volatile uint32_t* reg;
	switch(mode){

		case TIM_CR1:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CR1) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CR1;
			} else return;
			break;


		case TIM_CR2:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CR2) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CR2;
			} else return;
			break;


		case TIM_SMCR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_SMCR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_SMCR;
			} else return;
			break;


		case TIM_DIER:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DIER) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DIER;
			} else return;
			break;


		case TIM_SR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_SR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_SR;
			} else return;
			break;


		case TIM_EGR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_EGR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_EGR;
			} else return;
			break;


		case TIM_CCER:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCER) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCER;
			} else return;
			break;


		case TIM_CNT:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CNT) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CNT;
			} else return;
			break;


		case TIM_PSC:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_PSC) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_PSC;
			} else return;
			break;


		case TIM_ARR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_ARR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_ARR;
			} else return;
			break;


		case TIM_RCR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_RCR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_RCR;
			} else return;
			break;


		case TIM_CCR1:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR1) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR1;
			} else return;
			break;


		case TIM_CCR2:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR2) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR2;
			} else return;
			break;


		case TIM_CCR3:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR3) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR3;
			} else return;
			break;


		case TIM_CCR4:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR4) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR4;
			} else return;
			break;


		case TIM_BDTR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_BDTR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_BDTR;
			} else return;
			break;


		case TIM_DCR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DCR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DCR;
			} else return;
			break;


		case TIM_DMAR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DMAR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DMAR;
			} else return;
			break;


		case TIM2_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM2_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM2_OR;
			} else return;
			break;


		case TIM5_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM5_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM5_OR;
			} else return;
			break;


		case TIM11_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM11_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM11_OR;
			} else return;
			break;

		default: return;
	}

	//Auto detect bitWidth based on the "value" length
	uint32_t bitWidth = 0;
	uint32_t temp = value;

	while(temp > 0){
		bitWidth++;
		temp = temp >> 1;
	}

	if(value == 0 && bitWidth == 0) bitWidth = 1;
	if(bitWidth == 32 && bitPosition != 0) return; //C does not allow shifting more than or equal to the size of the typess

	/*
	 * ARR has a default reset value of 0xFFFF
	 * Since it's typically written as a full 16-bit value (not bit-masked) -> write directly
	 */
	if(mode == TIM_ARR){
		*reg = value;
		return;
	}

	//Mask off the old bit and OR with new bit
	uint32_t mask = ((1U << bitWidth) - 1U) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition) & mask;
	*reg = (*reg & ~mask) | shiftedValue;
}


/*
 * @brief	Dynamically read bit(s) from a specific register of a selected TIM peripheral
 *
 * @param	bitPosition		Bit offset within the register (0-31)
 * @param	userTIMx		Selected timer (my_TIM1 to my_TIM11)
 * @param	mode			Register to read from
 *
 * @retval	The value of the selected bit field
 * @retval	0xFFFFFFFF (ERROR_FLAG) if invalid bit/register/timer combo
 */
uint32_t readTimer(uint8_t bitPosition, TIM_Name_t userTIMx, TIM_RegName_t mode){
	const uint32_t ERROR_FLAG = 0xFFFFFFFF;

	if(bitPosition > 31) return ERROR_FLAG;

	//Due to the nature of CCMR register, which has input (capture mode) and output (compare mode)
	//Same bits but different mode(input/output) will have different functions
	if(mode == TIM_CCMR1 || mode == TIM_CCMR2){
		printf("Use writeCCMR() instead.\n");
		return ERROR_FLAG;
	}

	volatile uint32_t* reg;
	volatile timerRegOffset_t* TIMx_p;

	switch(userTIMx){
		case my_TIM1: TIMx_p = TIM1_REG; break;
		case my_TIM2: TIMx_p = TIM2_REG; break;
		case my_TIM3: TIMx_p = TIM3_REG; break;
		case my_TIM4: TIMx_p = TIM4_REG; break;
		case my_TIM5: TIMx_p = TIM5_REG; break;
		case my_TIM9: TIMx_p = TIM9_REG; break;
		case my_TIM10: TIMx_p = TIM10_REG; break;
		case my_TIM11: TIMx_p = TIM11_REG; break;
		default: return ERROR_FLAG;
	}

	uint8_t bitWidth = 1; //Default 1-bit fields

	switch(mode){

		case TIM_CR1:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CR1) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CR1;
				/*
				 * TIM1 to TIM5:
				 * 		CMS[1:0] at bit 5
				 * 		CKD[1:0] at bit 8
				 *
				 * TIM9 to TIM11:
				 * 		CKD[1:0] at 8: bit 8 to 9 -> 2 bits
				 */

				if(((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && (bitPosition == 8 || bitPosition == 5)) ||
				   ((userTIMx >= my_TIM9 && userTIMx <= my_TIM11) && bitPosition == 8)){
					bitWidth = 2;
				}

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_CR2:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CR2) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CR2;
				/*
				 * TIM1 to TIM5:
				 * 		MMS[2:0] at bit 4
				 */
				if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 4){
					bitWidth = 3;
				}

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_SMCR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_SMCR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_SMCR;
				/*
				 * TIM1 to TIM5
				 * 		SMS[2:0] at bit 0
				 * 		TS[2:0] at bit 4
				 * 		ETF[3:0] at bit 8
				 * 		ETPS[1:0] at bit 12
				 * 		else 1 bit
				 * TIM9
				 * 		SMS[2:0] at bit 0
				 * 		TS[2:0] at bit 4
				 * 		else 1 bit
				 */
				if((userTIMx >= my_TIM1 && userTIMx <= my_TIM9) && (bitPosition == 0 || bitPosition == 4)){
					bitWidth = 3; //read 3 bits
				}
				else if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 8){
					bitWidth = 4; //read 4 bits
				}
				else if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && bitPosition == 12){
					bitWidth = 2; //read 2 bits
				}

				return readBits(reg, bitPosition, bitWidth); //read 1 bit
			} else return ERROR_FLAG;
			break;


		case TIM_DIER:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DIER) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DIER;
				/*
				 * TIM1 to TIM11 have 1 bit read
				 */
				return readBits(reg, bitPosition, bitWidth); //read 1 bit
			} else return ERROR_FLAG;
			break;


		case TIM_SR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_SR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_SR;
				/*
				 * TIM1 to TIM11 have 1 bit read
				 */
				return readBits(reg, bitPosition, bitWidth); //read 1 bit
			} else return ERROR_FLAG;
			break;


		case TIM_EGR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_EGR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_EGR;
				/*
				 * TIM1 to TIM11 have 1-bit read
				 */
				return readBits(reg, bitPosition, bitWidth); //read 1 bit
			} else return ERROR_FLAG;
			break;


		case TIM_CCER:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCER) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCER;
				/*
				 * TIM1 to TIM11 have 1-bit read
				 */
				return readBits(reg, bitPosition, bitWidth); //read 1 bit
			} else return ERROR_FLAG;
			break;


		case TIM_CNT:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CNT) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CNT;
				/*
				 * TIM1, TIM3, TIM4, TIM9, TIM10 and TIM11: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4 ||
				   userTIMx == my_TIM9 || userTIMx == my_TIM10 || userTIMx == my_TIM11){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_PSC:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_PSC) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_PSC;
				/*
				 * TIM1 to TIM11 have 16-bit read
				 */
				bitWidth = 16;
				return readBits(reg, bitPosition, bitWidth); //read 16 bits
			} else return ERROR_FLAG;
			break;


		case TIM_ARR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_ARR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_ARR;
				/*
				 * TIM1, TIM3, TIM4, TIM9, TIM10 and TIM11: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4 ||
				   userTIMx == my_TIM9 || userTIMx == my_TIM10 || userTIMx == my_TIM11){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_CCR1:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR1) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR1;
				/*
				 * TIM1, TIM3, TIM4, TIM9, TIM10 and TIM11: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4 ||
				   userTIMx == my_TIM9 || userTIMx == my_TIM10 || userTIMx == my_TIM11){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_CCR2:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR2) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR2;
				/*
				 * TIM1, TIM3, TIM4: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_CCR3:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR3) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR3;
				/*
				 * TIM1, TIM3, TIM4: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_CCR4:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_CCR4) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_CCR4;
				/*
				 * TIM1, TIM3, TIM4: 16-bits field
				 * TIM2 and TIM5: 32-bits field
				 */
				if(userTIMx == my_TIM1 || userTIMx == my_TIM3 || userTIMx == my_TIM4){
					bitWidth = 16;
				}
				else bitWidth = 32; //TIM2 and TIM5

				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_BDTR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_BDTR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_BDTR;
				//Only available on TIM1
				if(bitPosition == 0){
					bitWidth = 8; //DTG[7:0]
				}
				else if (bitPosition == 8){
					bitWidth = 2; //LOCK[1:0]
				}
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_DCR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DCR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DCR;
				/*
				 * TIM1 to TIM5
				 * 		DBA[4:0] at bit 0
				 * 		DBL[4:0] at bit 8
				 */
				if((userTIMx >= my_TIM1 && userTIMx <= my_TIM5) && (bitPosition == 0 || bitPosition == 8)){
					bitWidth = 5;
				}
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM_DMAR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM_DMAR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM_DMAR;
				/*
				 * TIM1: 32-bits field
				 * TIM2 to TIM5: 16-bits field
				 */
				if(userTIMx == my_TIM1){
					bitWidth = 32;
				}
				else bitWidth = 16; //TIM2 to TIM5
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM2_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM2_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM2_OR;
				bitWidth = 2; //ITR1_RMP [1:0] at bit 10
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM5_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM5_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM5_OR;
				bitWidth = 2; //TI4_RMP at bit 6
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		case TIM11_OR:
			if(isValidTimerBit(bitPosition, userTIMx, TIM11_OR) == TIM_VALID_BIT){
				reg = &TIMx_p -> TIM11_OR;
				bitWidth = 2; //TI1_RMP at bit 0
				return readBits(reg, bitPosition, bitWidth);
			} else return ERROR_FLAG;
			break;


		default: return ERROR_FLAG; //Invalid mode
	}
}


/*
 * --------------------------------------------------------
 * Prescaler/ARR Calculation
 * --------------------------------------------------------
 */

/*
 * @brief	Compute the smallest PSC then matching ARR for a target update frequency.
 *
 * @param	sysClkFreq	Timer input clock (Hz)
 * @param	targetHz	Desired update rate (Hz)
 * @param	maxArr		Max ARR value (0xFFFF for 16-bit, 0xFFFFFFFF for 32-bit)
 *
 * @return	Filled ::TIM_Cal_t with psc, arr, actualHz
 */
TIM_Cal_t timerCalculation(uint32_t sysClkFreq, uint32_t targetHz, uint32_t maxArr){
	TIM_Cal_t output = {0};
	TIM_Cal_t error;

	uint32_t psc = (sysClkFreq / (targetHz * (maxArr + 1)));
	if(psc > 0xFFFF) psc = 0xFFFF; //PSC is 16-bit on stm32

	for(;; ++psc){
		uint32_t arr = (sysClkFreq/(targetHz * (psc + 1))) - 1;

		if(arr <= maxArr){
			output.psc = psc;
			output.arr = arr;
			output.actualHz = sysClkFreq / ((psc + 1) * (arr + 1));
			return output;
		}
	}
	return error;
}


/*
 * -----------------------------------------------------
 * Public API
 * -----------------------------------------------------
 */
void initTimer(TIM_Name_t userTIMx){
	TIM_Cal_t timConfig;

	switch(userTIMx){
		case my_TIM1: my_RCC_TIM1_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;

		case my_TIM2: my_RCC_TIM2_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFFFFFF);
			break;

		case my_TIM3: my_RCC_TIM3_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;

		case my_TIM4: my_RCC_TIM4_CLK_ENABLE();	timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;

		case my_TIM5: my_RCC_TIM5_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFFFFFF);
			break;

		case my_TIM9: my_RCC_TIM9_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;

		case my_TIM10: my_RCC_TIM10_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;

		case my_TIM11: my_RCC_TIM11_CLK_ENABLE(); timConfig = timerCalculation(SYSCLK_FREQ_100M, TICK_FREQ_1000Hz, 0xFFFF);
			break;
		default: return;
	}

	writeTimer(0, userTIMx, TIM_PSC, timConfig.psc);
	writeTimer(0, userTIMx, TIM_ARR, timConfig.arr);
	writeTimer(0, userTIMx, TIM_DIER, SET); //DMA Interrupt Enable
	NVIC_enableIRQ(TIM1_UP_TIM10); //Enable interrupt at IRQ 25
	writeTimer(0, userTIMx, TIM_CR1, SET); //Counter enabled
}


void TIM1_UP_TIM10_IRQHandler(){
	timeCnt++;
	writeTimer(0, my_TIM1, TIM_SR, RESET); //Clear the interrupt flag
}

void delay(int msec){
	timeCnt = 0;
	while(timeCnt < msec); //Busy wait
}

























