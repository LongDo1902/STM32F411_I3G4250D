/*
 * adc.c
 *
 *  Created on: Jul 6, 2025
 *  Updated on: Jul 23, 2025
 *      Author: dobao
 */

#include "adc.h"


/*
 * -----------------------------------------------------------------
 * Private Helpers
 * -----------------------------------------------------------------
 */

/*
 * @brief	Bit-mask of **writable** bits for every ADC reg
 * 			A clear bit (0) marks a *reserved* position that **must not** be written.
 *
 * 			Index:  ::ADC_regName_t
 */
static const uint32_t ADC_VALID_BITS[ADC_REG_COUNT] = {
		[ADC_SR] = ~(0x3FFFFFF << 6),
		[ADC_CR1] = ~((0x1F << 27) | (0x3F << 16)),
		[ADC_CR2] = ~((0x3F << 2) | (0xF << 12) | (0x1 << 23) | (0x1 << 31)),

		[ADC_SMPR1] = ~(0x1F << 27),
		[ADC_SMPR2] = ~(0b11 << 30),

		[ADC_JOFR1] = ~(0xFFFFF << 12),
		[ADC_JOFR2] = ~(0xFFFFF << 12),
		[ADC_JOFR3] = ~(0xFFFFF << 12),
		[ADC_JOFR4] = ~(0xFFFFF << 12),

		[ADC_HTR] = ~(0xFFFFF << 12),
		[ADC_LTR] = ~(0xFFFFF << 12),

		[ADC_SQR1] = ~(0xFF << 24),
		[ADC_SQR2] = ~(0b11 << 30),
		[ADC_SQR3] = ~(0b11 << 30),

		[ADC_JSQR] = ~(0x3FF << 22),

		[ADC_JDR1] = ~(0xFFFF << 16),
		[ADC_JDR2] = ~(0xFFFF << 16),
		[ADC_JDR3] = ~(0xFFFF << 16),
		[ADC_JDR4] = ~(0xFFFF << 16),

		[ADC_DR] = ~(0xFFFF << 16),
		[ADC_CCR] = ~((0xFFFF << 0) | (0xF << 18) | (0xFF << 24)),
};

/*
 * @brief	Lookup table for the ADC1 peripheral register
 *
 * 			This static pointer array maps each value of ::ADC_regName_t (index) to memory-mapped address
 * 			of the corresponding ADC Register. Using the table avoids open-coded 'switch'/'if' blocks
 * 			and makes register access simply
 *
 * 			**ADC_REG_COUNT**	Array Length
 */
#define REG_TABLE_ATTR static volatile uint32_t* const

REG_TABLE_ATTR ADCRegLookupTable[ADC_REG_COUNT] = {
		[ADC_SR] = 	ADC1_GET_REG(ADC_SR),
		[ADC_CR1] = ADC1_GET_REG(ADC_CR1),
		[ADC_CR2] = ADC1_GET_REG(ADC_CR2),

		[ADC_SMPR1] = ADC1_GET_REG(ADC_SMPR1),
		[ADC_SMPR2] = ADC1_GET_REG(ADC_SMPR2),

		[ADC_JOFR1] = ADC1_GET_REG(ADC_JOFR1),
		[ADC_JOFR2] = ADC1_GET_REG(ADC_JOFR2),
		[ADC_JOFR3] = ADC1_GET_REG(ADC_JOFR3),
		[ADC_JOFR4] = ADC1_GET_REG(ADC_JOFR4),

		[ADC_HTR] = ADC1_GET_REG(ADC_HTR),
		[ADC_LTR] = ADC1_GET_REG(ADC_LTR),

		[ADC_SQR1] = ADC1_GET_REG(ADC_SQR1),
		[ADC_SQR2] = ADC1_GET_REG(ADC_SQR2),
		[ADC_SQR3] = ADC1_GET_REG(ADC_SQR3),

		[ADC_JSQR] = ADC1_GET_REG(ADC_JSQR),

		[ADC_JDR1] = ADC1_GET_REG(ADC_JDR1),
		[ADC_JDR2] = ADC1_GET_REG(ADC_JDR2),
		[ADC_JDR3] = ADC1_GET_REG(ADC_JDR3),
		[ADC_JDR4] = ADC1_GET_REG(ADC_JDR4),

		[ADC_DR] = ADC1_GET_REG(ADC_DR),
		[ADC_CCR] = ADC1_COMMON_GET_REG(ADC_CCR),
};

/*
 * -----------------------------------------------------------------
 * Bit-manipulation Helpers
 * -----------------------------------------------------------------
 */

/*
 * @brief	Check that 'mode' is in range and 'bitPosition' is not reserved
 * 			This function flexibly check if multi-bits are valid
 *
 * @return	true	Inserted bit indice is valid
 * 			false	...invalid or reserved bit
 */
static inline bool isValidADCBit(uint8_t bitPosition, uint8_t bitWidth, ADC_regName_t regName){
	if(regName >= ADC_REG_COUNT || bitWidth == 0 || ((bitPosition + bitWidth) > 32)) return false;
	uint32_t mask = ((bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1U) << bitPosition);

	return (ADC_VALID_BITS[regName] & mask) == mask;
}

/*
 * @brief	Generic masked write helper
 *
 * @param	reg				Pointer points to the register
 * @param	bitPosition		First bit of the field
 * @param	bitWidth		Field width in bits
 * @param	value			Field value (must fit in @p bitWidth)
 */
static void writeADCBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	/*
	 * The function leaves the register unchanged if
	 * 		@p bitPosition larger than 31 because shifting by 32 is undefined in C
	 * 		@p value is too large for the field
	 * 		The field would spill past bit 31
	 */
	if(bitPosition > 31 || bitWidth > 32) return;
	if(bitWidth < 32 && value >= (1U << bitWidth)) return;
	if((bitWidth + bitPosition) > 32) return;

	//Mask off the old bit and OR with the new value
	uint32_t mask = (bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition);
	*reg = (*reg & ~mask) | (shiftedValue & mask);
}

/*
 * @brief	Read a field of "bitWidth" bits from a register starting at 'bitPosition'
 *
 * @param	reg			 	Pointer points to the register
 * @param	bitPosition		Starting bit position (0-31)
 * @param	bitWidth		Number of bits/bit size that fit @p value
 */
static uint32_t readADCBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	if(bitWidth == 32) return *reg; //Full-word: no mask needed

	uint32_t mask = ((1U << bitWidth) - 1U);
	return (*reg >> bitPosition) & mask;
}


/*
 * --------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------
 */

/*
 * @brief	Initialize temperature sensor
 */
void ADC_temperatureSensorInit(){
	my_RCC_ADC1_CLK_ENABLE();
	/*
	 * ADC Clock supports 36MHz max (datasheet)
	 * However, APB2 Clock is customized with 50MHz which is larger than ADC Clock
	 * Therefore, we set prescaler of 2 in register ADC_CCR to feed a correct frequency to ADC Clock that is below 36MHz
	 * 		50MHz / 2 = 25MHz < 36MHz
	 */
	writeADC(16, ADC_CCR, 0b00); //Set prescaler of 2

	/*
	 * T_adcCycle = 1/25MHz = 40ns (It costs 40ns to complete one ADC clock cycle)
	 * Datasheet says min sampling time when reading the temperature sensor is 10us
	 *
	 * Therefore, 10us/40ns = 250 cycles
	 * 		Choose 0b111: 480 cyles in ADC_SMPR1 for channel 16
	 */
	writeADC(18, ADC_SMPR1, 0b111); //480 cycles
	writeADC(20, ADC_JSQR, 0b00); //Choose 1 conversion by writing 0b00 to JL[1:0]
	writeADC(15, ADC_JSQR, 16); //Select JSQ4[4:1] start at bit 15
	writeADC(23, ADC_CCR, SET); //Enable temperature sensor
	writeADC(0, ADC_CR2, SET); //Enable ADC
}

/*
 * @brief
 */
float temperatureSensorRead(){
	writeADC(22, ADC_CR2, SET); //JSWSTART starts conversion of injected channels
	while((readADC(2, ADC_SR) & 1u) == 0u); //Wait until injected channe conversion is completed
	writeADC(2, ADC_SR, RESET); //Clear the bit since the ref manual says "cleared by software"

	uint16_t adcTemperatureRaw = readADC(0, ADC_JDR1);
	float v_in = (adcTemperatureRaw * 3.0) / 4095;
	float temperature = ((v_in - 0.76) / 0.0025) + 25;
	return temperature;
}

/*
 * @brief	Write a bit-field to an ADC1 peripheral register
 *
 * 			Figures out how many bits the setting needs
 * 			Won't touch ay bits the datasheet says are off-limits.
 * 			Changes only the bits you asked for, leaving the rest unchanged.
 *
 * @param	bitPosition		The LSB index of the field (0-31)
 * @param	mode			Which register of ADC to access (enum @ref ADC_regName_t)
 *
 * @param	value			The value to write. There is a helper function to check if
 * 							@p value fits into the target field width; if it does not,
 * 							the call ignore
 */
void writeADC(uint8_t bitPosition, ADC_regName_t regName, uint32_t value){
	//Early sanity
	uint8_t bitWidth = 1;

	switch(regName){
		case ADC_SR:
			break;

		case ADC_CR1:
			if(bitPosition == 0) bitWidth = 5;
			else if(bitPosition == 13) bitWidth = 3;
			else if(bitPosition == 24) bitWidth = 2;
			break;

		case ADC_CR2:
			if(bitPosition == 16 || bitPosition == 24) bitWidth = 4;
			else if(bitPosition == 20 || bitPosition == 28) bitWidth = 2;
			break;

		case ADC_SMPR1:
			bitWidth = 3;
			break;

		case ADC_SMPR2:
			bitWidth = 3;
			break;

		case ADC_JOFR1:
			bitWidth = 12;
			break;

		case ADC_JOFR2:
			bitWidth = 12;
			break;

		case ADC_JOFR3:
			bitWidth = 12;
			break;

		case ADC_JOFR4:
			bitWidth = 12;
			break;

		case ADC_HTR:
			bitWidth = 12;
			break;

		case ADC_LTR:
			bitWidth = 12;
			break;

		case ADC_SQR1:
			if(bitPosition == 0 || bitPosition == 5 || bitPosition == 10 || bitPosition == 15) bitWidth = 5;
			else if(bitPosition == 20) bitWidth = 4;
			break;

		case ADC_SQR2:
			bitWidth = 5;
			break;

		case ADC_SQR3:
			bitWidth = 5;
			break;

		case ADC_JSQR:
			if(bitPosition == 0 || bitPosition == 5 || bitPosition == 10 || bitPosition == 15) bitWidth = 5;
			else if(bitPosition == 20) bitWidth = 2;
			break;

		case ADC_JDR1:
			bitWidth = 16;
			break;

		case ADC_JDR2:
			bitWidth = 16;
			break;

		case ADC_JDR3:
			bitWidth = 16;
			break;

		case ADC_JDR4:
			bitWidth = 16;
			break;

		case ADC_DR:
			bitWidth = 16;
			break;

		case ADC_CCR:
			if(bitPosition == 16) bitWidth = 2;
			break;

		default: return;
	}

	if(!isValidADCBit(bitPosition, bitWidth, regName)) return;

	volatile uint32_t* reg = ADCRegLookupTable[regName];

	writeADCBits(reg, bitPosition, bitWidth, value);
}


/*
 * @brief	Read a bit-field from an ADC1 peripheral register
 *
 * @param	bitPosition		The LSB index of the field (0-31)
 * @param	mode			Which register of ADC to access (enum @ref ADC_regName_t)
 *
 * @param	value			The value to write. There is a helper function to check if
 * 							@p value fits into the target field width; if it does not,
 * 							the call ignore
 *
 * @return	The extracted field value on success.
 * 			If the cal is invalid, the constant @c 0xFFFFFFFF is returned as an ERROR_FLAG
 */
uint32_t readADC(uint8_t bitPosition, ADC_regName_t regName){
	uint32_t const ERROR_FLAG = 0xFFFFFFFF;
	uint8_t bitWidth = 1;

	switch(regName){
		case ADC_SR:
			break;

		case ADC_CR1:
			if(bitPosition == 0) bitWidth = 5;
			else if(bitPosition == 13) bitWidth = 3;
			else if(bitPosition == 24) bitWidth = 2;
			break;

		case ADC_CR2:
			if(bitPosition == 16 || bitPosition == 24) bitWidth = 4;
			else if(bitPosition == 20 || bitPosition == 28) bitWidth = 2;
			break;

		case ADC_SMPR1:
			bitWidth = 3;
			break;

		case ADC_SMPR2:
			bitWidth = 3;
			break;

		case ADC_JOFR1:
			bitWidth = 12;
			break;

		case ADC_JOFR2:
			bitWidth = 12;
			break;

		case ADC_JOFR3:
			bitWidth = 12;
			break;

		case ADC_JOFR4:
			bitWidth = 12;
			break;

		case ADC_HTR:
			bitWidth = 12;
			break;

		case ADC_LTR:
			bitWidth = 12;
			break;

		case ADC_SQR1:
			if(bitPosition == 0 || bitPosition == 5 || bitPosition == 10 || bitPosition == 15) bitWidth = 5;
			else if(bitPosition == 20) bitWidth = 4;
			break;

		case ADC_SQR2:
			bitWidth = 5;
			break;

		case ADC_SQR3:
			bitWidth = 5;
			break;

		case ADC_JSQR:
			if(bitPosition == 0 || bitPosition == 5 || bitPosition == 10 || bitPosition == 15) bitWidth = 5;
			else if(bitPosition == 20) bitWidth = 2;
			break;

		case ADC_JDR1:
			bitWidth = 16;
			break;

		case ADC_JDR2:
			bitWidth = 16;
			break;

		case ADC_JDR3:
			bitWidth = 16;
			break;

		case ADC_JDR4:
			bitWidth = 16;
			break;

		case ADC_DR:
			bitWidth = 16;
			break;

		case ADC_CCR:
			if(bitPosition == 16) bitWidth = 2;
			break;

		default: return ERROR_FLAG;
	}

	if(!isValidADCBit(bitPosition, bitWidth, regName)) return ERROR_FLAG;

	volatile uint32_t* reg = ADCRegLookupTable[regName];
	return readADCBits(reg, bitPosition, bitWidth);
}




