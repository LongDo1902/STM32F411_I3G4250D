/*
 * dma.c
 *
 *  Created on: Jul 18, 2025
 *  Updated on: Jul 23, 2025
 *  	Bug fixed
 *  	More efficient execution
 *      Author: dobao
 */

#include "dma.h"
/*
 * ----------------------------------------------------------------------
 * Private Helpers
 * ----------------------------------------------------------------------
 */
static volatile uint32_t* const dma2RegLookupTable[DMA_REG_COUNT] = {
		[DMA_LISR] 	= DMA2_GET_REG(DMA_LISR),
		[DMA_HISR] 	= DMA2_GET_REG(DMA_HISR),
		[DMA_LIFCR] = DMA2_GET_REG(DMA_LIFCR),
		[DMA_HIFCR] = DMA2_GET_REG(DMA_HIFCR),

		[DMA_S0CR] 		= DMA2_GET_REG(DMA_S0CR),
		[DMA_S0NDTR] 	= DMA2_GET_REG(DMA_S0NDTR),
		[DMA_S0PAR] 	= DMA2_GET_REG(DMA_S0PAR),
		[DMA_S0M0AR] 	= DMA2_GET_REG(DMA_S0M0AR),
		[DMA_S0M1AR] 	= DMA2_GET_REG(DMA_S0M1AR),
		[DMA_S0FCR] 	= DMA2_GET_REG(DMA_S0FCR),

		[DMA_S1CR] 		= DMA2_GET_REG(DMA_S1CR),
		[DMA_S1NDTR] 	= DMA2_GET_REG(DMA_S1NDTR),
		[DMA_S1PAR] 	= DMA2_GET_REG(DMA_S1PAR),
		[DMA_S1M0AR] 	= DMA2_GET_REG(DMA_S1M0AR),
		[DMA_S1M1AR] 	= DMA2_GET_REG(DMA_S1M1AR),
		[DMA_S1FCR] 	= DMA2_GET_REG(DMA_S1FCR),

		[DMA_S2CR] 		= DMA2_GET_REG(DMA_S2CR),
		[DMA_S2NDTR] 	= DMA2_GET_REG(DMA_S2NDTR),
		[DMA_S2PAR] 	= DMA2_GET_REG(DMA_S2PAR),
		[DMA_S2M0AR] 	= DMA2_GET_REG(DMA_S2M0AR),
		[DMA_S2M1AR] 	= DMA2_GET_REG(DMA_S2M1AR),
		[DMA_S2FCR] 	= DMA2_GET_REG(DMA_S2FCR),

		[DMA_S3CR] 		= DMA2_GET_REG(DMA_S3CR),
		[DMA_S3NDTR] 	= DMA2_GET_REG(DMA_S3NDTR),
		[DMA_S3PAR] 	= DMA2_GET_REG(DMA_S3PAR),
		[DMA_S3M0AR] 	= DMA2_GET_REG(DMA_S3M0AR),
		[DMA_S3M1AR] 	= DMA2_GET_REG(DMA_S3M1AR),
		[DMA_S3FCR] 	= DMA2_GET_REG(DMA_S3FCR),

		[DMA_S4CR] 		= DMA2_GET_REG(DMA_S4CR),
		[DMA_S4NDTR] 	= DMA2_GET_REG(DMA_S4NDTR),
		[DMA_S4PAR] 	= DMA2_GET_REG(DMA_S4PAR),
		[DMA_S4M0AR] 	= DMA2_GET_REG(DMA_S4M0AR),
		[DMA_S4M1AR] 	= DMA2_GET_REG(DMA_S4M1AR),
		[DMA_S4FCR] 	= DMA2_GET_REG(DMA_S4FCR),

		[DMA_S5CR] 		= DMA2_GET_REG(DMA_S5CR),
		[DMA_S5NDTR] 	= DMA2_GET_REG(DMA_S5NDTR),
		[DMA_S5PAR] 	= DMA2_GET_REG(DMA_S5PAR),
		[DMA_S5M0AR] 	= DMA2_GET_REG(DMA_S5M0AR),
		[DMA_S5M1AR] 	= DMA2_GET_REG(DMA_S5M1AR),
		[DMA_S5FCR] 	= DMA2_GET_REG(DMA_S5FCR),

		[DMA_S6CR] 		= DMA2_GET_REG(DMA_S6CR),
		[DMA_S6NDTR] 	= DMA2_GET_REG(DMA_S6NDTR),
		[DMA_S6PAR] 	= DMA2_GET_REG(DMA_S6PAR),
		[DMA_S6M0AR] 	= DMA2_GET_REG(DMA_S6M0AR),
		[DMA_S6M1AR] 	= DMA2_GET_REG(DMA_S6M1AR),
		[DMA_S6FCR] 	= DMA2_GET_REG(DMA_S6FCR),

		[DMA_S7CR] 		= DMA2_GET_REG(DMA_S7CR),
		[DMA_S7NDTR] 	= DMA2_GET_REG(DMA_S7NDTR),
		[DMA_S7PAR] 	= DMA2_GET_REG(DMA_S7PAR),
		[DMA_S7M0AR] 	= DMA2_GET_REG(DMA_S7M0AR),
		[DMA_S7M1AR] 	= DMA2_GET_REG(DMA_S7M1AR),
		[DMA_S7FCR] 	= DMA2_GET_REG(DMA_S7FCR),
};



static volatile uint32_t* const dma1RegLookupTable[DMA_REG_COUNT] = {
		[DMA_LISR] 	= DMA1_GET_REG(DMA_LISR),
		[DMA_HISR] 	= DMA1_GET_REG(DMA_HISR),
		[DMA_LIFCR] = DMA1_GET_REG(DMA_LIFCR),
		[DMA_HIFCR] = DMA1_GET_REG(DMA_HIFCR),

		[DMA_S0CR] 		= DMA1_GET_REG(DMA_S0CR),
		[DMA_S0NDTR] 	= DMA1_GET_REG(DMA_S0NDTR),
		[DMA_S0PAR] 	= DMA1_GET_REG(DMA_S0PAR),
		[DMA_S0M0AR] 	= DMA1_GET_REG(DMA_S0M0AR),
		[DMA_S0M1AR] 	= DMA1_GET_REG(DMA_S0M1AR),
		[DMA_S0FCR] 	= DMA1_GET_REG(DMA_S0FCR),

		[DMA_S1CR] 		= DMA1_GET_REG(DMA_S1CR),
		[DMA_S1NDTR] 	= DMA1_GET_REG(DMA_S1NDTR),
		[DMA_S1PAR] 	= DMA1_GET_REG(DMA_S1PAR),
		[DMA_S1M0AR] 	= DMA1_GET_REG(DMA_S1M0AR),
		[DMA_S1M1AR] 	= DMA1_GET_REG(DMA_S1M1AR),
		[DMA_S1FCR] 	= DMA1_GET_REG(DMA_S1FCR),

		[DMA_S2CR] 		= DMA1_GET_REG(DMA_S2CR),
		[DMA_S2NDTR] 	= DMA1_GET_REG(DMA_S2NDTR),
		[DMA_S2PAR] 	= DMA1_GET_REG(DMA_S2PAR),
		[DMA_S2M0AR] 	= DMA1_GET_REG(DMA_S2M0AR),
		[DMA_S2M1AR] 	= DMA1_GET_REG(DMA_S2M1AR),
		[DMA_S2FCR] 	= DMA1_GET_REG(DMA_S2FCR),

		[DMA_S3CR] 		= DMA1_GET_REG(DMA_S3CR),
		[DMA_S3NDTR] 	= DMA1_GET_REG(DMA_S3NDTR),
		[DMA_S3PAR] 	= DMA1_GET_REG(DMA_S3PAR),
		[DMA_S3M0AR] 	= DMA1_GET_REG(DMA_S3M0AR),
		[DMA_S3M1AR] 	= DMA1_GET_REG(DMA_S3M1AR),
		[DMA_S3FCR] 	= DMA1_GET_REG(DMA_S3FCR),

		[DMA_S4CR] 		= DMA1_GET_REG(DMA_S4CR),
		[DMA_S4NDTR] 	= DMA1_GET_REG(DMA_S4NDTR),
		[DMA_S4PAR] 	= DMA1_GET_REG(DMA_S4PAR),
		[DMA_S4M0AR] 	= DMA1_GET_REG(DMA_S4M0AR),
		[DMA_S4M1AR] 	= DMA1_GET_REG(DMA_S4M1AR),
		[DMA_S4FCR] 	= DMA1_GET_REG(DMA_S4FCR),

		[DMA_S5CR] 		= DMA1_GET_REG(DMA_S5CR),
		[DMA_S5NDTR] 	= DMA1_GET_REG(DMA_S5NDTR),
		[DMA_S5PAR] 	= DMA1_GET_REG(DMA_S5PAR),
		[DMA_S5M0AR] 	= DMA1_GET_REG(DMA_S5M0AR),
		[DMA_S5M1AR] 	= DMA1_GET_REG(DMA_S5M1AR),
		[DMA_S5FCR] 	= DMA1_GET_REG(DMA_S5FCR),

		[DMA_S6CR] 		= DMA1_GET_REG(DMA_S6CR),
		[DMA_S6NDTR] 	= DMA1_GET_REG(DMA_S6NDTR),
		[DMA_S6PAR] 	= DMA1_GET_REG(DMA_S6PAR),
		[DMA_S6M0AR] 	= DMA1_GET_REG(DMA_S6M0AR),
		[DMA_S6M1AR] 	= DMA1_GET_REG(DMA_S6M1AR),
		[DMA_S6FCR] 	= DMA1_GET_REG(DMA_S6FCR),

		[DMA_S7CR] 		= DMA1_GET_REG(DMA_S7CR),
		[DMA_S7NDTR] 	= DMA1_GET_REG(DMA_S7NDTR),
		[DMA_S7PAR] 	= DMA1_GET_REG(DMA_S7PAR),
		[DMA_S7M0AR] 	= DMA1_GET_REG(DMA_S7M0AR),
		[DMA_S7M1AR] 	= DMA1_GET_REG(DMA_S7M1AR),
		[DMA_S7FCR] 	= DMA1_GET_REG(DMA_S7FCR),
};



/*
 * @brief	Generic masked write helper
 * @param	reg		Pointers to the register
 * @param	bitPosition		First bit of the field
 * @param	bitWidth		Field width in bits
 * @param	value			Field value (must fit in @p bitWidth)
 */
static void writeDMABits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	if(bitWidth > 32) return; //Prevent undefined behavior: 32-bit shift is invalid on 32-bit targets
	if((bitWidth + bitPosition) > 32) return; //Prevent writing beyong 32-bit reg boundary

	/* Reject out of range value */
	if(bitWidth < 32){
		uint32_t maxVal = (1U << bitWidth) - 1U;
		if(value > maxVal) return;
	}

	if(bitWidth == 32 && bitPosition == 0){
		*reg = value;
		return;
	}

	/* Other general cases */
	//Mask off the old bit and or with new bit
	uint32_t mask = ((1U << bitWidth) - 1U) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition) & mask;
	*reg = (*reg & ~mask) | shiftedValue;
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
static inline uint32_t readDMABits(const volatile uint32_t* reg,
								   uint8_t bitPosition,
								   uint8_t bitWidth){
	uint32_t ERROR = 0xFFFFFFFFU;

	/* Sanity Check */
	if(bitWidth == 0 || bitWidth > 32)	return ERROR;
	if(bitPosition > 31)				return ERROR;
	if(bitPosition + bitWidth > 32)		return ERROR;

	if(bitWidth == 32 && bitPosition == 0){
		return *reg; //Full-word; no mask needed
	}

	uint32_t mask = (1U << bitWidth) - 1U;
	return (*reg >> bitPosition) & mask;
}



/*
 * @brief	Write a bit-field inside an DMAx register
 *
 * 			This function checks if the bit position and mode are valid
 * 			then writes a bitfield to the corresponding DMAx reg without
 * 			affecting other bits.
 *
 * 			It uses a lookup table to get the DMAx register address, and ensures
 * 			safe bit manipulation even for multi-bit values.
 *
 * @param	bitPosition		Starting bit (0-31)
 * @param	regName			Target register (see ::DMA_RegName_t)
 * @param	value			New field value.
 */
void writeDMA2(uint8_t bitPosition, DMA_RegName_t regName, uint32_t value){
	uint8_t bitWidth = 1;

	switch(regName){
		case DMA_LISR:
			break;

		case DMA_HISR:
			break;

		case DMA_LIFCR:
			break;

		case DMA_HIFCR:
			break;

		case DMA_S0CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S0NDTR:
			bitWidth = 16;
			break;

		case DMA_S0PAR:
			bitWidth = 32;
			break;

		case DMA_S0M0AR:
			bitWidth = 32;
			break;

		case DMA_S0M1AR:
			bitWidth = 32;
			break;

		case DMA_S0FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S1CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S1NDTR:
			bitWidth = 16;
			break;

		case DMA_S1PAR:
			bitWidth = 32;
			break;

		case DMA_S1M0AR:
			bitWidth = 32;
			break;

		case DMA_S1M1AR:
			bitWidth = 32;
			break;

		case DMA_S1FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S2CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S2NDTR:
			bitWidth = 16;
			break;

		case DMA_S2PAR:
			bitWidth = 32;
			break;

		case DMA_S2M0AR:
			bitWidth = 32;
			break;

		case DMA_S2M1AR:
			bitWidth = 32;
			break;

		case DMA_S2FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S3CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S3NDTR:
			bitWidth = 16;
			break;

		case DMA_S3PAR:
			bitWidth = 32;
			break;

		case DMA_S3M0AR:
			bitWidth = 32;
			break;

		case DMA_S3M1AR:
			bitWidth = 32;
			break;

		case DMA_S3FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S7CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S7NDTR:
			bitWidth = 16;
			break;

		case DMA_S7PAR:
			bitWidth = 32;
			break;

		case DMA_S7M0AR:
			bitWidth = 32;
			break;

		case DMA_S7M1AR:
			bitWidth = 32;
			break;

		case DMA_S7FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//...

		default: return;
	}

	volatile uint32_t* reg = dma2RegLookupTable[regName];
	writeDMABits(reg, bitPosition, bitWidth, value);
}



/*
 * @brief	Read a bit-field from an DMA2 peripheral register
 *
 * @param	bitPosition		The LSB index of the field (0-31)
 * @param	mode			Which register of DMA2 to access (enum @ref DMA_regName_t)
 *
 * @param	value			The value to write. There is a helper function to check if
 * 							@p value fits into the target field width; if it does not,
 * 							the call ignore
 *
 * @return	The extracted field value on success.
 * 			If the cal is invalid, the constant @c 0xFFFFFFFF is returned as an ERROR
 */
uint32_t readDMA2(uint8_t bitPosition, DMA_RegName_t regName){
	uint32_t const ERROR = 0xFFFFFFFF;
	uint8_t bitWidth = 1;

	switch(regName){
		case DMA_LISR:
			break;

		case DMA_HISR:
			break;

		case DMA_LIFCR:
			break;

		case DMA_HIFCR:
			break;

		case DMA_S0CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S0NDTR:
			bitWidth = 16;
			break;

		case DMA_S0PAR:
			bitWidth = 32;
			break;

		case DMA_S0M0AR:
			bitWidth = 32;
			break;

		case DMA_S0M1AR:
			bitWidth = 32;
			break;

		case DMA_S0FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//

		case DMA_S1CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S1NDTR:
			bitWidth = 16;
			break;

		case DMA_S1PAR:
			bitWidth = 32;
			break;

		case DMA_S1M0AR:
			bitWidth = 32;
			break;

		case DMA_S1M1AR:
			bitWidth = 32;
			break;

		case DMA_S1FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//

		case DMA_S2CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S2NDTR:
			bitWidth = 16;
			break;

		case DMA_S2PAR:
			bitWidth = 32;
			break;

		case DMA_S2M0AR:
			bitWidth = 32;
			break;

		case DMA_S2M1AR:
			bitWidth = 32;
			break;

		case DMA_S2FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S3CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S3NDTR:
			bitWidth = 16;
			break;

		case DMA_S3PAR:
			bitWidth = 32;
			break;

		case DMA_S3M0AR:
			bitWidth = 32;
			break;

		case DMA_S3M1AR:
			bitWidth = 32;
			break;

		case DMA_S3FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;

		//------------------------------------------------//
		case DMA_S7CR:
			if(bitPosition == 6 || bitPosition == 11 ||
			   bitPosition == 13 || bitPosition == 16 ||
			   bitPosition == 21 || bitPosition == 23){
				bitWidth = 2;
			}
			else if(bitPosition == 25){
				bitWidth = 3;
			}
			break;

		case DMA_S7NDTR:
			bitWidth = 16;
			break;

		case DMA_S7PAR:
			bitWidth = 32;
			break;

		case DMA_S7M0AR:
			bitWidth = 32;
			break;

		case DMA_S7M1AR:
			bitWidth = 32;
			break;

		case DMA_S7FCR:
			if(bitPosition == 0) bitWidth = 2;
			else if(bitPosition == 3) bitWidth = 3;
			break;
		//...

		default: return ERROR;
	}

	volatile uint32_t* reg = dma2RegLookupTable[regName];
	return readDMABits(reg, bitPosition, bitWidth);
}




