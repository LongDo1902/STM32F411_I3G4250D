/*
 * flash.c
 *
 *  Created on: July 18, 2025
 *  Updated on: Jul 25, 2025
 *  	Improved efficiency
 *      Author: dobao
 */


#include "flash.h"

__attribute__ ((section(".RamConst")))const sectorAddr_t flashSectors = {
		(volatile uint32_t*) 0x08000000U,
		(volatile uint32_t*) 0x08004000U,
		(volatile uint32_t*) 0x08008000U,
		(volatile uint32_t*) 0x0800C000U,
		(volatile uint32_t*) 0x08010000U,
		(volatile uint32_t*) 0x08020000U,
		(volatile uint32_t*) 0x08040000U,
		(volatile uint32_t*) 0x08060000U
};

/*
 * @brief	Initialize a lookup table for readFlash() and writeFlash() to access any Flash register generically via an enum index
 * 			instead osf hard-coding addresses each time
 *
 * @note	'const keeps the table in Flash'; registers themselve stay 'volatile' because the hardware can change them anytime
 */
__attribute__((section(".RamConst")))volatile uint32_t* const flashRegLookupTable[FLASH_REG_COUNT] = {
		[FLASH_ACR] 	= FLASH_GET_REG(FLASH_ACR),
		[FLASH_KEYR] 	= FLASH_GET_REG(FLASH_KEYR),
		[FLASH_OPTKEYR]	= FLASH_GET_REG(FLASH_OPTKEYR),
		[FLASH_SR]		= FLASH_GET_REG(FLASH_SR),
		[FLASH_CR]		= FLASH_GET_REG(FLASH_CR),
		[FLASH_OPTCR]	= FLASH_GET_REG(FLASH_OPTCR)
};



/*
 * @brief	A lookup table of **valid-bit masks** for every FLASH register
 * 			The array is indexed by 'FLASH_REG_COUNT'
 */
__attribute__((section(".RamConst")))const uint32_t FLASH_VALID_BIT[FLASH_REG_COUNT] = {
		[FLASH_ACR] = ~((0xFu << 4) | 0xFFFFE000u), //These bits are reserved
		[FLASH_KEYR] = 0xFFFFFFFFu,
		[FLASH_OPTKEYR] = 0xFFFFFFFFu,
		[FLASH_SR] = ~((0x3u << 2) | (0x7Fu << 9) | 0x7FFFu << 17),
		[FLASH_CR] = ~((1u << 7) | (0x3Fu << 10) | (0x7Fu << 17) | (0x1Fu << 26)),
		[FLASH_OPTCR] = ~((1u << 4) | (0x7F << 24)),
};



/*
 * @brief	Check if a specific bit position in a Flash reg is valid (not reserved)
 *
 * @param	bitPosition		Bit index (0-31) to check within the given Flash reg
 * @param	mode			Enum value representing flash register being access
 *
 * @retval	true			Bit is valid
 * 			false			Bit is reserved or inputs are out of range
 */
RAMFUNC static inline bool isValidFlashBit(uint8_t bitPosition, Flash_RegName_t mode){
	return ((mode < FLASH_REG_COUNT && bitPosition < 32) && (FLASH_VALID_BIT[mode] >> bitPosition) & 0x01);
}


RAMFUNC static void writeFLASHBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	if((bitWidth > 32) || (bitWidth + bitPosition) > 32) return;

	/* Reject out of range value	 */
	if((bitWidth < 32) && (value > ((1U << bitWidth) - 1U))) return;

	if((bitPosition == 0) && (bitWidth == 32)) {
		*reg = value;
		return;
	}

	/* Other general cases */
	uint32_t mask = ((1U << bitWidth) - 1U) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition) & mask;
	*reg = (*reg & ~mask) | shiftedValue;
}


RAMFUNC static uint32_t readFLASHBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	uint32_t ERROR = 0xFFFFFFFFU;

	/* Sanity Check */
	if((bitWidth == 0) || (bitWidth > 32) ||
	   (bitPosition > 31) || (bitPosition + bitWidth) > 32) return ERROR;

	if((bitWidth == 32) && (bitPosition == 0)){
		return *reg;
	}

	uint32_t mask = (1U << bitWidth) - 1U;
	return (*reg >> bitPosition)  & mask;
}


/*
 * --------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------
 */

/*
 * @brief
 *
 * @param
 */
RAMFUNC void FLASH_Sector_Erase(uint8_t sector){
	while((readFLASH(16, FLASH_SR) & 1) == 1); //Wait until there is no Flash memory operation in progress
	if((readFLASH(31, FLASH_CR) & 1) == 1){ //Check if the FLASH is locked
		/* Unlock the FLASH by writing these sequences */
		writeFLASH(0, FLASH_KEYR, 0x45670123); //Sequence 1
		writeFLASH(0, FLASH_KEYR, 0xCDEF89AB); //Sequence 2
	}
	writeFLASH(1, FLASH_CR, SET); //Sector Erase Activated
	writeFLASH(3, FLASH_CR, sector); //Select sector that want to be deleted
	writeFLASH(16, FLASH_CR, SET); //Start
	while((readFLASH(16, FLASH_SR) & 1) == 1); //Wait for BUSY bit to be cleared
}



/*
 * @brief
 *
 * @param
 * @param
 * @param
 */
RAMFUNC void FLASH_Programming(volatile uint8_t* flashDest, uint8_t* programBuf, int bufSize){
	while((readFLASH(16, FLASH_SR) & 1) == 1); //Wait untio there is no Flash memory operation in progress
	if((readFLASH(31, FLASH_CR) & 1) == 1){ //Check if the FLASH is locked
		/* Unlock the FLASH by writing these sequences */
		writeFLASH(0, FLASH_KEYR, 0x45670123); //Sequence 1
		writeFLASH(0, FLASH_KEYR, 0xCDEF89AB); //Sequence 2
	}
	writeFLASH(0, FLASH_CR, SET); //Activate FLASH programming
	writeFLASH(8, FLASH_CR, 0b00); //8-bits write
	for(int i = 0; i < bufSize; i++){
		flashDest[i] = programBuf[i];
		while(readFLASH(16, FLASH_SR) & 1); //Wait for BUSY bit to be cleared
	}
	writeFLASH(0, FLASH_CR, RESET); //Deactivate programming
	writeFLASH(1, FLASH_CR, RESET); //Deactivate sector erase
	writeFLASH(31, FLASH_CR, SET); //Lock FLASH
}


/*
 * @brief
 *
 * @param
 * @param
 * @param
 */
RAMFUNC void firmwareUpdate(uint8_t* programBuf, int bufSize){
	/* Erase sector 0 */
	if(bufSize < 16000){
		FLASH_Sector_Erase(0);
	}
	else if ((bufSize >= 16000) && (bufSize < 32000)){
		for(uint8_t s = 0; s < 2; s++) FLASH_Sector_Erase(s); //Erase sec0 and sec1
	}
	else if ((bufSize >= 32000) && (bufSize < 48000)){
		for(uint8_t s = 0; s < 3; s++) FLASH_Sector_Erase(s); //Erase sec0 to 2
	}
	else if ((bufSize >= 48000) && (bufSize < 64000)){
		for(uint8_t s = 0; s < 4; s++) FLASH_Sector_Erase(s); //Erase sec0 to 3
	}
	// ...
	FLASH_Programming((volatile uint8_t*)flashSectors.SECTOR0, programBuf, bufSize);

	/*
	 * Activate NVIC_System_Reset(), check cotexM4
	 */
	volatile uint32_t* AIRCR = (volatile uint32_t*) 0xE000ED0CU;
	*AIRCR |= (0x5FA << 16) | (1 << 2);
}


__attribute__((section(".RamFunc"))) uint32_t readFLASH(uint8_t bitPosition, Flash_RegName_t regName){
	uint32_t ERROR = 0xFFFFFFFFU;
	uint8_t bitWidth = 1;

	switch(regName){
		case FLASH_ACR:
			if(bitPosition == 0) bitWidth = 4;
			break;

		case FLASH_KEYR:
			bitWidth = 32;
			break;

		case FLASH_OPTKEYR:
			bitWidth = 32;
			break;

		case FLASH_SR:
			break;

		case FLASH_CR:
			if(bitPosition == 3) bitWidth = 4;
			else if(bitPosition == 8) bitWidth = 2;
			break;

		case FLASH_OPTCR:
			if(bitPosition == 2) bitWidth = 2;
			else if(bitPosition == 8) bitWidth = 8;
			else if(bitPosition == 16) bitWidth = 8;
			break;

		default: return ERROR;
	}

	volatile uint32_t* reg = flashRegLookupTable[regName];
	return readFLASHBits(reg, bitPosition, bitWidth);
}


/*
 * @brief	Safely write a value to a bit-field inside one of the Flash reg
 *
 * The helper:
 * 		Rejects invalid bit position / reserved bits
 * 		Calculates how many bits are needed to store @p value
 *
 * @param	bitPosition		The starting bit position (0-31) to write the value
 * @param	mode			Which Flash register to access (see Flash_Name_t)
 * @param	value			New value to write into the field.
 */
RAMFUNC void writeFLASH(uint8_t bitPosition, Flash_RegName_t regName, uint32_t value){
	/* Sanity Checks*/
	if(bitPosition > 31) return;
	if(!isValidFlashBit(bitPosition, regName)) return;

	uint8_t bitWidth = 1;

	switch(regName){
		case FLASH_ACR:
			if(bitPosition == 0) bitWidth = 4;
			break;

		case FLASH_KEYR:
			bitWidth = 32;
			break;

		case FLASH_OPTKEYR:
			bitWidth = 32;
			break;

		case FLASH_SR:
			break;

		case FLASH_CR:
			if(bitPosition == 3) bitWidth = 4;
			else if(bitPosition == 8) bitWidth = 2;
			break;

		case FLASH_OPTCR:
			if(bitPosition == 2) bitWidth = 2;
			else if(bitPosition == 8) bitWidth = 8;
			else if(bitPosition == 16) bitWidth = 8;
			break;

		default: return;
	}

	volatile uint32_t* reg = flashRegLookupTable[regName];

	writeFLASHBits(reg, bitPosition, bitWidth, value);
}


