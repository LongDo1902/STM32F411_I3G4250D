/*
 * flash.h
 *
 *  Created on: July 18, 2025
 *      Author: dobao
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx.h"

#include "stm32PeripheralAddr.h"

#define RAMFUNC __attribute__((section(".RamFunc")))

typedef struct{
	volatile uint32_t* SECTOR0;
	volatile uint32_t* SECTOR1;
	volatile uint32_t* SECTOR2;
	volatile uint32_t* SECTOR3;
	volatile uint32_t* SECTOR4;
	volatile uint32_t* SECTOR5;
	volatile uint32_t* SECTOR6;
	volatile uint32_t* SECTOR7;
}sectorAddr_t;

typedef enum{
	FLASH_ACR,
	FLASH_KEYR,
	FLASH_OPTKEYR,
	FLASH_SR,
	FLASH_CR,
	FLASH_OPTCR,

	FLASH_REG_COUNT // == 6
}Flash_RegName_t;


/*
 * List of function declarations
 */
RAMFUNC void FLASH_Sector_Erase(uint8_t sector);
RAMFUNC void FLASH_Programming(volatile uint8_t* flashDest, uint8_t* programBuf, int bufSize);
RAMFUNC void firmwareUpdate(uint8_t* programBuf, int bufSize);

RAMFUNC void writeFLASH(uint8_t bitPosition, Flash_RegName_t mode, uint32_t value);
RAMFUNC uint32_t readFLASH(uint8_t bitPostion, Flash_RegName_t regName);

#endif /* INC_FLASH_H_ */
