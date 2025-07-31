/*
 * dma.h
 *
 *  Created on: Jul 18, 2025
 *      Author: dobao
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include <stdio.h>
#include <stdint.h>

#include "stm32PeripheralAddr.h"

typedef enum {
	DMA_LISR,
	DMA_HISR,
	DMA_LIFCR,
	DMA_HIFCR,

	DMA_S0CR,
	DMA_S0NDTR,
	DMA_S0PAR,
	DMA_S0M0AR,
	DMA_S0M1AR,
	DMA_S0FCR,

	DMA_S1CR,
	DMA_S1NDTR,
	DMA_S1PAR,
	DMA_S1M0AR,
	DMA_S1M1AR,
	DMA_S1FCR,

	DMA_S2CR,
	DMA_S2NDTR,
	DMA_S2PAR,
	DMA_S2M0AR,
	DMA_S2M1AR,
	DMA_S2FCR,

	DMA_S3CR,
	DMA_S3NDTR,
	DMA_S3PAR,
	DMA_S3M0AR,
	DMA_S3M1AR,
	DMA_S3FCR,

	DMA_S4CR,
	DMA_S4NDTR,
	DMA_S4PAR,
	DMA_S4M0AR,
	DMA_S4M1AR,
	DMA_S4FCR,

	DMA_S5CR,
	DMA_S5NDTR,
	DMA_S5PAR,
	DMA_S5M0AR,
	DMA_S5M1AR,
	DMA_S5FCR,

	DMA_S6CR,
	DMA_S6NDTR,
	DMA_S6PAR,
	DMA_S6M0AR,
	DMA_S6M1AR,
	DMA_S6FCR,

	DMA_S7CR,
	DMA_S7NDTR,
	DMA_S7PAR,
	DMA_S7M0AR,
	DMA_S7M1AR,
	DMA_S7FCR,

	DMA_REG_COUNT
}DMA_RegName_t;

/*
 * Function Declarations
 */
void writeDMA2(uint8_t bitPosition, DMA_RegName_t regName, uint32_t value);
uint32_t readDMA2(uint8_t bitPosition, DMA_RegName_t regName);

#endif /* INC_DMA_H_ */
