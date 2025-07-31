/*
 * adc.h
 *
 *  Created on: Jul 12, 2025
 *  Updated on: Jul 23, 2025
 *      Author: dobao
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "rcc.h"
#include "stm32PeripheralAddr.h"

typedef enum{
	ADC_SR,

	ADC_CR1,
	ADC_CR2,

	ADC_SMPR1,
	ADC_SMPR2,

	ADC_JOFR1,
	ADC_JOFR2,
	ADC_JOFR3,
	ADC_JOFR4,

	ADC_HTR,
	ADC_LTR,

	ADC_SQR1,
	ADC_SQR2,
	ADC_SQR3,

	ADC_JSQR,

	ADC_JDR1,
	ADC_JDR2,
	ADC_JDR3,
	ADC_JDR4,

	ADC_DR,
	ADC_CCR,

	ADC_REG_COUNT
}ADC_regName_t;

typedef enum{
	ADC_OK,
	ADC_ERROR
}ADC_statusFlag;


/*
 * Public API
 */
void writeADC(uint8_t bitPosition, ADC_regName_t regName, uint32_t value);
uint32_t readADC(uint8_t bitPosition, ADC_regName_t regName);

void ADC_temperatureSensorInit();
float temperatureSensorRead();

#endif /* INC_ADC_H_ */
