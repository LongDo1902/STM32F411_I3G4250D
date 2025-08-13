/*
 * main.c
 *
 *  Created on: Jul 27, 2025
 *      Author: dobao
 */
#include <string.h>
#include <stdbool.h>

#include "stm32PeripheralAddr.h"
#include "stm32f4xx_hal.h"
#include "rcc.h"
#include "led.h"
#include "spi.h"
#include "uart.h"

const SPI_GPIO_Config_t spiConfig = {
		.SPIx = _SPI1,

		.sckPin = my_GPIO_PIN_5,
		.sckPort = my_GPIOA,

		.nssPin = my_GPIO_PIN_3,
		.nssPort = my_GPIOE,

		.mosiPin = my_GPIO_PIN_7,
		.mosiPort = my_GPIOA,

		.misoPin = my_GPIO_PIN_6,
		.misoPort = my_GPIOA
};

static bool i3g4250dFound = false;

/* ------------------------------------------------------------------------------------ */
int main(void){
	HAL_Init();
	ledGreenInit();
//	SPI_GPIO_init(&spiConfig);
//	SPI_basicConfigInit(&spiConfig, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV16, SOFTWARE_SLAVE_ENABLE);
//	uint8_t temp = SPI_readRegUnsigned(&spiConfig, 0x0F);
	if(i3g4250d_init()) i3g4250dFound = true;

	while(1){
		if(i3g4250dFound){
			ledControl(LED_GREEN, ON);
			HAL_Delay(200);
			ledControl(LED_GREEN, OFF);
			HAL_Delay(200);
		}
	}
}
