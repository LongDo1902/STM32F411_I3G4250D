/*
 * main.c
 *
 *  Created on: Jul 27, 2025
 *      Author: dobao
 */
#include <string.h>
#include <stdbool.h>

#include "stm32PeripheralAddr.h"
#include "rcc.h"
#include "led.h"
#include "spi.h"

/* ------------------------------------------------------------------------------------ */
int main(void){
	RCC_init();

	ledBlueInit();
	ledOrangeInit();
	ledRedInit();
	ledGreenInit();

	SPI_GPIO_Config_t spiConfig = {
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

	SPI_GPIO_init(spiConfig);
	SPI_basicConfigInit(spiConfig, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV128, SOFTWARE_SLAVE_ENABLE, SPI_ENABLE);
	char spiRead1 = SPI_readReceivedData(spiConfig, 0x0F);
	char spiRead2 = SPI_readReceivedData(spiConfig, 0x20);

	while(1){
	}
}
