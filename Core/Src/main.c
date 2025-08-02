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
#include "i3g4250d.h"

/* ------------------------------------------------------------------------------------ */
int main(void){
	HAL_Init();
	i3g4250d_init();

	while(1){
	}
}
