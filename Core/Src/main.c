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
	i3g4250d_calibrate(500); // biasX = -4, biasY = -8, biasZ = -5

	while(1){
		i3g4250d_updateAngle();
	}
}
