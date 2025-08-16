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
#include "i3g4250d.h"


static float roll = 0;
static float pitch = 0;
static float yaw = 0;
/* ------------------------------------------------------------------------------------ */
int main(void){
	HAL_Init();
	i3g4250d_init();
	i3g4250d_route_LPF();
	i3g4250d_calibrate(300);

	while(1){
		i3g4250d_getAngle(&roll, &pitch, &yaw);
	}
}
