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

//static float angularX = 0;
//static float angularY = 0;
//static float angularZ = 0;

static float rollAngle = 0;
static float pitchAngle = 0;
static float yawAngle = 0;

float sampleRate = 200; //Hz

/* ------------------------------------------------------------------------------------ */
int main(void){
	HAL_Init();
	ledGreenInit();
	if(i3g4250d_init());
	i3g4250d_LPF_enable();
	i3g4250d_calibrate(500);
	i3g4250d_softLPF_config(2, sampleRate); //2Hz more smooth but more lag, 10Hz more responsive but more wiggle

	while(1){
//		i3g4250d_angularVelocityFiltered(&angularX, &angularY, &angularZ);
//		i3g4250d_angularVelocity(&angularX, &angularY, &angularZ);
		i3g4250d_getAngle(&rollAngle, &pitchAngle, &yawAngle, sampleRate);
	}
}
