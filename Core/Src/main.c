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

static float rollAngle = 0;
static float pitchAngle = 0;
static float yawAngle = 0;

static float rollAngleFiltered = 0;
static float pitchAngleFiltered = 0;
static float yawAngleFiltered = 0;

float sampleRate = 200; //Hz

float rawRollBuf[400];
float filteredRollBuf[400];

#define arraySize(a)	(sizeof(a)/sizeof((a)[0]))

/* ------------------------------------------------------------------------------------ */
int main(void){
	HAL_Init();
	UART_Init(my_GPIO_PIN_6,//TX pin
			my_GPIO_PIN_7, 	//RX pin
			my_GPIOB, 		//GPIOB
			my_UART1, 		//UART1
			9600, 			//Baud rate
			PARITY_ODD,
			_9B_WORDLENGTH);
	UART1_DMA_Transmitter_Init();

	if(i3g4250d_init());
	i3g4250d_LPF_enable();
	i3g4250d_calibrate(1200);
	i3g4250d_softLPF_config(6, sampleRate); //2Hz more smooth but more lag, 10Hz more responsive but more wiggle

	for(int i = 0; i < arraySize(rawRollBuf); i++){
		i3g4250d_getAngle(&rollAngle, &pitchAngle, &yawAngle, sampleRate);
		i3g4250d_getAngleFiltered(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
		rawRollBuf[i] = rollAngle;
		filteredRollBuf[i] = rollAngleFiltered;
	}

	i3g4250d_sendRollToUART(&rawRollBuf, &filteredRollBuf, arraySize(rawRollBuf));

	while(1){
	}
}
