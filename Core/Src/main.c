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

float rawRollBuf[350];
float filteredRollBuf[350];

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
	//Test one by one
	//Test the filtered data when the sensor is stand still
	//Test the raw data when the sensor is standing still
	//test the filtered data when the sensor rotate from 0 to 90deg
	//Test the raw data when the sensor rotate from 0 to 90 deg
	i3g4250d_init();
	i3g4250d_calibrate(1000);
	i3g4250d_softLPF_config(10, sampleRate); //2Hz more smooth but more lag, 10Hz more responsive but more wiggle

	while(1){
		i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
	}
}

//#if 0 //Write 1 to activate this block of code for testing raw estimated roll vs LPF estimated roll angle
//	UART1_DMA_Transmitter_Init();
//
//	for(int i = 0; i < arraySize(rawRollBuf); i++){
//		i3g4250d_getAngle(&rollAngle, &pitchAngle, &yawAngle, sampleRate);
//		i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
//		rawRollBuf[i] = rollAngle;
//		filteredRollBuf[i] = rollAngleFiltered;
//	}
//
//	i3g4250d_sendRollToUART(&rawRollBuf, &filteredRollBuf, arraySize(rawRollBuf));
//
//	while(1){
//
//	}
//#elif 0 //Write 1 to print filtered roll angles through UART
//	while(1){
//		uartPrintLog(my_UART1, "rollAngle: ");
//		i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
//		uartPrintFloat(my_UART1, rollAngleFiltered, 2);
//		uartPrintLog(my_UART1, "\n");
//	}
//
//#elif 1 //Write 1 to print filtered pitch angles through UART
//	while(1){
//		uartPrintLog(my_UART1, "pitchAngle: ");
//		i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
//		uartPrintFloat(my_UART1, pitchAngleFiltered, 2);
//		uartPrintLog(my_UART1, "\n");
//	}
//
//#else //Otherwise, print filtered yaw angles through UART
//	while(1){
//		uartPrintLog(my_UART1, "yawAngle: ");
//		i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
//		uartPrintFloat(my_UART1, yawAngleFiltered, 2);
//		uartPrintLog(my_UART1, "\n");
//	}
//
//#endif
//}





