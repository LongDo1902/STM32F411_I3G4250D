/*
 * led.c
 *
 *  Created on: Jul 16, 2025
 *      Author: dobao
 */
#include "led.h"

void ledGreenInit(){
	my_RCC_GPIOD_CLK_ENABLE();
	writePin(LED_GREEN, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledOrangeInit(){
	my_RCC_GPIOD_CLK_ENABLE();
	writePin(LED_ORANGE, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledRedInit(){
	my_RCC_GPIOD_CLK_ENABLE();
	writePin(LED_RED, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledBlueInit(){
	my_RCC_GPIOD_CLK_ENABLE();
	writePin(LED_BLUE, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledControl(LED_Color_t ledColor, LED_State_t on_off){
	if(on_off == ON){
		writePin(ledColor, my_GPIOD, BSRR, SET);
	}
	else if(on_off == OFF){
		writePin(ledColor, my_GPIOD, BSRR, RESET);
	}
	else return;
}


