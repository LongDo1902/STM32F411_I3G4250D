/*
 * led.h
 *
 *  Created on: Jul 16, 2025
 *      Author: dobao
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "gpioWriteRead.h"
#include "stm32PeripheralAddr.h"
#include "rcc.h"

typedef enum{
	LED_GREEN = 12, //Pin 12
	LED_ORANGE, //Pin 13
	LED_RED, //Pin 14
	LED_BLUE //Pin 15
}LED_Color_t;

typedef enum{
	OFF = 0,
	ON = 1
}LED_State_t;

void ledGreenInit();
void ledOrangeInit();
void ledRedInit();
void ledBlueInit();

void ledControl(LED_Color_t ledColor, LED_State_t on_off);

#endif /* INC_LED_H_ */
