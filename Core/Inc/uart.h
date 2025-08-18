/*
 * UART.h
 *
 *  Created on: July 18, 2025
 *      Author: dobao
 */

#ifndef INC_UART_H_
#define INC_UART_H_
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "stm32PeripheralAddr.h"
#include "stm32f4xx_hal.h"
#include "timer.h"
#include "rcc.h"
#include "gpioWriteRead.h"
#include "dma.h"
#include "exti.h"
#include "led.h"

#define UART_CR1_M (1U << 12)	//Wordlength 9Bit
#define UART_CR1_PCE (1U << 10) //Parity Control Enable

typedef enum{
	UART_SR,
	UART_DR,
	UART_BRR,
	UART_CR1,
	UART_CR2,
	UART_CR3,
	UART_GTPR
}UART_RegName_t;

typedef enum{
	my_UART1,
	my_UART2,
	my_UART6
}UART_Name_t;

typedef enum{
	PARITY_NONE,
	PARITY_EVEN,
	PARITY_ODD
}UART_Parity_t;

typedef enum{
	_8B_WORDLENGTH,
	_9B_WORDLENGTH
}UART_WordLength_t;

typedef enum{
	UART_OK,
	UART_NOT_OK,
	INVALID_UART,
	INVALID_REG
}UART_Status_t;

typedef enum{
	INVALID_PORT,
	OK
}General_Status_t;

typedef enum{
	DISABLE_DMA,
	ENABLE_DMA
}Enable_DMA_t;

/*
 * Function Declaration
 */
UART_Status_t writeUART(uint8_t bitPosition, UART_Name_t uartName, UART_RegName_t regName, uint32_t value);
int32_t readUART(uint8_t bitPosition, UART_Name_t uartName, UART_RegName_t regName);

void UART_Init(GPIO_Pin_t TXPin,
			   GPIO_Pin_t RXPin,
			   GPIO_PortName_t portName,
			   UART_Name_t uartName,
			   uint32_t baudRate,
			   UART_Parity_t parity,
			   UART_WordLength_t wordLength);

void my_UART_Transmit(UART_Name_t UARTx, uint16_t inputData);
int32_t my_UART_Receive(UART_Name_t uartName);

void UART1_DMA_Receiver_Init(char *rxBuffer, uint32_t bufferSize);
void UART1_DMA_Receiver_Start();

void UART1_DMA_Transmitter_Init(void);
void UART1_DMA_Transmitter_Start(const char* txBuffer, uint32_t bufferSize);
void uartPrintLog(UART_Name_t uartName, char* message);
void uartPrintFloat(UART_Name_t uartName, float val, uint8_t decimals);
void UART1_DMA_Transmitter_Complete(void);
#endif /* INC_UART_H_ */
