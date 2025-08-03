/*
 * spi.h
 *
 *  Created on: Jul 26, 2025
 *      Author: dobao
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32PeripheralAddr.h"
#include "gpioWriteRead.h"
#include "rcc.h"

typedef enum{
	_SPI1,
	_SPI2,
	_SPI3,
	_SPI4,
	_SPI5
}SPI_Name_t;

typedef enum{
	SPI_CR1,
	SPI_CR2,
	SPI_SR,
	SPI_DR,
	SPI_CRC_PR,
	SPI_RX_CRCR,
	SPI_TX_CRCR,
	SPI_I2S_CFGR,
	SPI_I2S_PR,

	SPI_REG_COUNT
}SPI_Reg_t;

typedef enum{
	SOFTWARE_SLAVE_DISABLE,
	SOFTWARE_SLAVE_ENABLE
}SPI_SSM_t; //Software Slave Management

typedef enum{
	SPI_DISABLE,
	SPI_ENABLE
}SPI_Enable_t;

typedef enum{
	FPCLK_DIV2 = 0b000, //freq/2 = 16MHz/2
	FPCLK_DIV4 = 0b001,
	FPCLK_DIV8 = 0b010,
	FPCLK_DIV16 = 0b011, //freq/16 = 16MHz/16
	FPCLK_DIV32 = 0b100,
	FPCLK_DIV64 = 0b101,
	FPCLK_DIV128 = 0b110,
	FPCLK_DIV2256 = 0b111
}SPI_BaudRate_t;

typedef enum{
	STM32_AS_SLAVE,
	STM32_AS_MASTER
}SPI_MSTR_t; //Master selection

typedef enum{
	DFF_8BITS,
	DFF_16BITS
}SPI_DFF_t; //Data Frame Format

typedef struct{
	SPI_Name_t SPIx;

	GPIO_Pin_t sckPin;
	GPIO_PortName_t sckPort;

	GPIO_Pin_t nssPin;
	GPIO_PortName_t nssPort;

	GPIO_Pin_t mosiPin;
	GPIO_PortName_t mosiPort;

	GPIO_Pin_t misoPin;
	GPIO_PortName_t misoPort;
}SPI_GPIO_Config_t;


/*
 * ===============================================================
 * Public Helper Function Declarations
 * ===============================================================
 */
void writeSPI(uint8_t bitPosition,
			  SPI_Name_t SPIx,
			  SPI_Reg_t regName,
			  uint32_t value);

uint16_t readSPI(uint8_t bitPosition,
				 SPI_Name_t SPIx,
				 SPI_Reg_t regName);

char SPI_readReceivedData(SPI_GPIO_Config_t config,
						  uint8_t slaveDeviceAddr);

bool SPI_readBurstBuf(SPI_GPIO_Config_t config,
					uint8_t startAddr,
					uint8_t* rxBuf,
					uint8_t len);

void SPI_write2Device(SPI_GPIO_Config_t config,
					  uint8_t slaveDeviceAddr,
					  uint8_t writeValue);


/*
 * ===============================================================
 * Public Main Function Declarations
 * ===============================================================
 */
void SPI_sckPin_init(GPIO_Pin_t sckPin, GPIO_PortName_t sckPort, SPI_Name_t SPIx);
void SPI_mosiPin_init(GPIO_Pin_t mosiPin, GPIO_PortName_t mosiPort, SPI_Name_t SPIx);
void SPI_misoPin_init(GPIO_Pin_t misoPin, GPIO_PortName_t misoPort, SPI_Name_t SPIx);
void SPI_GPIO_init(SPI_GPIO_Config_t config);
void SPI_basicConfigInit(SPI_GPIO_Config_t config,
						 SPI_MSTR_t masterSlaveSel,
						 SPI_DFF_t dataFrameSize,
						 SPI_BaudRate_t baudRateSel,
						 SPI_SSM_t softSlaveEn,
						 SPI_Enable_t enableMode);

#endif /* INC_SPI_H_ */
