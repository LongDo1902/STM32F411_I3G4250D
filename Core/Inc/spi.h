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

#define SPI_SR_BSY_POS		7 //Busy flag
#define SPI_SR_OVR_POS		6 //Overrun flag
#define SPI_SR_MODF_POS		5 //Mode fault
#define SPI_SR_CRCERR_POS	4 //CRC error flag
#define SPI_SR_UDR_POS		3 //Underrun flag
#define	SPI_SR_CHSIDE_POS	2 //Channel side
#define SPI_SR_TXE_POS		1 //Transmit buffer empty
#define SPI_SR_RXNE_POS		0 //Receive buffer not empty

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

uint8_t SPI_readRegUnsigned(const SPI_GPIO_Config_t *config, uint8_t regAddr);
int8_t SPI_readRegSigned(const SPI_GPIO_Config_t *config, uint8_t regAddr);
bool SPI_readRegBurst(const SPI_GPIO_Config_t *config, uint8_t startRegAddr, uint8_t *rxBuf, uint8_t len);
void SPI_writeReg(const SPI_GPIO_Config_t *config, uint8_t regAddr, uint8_t writeValue);


/*
 * ===============================================================
 * Public Main Function Declarations
 * ===============================================================
 */
void SPI_sckPin_init(GPIO_Pin_t sckPin, GPIO_PortName_t sckPort, SPI_Name_t SPIx);
void SPI_mosiPin_init(GPIO_Pin_t mosiPin, GPIO_PortName_t mosiPort, SPI_Name_t SPIx);
void SPI_misoPin_init(GPIO_Pin_t misoPin, GPIO_PortName_t misoPort, SPI_Name_t SPIx);
void SPI_GPIO_init(const SPI_GPIO_Config_t *config);
void SPI_basicConfigInit(const SPI_GPIO_Config_t *config,
						 SPI_MSTR_t masterSlaveSel,
						 SPI_DFF_t dataFrameSize,
						 SPI_BaudRate_t baudRateSel,
						 SPI_SSM_t softSlaveEn);

#endif /* INC_SPI_H_ */
