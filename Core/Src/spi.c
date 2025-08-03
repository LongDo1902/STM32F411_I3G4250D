/*
 * @file	spi.c
 * @brief	Low-level helpers for bit-masked accessm GPIO/AF setup and simple polling-mode
 * 			transfer (single-byte and burst) suitable for MEMS sensors such as the I3G4250D
 *
 * Created on: Jul 26, 2025
 *      Author: dobao
 */
#include "spi.h"

/*
 * =====================================================================
 * Private Helpers - Register base address lookup
 * =====================================================================
 */
static inline volatile spiRegOffset_t* spiBase(SPI_Name_t SPIx){
	switch(SPIx){
		case _SPI1:	return SPI1_REG;
		case _SPI2: return SPI2_REG;
		case _SPI3: return SPI3_REG;
		case _SPI4:	return SPI4_REG;
		case _SPI5:	return SPI5_REG;
		default: return NULL; //Invalid enum value
	}
}


/*
 * =====================================================================
 * Generic Bit-field Private Helpers (write/read)
 * =====================================================================
 */
static inline void writeSPIBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	if(bitPosition > 31 || bitWidth > 32) return;
	if((bitWidth + bitPosition) > 32) return;
	if((bitWidth < 32) && (value > ((1U << bitWidth) - 1U))) return;

	//Mask off the old bit and OR with the new value
	uint32_t mask = (bitWidth == 32) ? 0xFFFFFFFFU : (((1U << bitWidth) - 1U) << bitPosition);
	uint32_t shiftedVal = (value << bitPosition) & mask;
	*reg = (*reg & ~mask) | shiftedVal;
}

static inline uint16_t readSPIBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	if(bitWidth == 32) return *reg;
	uint32_t mask = ((1U << bitWidth) - 1U);
	return (*reg >> bitPosition) & mask;
}


/*
 * =====================================================================
 * Public Helpers - writeSPI
 * 					readSPI
 * 					SPI_readReceivedData
 * 					SPI_readBuffer
 * 					SPI_write2Device
 * =====================================================================
 */
/*
 *  @brief	This function configures bit to SPIx's register
 *
 *  @param	bitPosition		bit location that you want to write
 *  @param	SPIx			write my_SPI1 when you want to config SPI1
 *  @param	regName			choose specific SPI register to write bit
 *  @param	value			any value that < 32 bits
 */
void writeSPI(uint8_t bitPosition,
			  SPI_Name_t SPIx,
			  SPI_Reg_t regName,
			  uint32_t value){

	volatile spiRegOffset_t* SPIx_p = spiBase(SPIx);
	if(!SPIx_p) return; //Paranoia check

	uint8_t bitWidth = 1;
	volatile uint32_t* reg = NULL;

	switch(regName){
		case SPI_CR1:
			if(bitPosition == 3) bitWidth = 3;
			reg = &SPIx_p -> SPI_CR1;
			break;

		case SPI_CR2:
			reg = &SPIx_p -> SPI_CR2;
			break;

		case SPI_SR:
			if(bitPosition != 4) return;
			reg = &SPIx_p -> SPI_SR;
			break;

		case SPI_DR:
			bitWidth = ((SPIx_p -> SPI_CR1) & (1U << 11)) ? 16 : 8;
			reg = &SPIx_p -> SPI_DR;
			break;

		case SPI_CRC_PR:
		case SPI_RX_CRCR:
		case SPI_TX_CRCR:
			bitWidth = 16;
			reg = &SPIx_p -> SPI_CRC_PR + (regName - SPI_CRC_PR); //All 16-bit
			break;

		case SPI_I2S_CFGR:
			if((bitPosition == 1) || (bitPosition == 4) || (bitPosition == 8)) bitWidth = 2;
			reg = &SPIx_p -> SPI_I2S_CFGR;
			break;

		case SPI_I2S_PR:
			if(bitPosition == 0) bitWidth = 8;
			reg = &SPIx_p -> SPI_I2S_PR;
			break;

		default: return;
	}
	if(!reg) return;
	writeSPIBits(reg, bitPosition, bitWidth, value);
}


/*
 *  @brief	This function reads and returns bit/status from SPIx's register
 *
 *  @param	bitPosition		bit location that you want to write
 *  @param	SPIx			write my_SPI1 when you want to config SPI1
 *  @param	regName			choose specific SPI register to write bit
 */
uint16_t readSPI(uint8_t bitPosition, SPI_Name_t SPIx, SPI_Reg_t regName){
	const uint16_t ERROR = 0xFFFF;

	volatile spiRegOffset_t* SPIx_p = spiBase(SPIx);
	volatile uint32_t* reg = NULL;

	uint8_t bitWidth = 1;

	switch(regName){
		case SPI_CR1:
			if(bitPosition == 3) bitWidth = 3;
			reg = &SPIx_p -> SPI_CR1;
			break;

		case SPI_CR2:
			reg = &SPIx_p -> SPI_CR2;
			break;

		case SPI_SR:
			reg = &SPIx_p -> SPI_SR;
			break;

		case SPI_DR:
			bitWidth = ((SPIx_p -> SPI_CR1) & (1U << 11)) ? 16 : 8;
			reg = &SPIx_p -> SPI_DR;
			break;

		case SPI_CRC_PR:
		case SPI_RX_CRCR:
		case SPI_TX_CRCR:
			bitWidth = 16;
			reg = &SPIx_p -> SPI_CRC_PR + (regName - SPI_CRC_PR); //All 16-bit
			break;

		case SPI_I2S_CFGR:
			if((bitPosition == 1) || (bitPosition == 4) || (bitPosition == 8)) bitWidth = 2;
			reg = &SPIx_p -> SPI_I2S_CFGR;
			break;

		case SPI_I2S_PR:
			if(bitPosition == 0) bitWidth = 8;
			reg = &SPIx_p -> SPI_I2S_PR;
			break;

		default: return ERROR;
	}
	if(!reg) return ERROR;
	return readSPIBits(reg, bitPosition, bitWidth);
}


/*
 *	@brief		Reads a single byte of data from an SPI slave device using 8-bit full-duplex SPI
 *				Follows the standard SPI read protocol with a command phase and dummy-byte phase
 *				Works with devices like the L3GD20 or I3G4250 (MSB read = 1).
 *
 *	@param		config		Struct containing SPI peripheral and pin mappings (SPI_GPIO_Config_t)
 *	@param		slaveDeviceAddr		Register address of the target data in the slave device
 *
 *	@return		Received byte from the SPI slave.
 */
char SPI_readReceivedData(SPI_GPIO_Config_t config, uint8_t slaveDeviceAddr){
	//Parameters
	const uint8_t READ_FLAG = (1 << 7); //I3G4250 Read Flag (Write 1 to bit 7)
	const uint8_t DUMMYBYTE = 0xFF;

	//Ensure NSS GPIO is clocked and configured as output
//	writePin(config.nssPin, config.nssPort, ODR, my_GPIO_PIN_RESET); //Pull NSS pin low to activate the slave and begin communication
	writePin(config.nssPin, config.nssPort, BSRR, my_GPIO_PIN_RESET); //Pull NSS pin low to activate the slave and begin communication
	for(volatile int i = 0; i < 50; i++); //Add delay when BSRR is used

	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //SPI is busy in communication or TX buffer is not empty
	writeSPI(0, config.SPIx, SPI_DR, (slaveDeviceAddr | READ_FLAG)); //(1 << 7) is refered to L3GD20 gyro for reading mode

	while((readSPI(1, config.SPIx, SPI_SR) & 1) == 0); //Wait until TX buffer is empty
	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	while((readSPI(0, config.SPIx, SPI_SR) & 1) == 0); //Wait until RX buffer is full data

	char data = readSPI(0, config.SPIx, SPI_DR); //Read and discard first received byte (dummy value received)

	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	writeSPI(0, config.SPIx, SPI_DR, DUMMYBYTE); //Send dummy byte (0xFF) so the slave can send actual data back

	while((readSPI(1, config.SPIx, SPI_SR) & 1) == 0); //Wait until TX buffer is empty
	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	while((readSPI(0, config.SPIx, SPI_SR) & 1) == 0); //Wait until RX buffer is full data

	data = readSPI(0, config.SPIx, SPI_DR); //Read actual data
//	writePin(config.nssPin, config.nssPort, ODR, my_GPIO_PIN_SET); //Deactivate the slave, pull NSS pin high again to end communication
	writePin(config.nssPin, config.nssPort, BSRR, my_GPIO_PIN_SET); //Pull NSS pin low to activate the slave and begin communication
	return (char) data;
}


/*
 * @brief	Brust-read a block of registers from an SPI slave
 *
 *			Keeps NSS low
 *			Assigns command byte with READ (bit 7) and AUTO INCREMENT (bit 6) set
 *			Then, clocks in 'len'consecutive data bytes into the caller's buffer
 *
 * @param	config		GPIO/SPI mapping for the target bus instance
 * @param	startAddr	First register address to read (bit 0 to bit 5 only)
 * @param 	rxBuf		Pointer to destination buffer that will receive 'len' bytes. Must not be NULL
 * @param	len			Number of bytes to read; must be > 0
 *
 * @retVal	true	Transaction completed without any error
 * 			false	Invalid arguments (len == 0 or rxBuf == NULL) or SPI busy / timeout detected during the transfer
 */
bool SPI_readBurstBuf(SPI_GPIO_Config_t config, uint8_t startAddr, uint8_t* rxBuf, uint8_t len){
	if(len == 0u || rxBuf == NULL) return false;
	const uint8_t dummyByte = 0xFFu;
	const uint8_t cmd = startAddr | 0xC0u; //Read + auto increment addr

	writePin(config.nssPin, config.nssPort, BSRR, my_GPIO_PIN_RESET);
	for(volatile int i = 0; i < 25; i++);

	while(readSPI(7, config.SPIx, SPI_SR) & 1u);
	writeSPI(0, config.SPIx, SPI_DR, cmd);

	/* Wait for command to shift out and & dummy to shift in */
	while((readSPI(1, config.SPIx, SPI_SR) & 1u) == 0u); //Wait until TX buffer is empty
	while((readSPI(7, config.SPIx, SPI_SR) & 1u) == 1u); //Wait until SPI is not busy
	while((readSPI(0, config.SPIx, SPI_SR) & 1u) == 0u); //Wait until RX buffer is full data
	(void)readSPI(0, config.SPIx, SPI_DR); //Read dummy data

	/*
	 * Start to loop through length and store received data(s) to the buffer
	 */
	for(volatile uint8_t i = 0; i < len; i++){
		writeSPI(0, config.SPIx, SPI_DR, dummyByte); //Send dummy byte so the slave can send the actual data back

		while((readSPI(1, config.SPIx, SPI_SR) & 1u) == 0u); //Wait until TX buffer is empty
		while((readSPI(7, config.SPIx, SPI_SR) & 1u) == 1u); //Wait until SPI is not busy
		while((readSPI(0, config.SPIx, SPI_SR) & 1u) == 0u); //Wait until RX buffer is full dataFrameSize

		rxBuf[i] = (uint8_t)readSPI(0, config.SPIx, SPI_DR);
	}
	writePin(config.nssPin, config.nssPort, BSRR, my_GPIO_PIN_SET);
	return true;
}


/*
 * @brief	Write single register to the slave device at a specific register
 *
 * @note	bit 7 of 'slaveDeviceAddr' must already be 0
 *
 * @retVal	true on success
 * 			false on SPI busy/error time-out
 */
void SPI_write2Device(SPI_GPIO_Config_t config,
					  uint8_t slaveDeviceAddr,
					  uint8_t writeValue){
	writePin(config.nssPin, config.nssPort, ODR, my_GPIO_PIN_RESET); //Pull NSS pin low to start communicating

	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //SPI is busy in communicating or TX buffer is not yet empty
	writeSPI(0, config.SPIx, SPI_DR, slaveDeviceAddr);

	while((readSPI(1, config.SPIx, SPI_SR) & 1) == 0); //Wait until TX buffer is empty
	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	while((readSPI(0, config.SPIx, SPI_SR) & 1) == 0); //Wait until RX buffer is full data

	(void)readSPI(0, config.SPIx, SPI_DR); //Read and discard first received byte (dummy value received)

	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	writeSPI(0, config.SPIx, SPI_DR, writeValue); //Send the value that user want to write to the slave

	while((readSPI(1, config.SPIx, SPI_SR) & 1) == 0); //Wait until TX buffer is empty
	while((readSPI(7, config.SPIx, SPI_SR) & 1) == 1); //Wait until SPI is not busy
	while((readSPI(0, config.SPIx, SPI_SR) & 1) == 0); //Wait until RX buffer is full data

	writePin(config.nssPin, config.nssPort, ODR, my_GPIO_PIN_SET); //Pull the NSS pin high -> deactivate slave
}



/*
 * ===========================================================================
 * Public Main Functions	SPI_GPIO_init
 * 							SPI_basicConfigInit
 * 							SPI_mosiPin_init
 * 							SPI_misoPin_init
 * 							SPI_sckPin_init
 * ===========================================================================
 */
/*
 *  @brief	Initializes the selected SPI peripheral and its SCK, MOSI, MISO pins.
 */
void SPI_GPIO_init(SPI_GPIO_Config_t config){
	Enable_GPIO_Clock(config.sckPort);
	Enable_GPIO_Clock(config.nssPort);
	Enable_GPIO_Clock(config.mosiPort);
	Enable_GPIO_Clock(config.misoPort);

	writePin(config.nssPin, config.nssPort, MODER, OUTPUT_MODE); //Set NSS pin as output

	SPI_sckPin_init(config.sckPin, config.sckPort, config.SPIx);
	SPI_mosiPin_init(config.mosiPin, config.mosiPort, config.SPIx);
	SPI_misoPin_init(config.misoPin, config.misoPort, config.SPIx);
}


/*
 * @brief	Initialize the basic configuration of an SPI peripheral
 *
 * This includes:
 * 		Clock enabled for the selected SPI instance
 * 		Master/Slave mode selection
 * 		Baud rate configuration
 * 		Software slave management (SSM) and SSI
 * 		SPI peripheral enable
 *
 * @param	SPIx				Selected SPI peripheral (my_SPI1 to my_SPI5)
 * @param	masterSlaveSel		Set device as master or slave (SPI_MSTR_t)
 * @param	dataFrameSize		Set the data frame size (1 for 16-bits and 0 for 8bits)
 * @param	enableMode			Enable or disable SPI (SPI_Enable_t)
 * @param	baudRateSel			Baud rate prescaler (SPI_BaudRate_t)
 * @param	dataFrameSize		Data frame size for transmission/reception (0 for 8 bits and 1 for 16 bits)
 * @param	softSlaveEn 		Software slave management enable/disable (SPI_SSM_t)
 */
void SPI_basicConfigInit(SPI_GPIO_Config_t config,
						 SPI_MSTR_t masterSlaveSel,
						 SPI_DFF_t dataFrameSize,
						 SPI_BaudRate_t baudRateSel,
						 SPI_SSM_t softSlaveEn,
						 SPI_Enable_t enableMode){
#if 0
	//Flexible enable SPI clock
	switch(config.SPIx){
		case _SPI1: my_RCC_SPI1_CLK_ENABLE(); break; //100MHz
		case _SPI2: my_RCC_SPI2_CLK_ENABLE(); break; //50MHz
		case _SPI3: my_RCC_SPI3_CLK_ENABLE(); break; //50MHz
		case _SPI4: my_RCC_SPI4_CLK_ENABLE(); break; //100MHz
		case _SPI5: my_RCC_SPI5_CLK_ENABLE(); break; //100MHz
		default: return;
	}
#else
	switch(config.SPIx){
		case _SPI1: __HAL_RCC_SPI1_CLK_ENABLE(); break;
		case _SPI2: __HAL_RCC_SPI2_CLK_ENABLE(); break;
		case _SPI3: __HAL_RCC_SPI3_CLK_ENABLE(); break;
		case _SPI4: __HAL_RCC_SPI4_CLK_ENABLE(); break;
		case _SPI5: __HAL_RCC_SPI5_CLK_ENABLE(); break;
		default: return;
	}
#endif
	writeSPI(2, config.SPIx, SPI_CR1, masterSlaveSel); //Set STM32F411VET as master or slave
	writeSPI(11, config.SPIx, SPI_CR1, dataFrameSize); //Set data frame size (must be written before SPI is enabled)
	writeSPI(3, config.SPIx, SPI_CR1, baudRateSel); //Set how fast sckPin can generate (Hz)
	writeSPI(9, config.SPIx, SPI_CR1, softSlaveEn); //Needed if NSS is not physically connected

	/*
	 * bit 8 = 1 -> Simulate NSS being high so that MODF in SR is disabled
	 * bit 8 = 0 -> physical NSS is selected/connected
	 */
	writeSPI(8, config.SPIx, SPI_CR1, softSlaveEn == SOFTWARE_SLAVE_ENABLE ? 1 : 0);
	//Enable SPI must be put after all other features are activated
	writeSPI(6, config.SPIx, SPI_CR1, enableMode); //Disable or Enable SPI
}


/*
 * @brief	Initializes the MISO pin for a given SPI peripheral.
 * @param	misoPin		SPI MISO Pin
 * @param	misoPort	Port of the MISO pin
 * @param	SPIx		Target SPI (e.g., my_SPI1)
 */
void SPI_misoPin_init(GPIO_Pin_t misoPin, GPIO_PortName_t misoPort, SPI_Name_t SPIx){
	writePin(misoPin, misoPort, MODER, AF_MODE);
	GPIO_Mode_t afrRegMiso = (misoPin <= 7) ? AFRL : AFRH;

	/*
	 * Conditions to check which AFx order is chosen corresponding to misoPin, misoPort, and SPIx
	 * SPI1/2/4 -> AF5 (PA11 special case -> AF6)
	 * SPI3/5 -> AF6
	 */
	if(SPIx == _SPI1 || SPIx == _SPI2 || SPIx == _SPI4){
		if(misoPin == my_GPIO_PIN_11 && misoPort == my_GPIOA){
			writePin(misoPin, misoPort, afrRegMiso, AF6);
		}
		else{
			writePin(misoPin, misoPort, afrRegMiso, AF5);
		}
	}
	else if(SPIx == _SPI3 || SPIx == _SPI5){
		writePin(misoPin, misoPort, afrRegMiso, AF6);
	}
}


/*
 * @brief	Initializes the MOSI pin for a given SPI peripheral.
 * @param	mosiPin		SPI MOSI Pin
 * @param	mosiPort	Port of the mosi pin
 * @param	SPIx		Target SPI (e.g., my_SPI1)
 */
void SPI_mosiPin_init(GPIO_Pin_t mosiPin, GPIO_PortName_t mosiPort, SPI_Name_t SPIx){
	writePin(mosiPin, mosiPort, MODER, AF_MODE); //Config this pin in MODER to Alternate Function mode for SPI interface
	GPIO_Mode_t afrRegMosi = (mosiPin <= 7) ? AFRL : AFRH; //Check which AF reg (AFRL or AFRH) is chosen corresponding to pin number

	/*
	 * SPIx to Alternate Function Mapping (STM32F411VET)
	 * Conditions to check which AFx order is chosen corresponding to mosiPin, mosiPort, and SPIx
	 * SPI3/5 -> AF6 (PD6 special case -> AF5)
	 * SPI1/2/4 -> AF5
	 */
	if(SPIx == _SPI5 || SPIx == _SPI3){
		if(mosiPin == my_GPIO_PIN_6 && mosiPort == my_GPIOD){
			writePin(mosiPin, mosiPort, afrRegMosi, AF5);
		}
		else{
			writePin(mosiPin, mosiPort, afrRegMosi, AF6);
		}
	}
	else{
		writePin(mosiPin, mosiPort, afrRegMosi, AF5);
	}
}


/*
 * @brief	Initializes the SCK pin for a given SPI peripheral.
 * @param	sckPin		SPI clock pin
 * @param	sckPort		Port of the SCK pin
 * @param	SPIx		Target SPI (e.g., my_SPI1)
 */
void SPI_sckPin_init(GPIO_Pin_t sckPin, GPIO_PortName_t sckPort, SPI_Name_t SPIx){
	writePin(sckPin, sckPort, MODER, AF_MODE); //Config this pin in MODER to Alternate Function mode for SPI interface
	GPIO_Mode_t afrRegSck = (sckPin <= 7) ? AFRL : AFRH; //Check which AF reg (AFRL or AFRH) is chosen corresponding to pin number

	/*
	 * SPIx to Alternate Function Mapping (STM32F411VET)
	 * Conditions to check which AFx order is chosen corresponding to sckPin, sckPort, and SPIx
	 * SPI1/2/4 -> AF5
	 * SPI3 -> AF6 (PB12 special case -> AF7)
	 * SPI5 -> AF6
	 */
	if(sckPin == 12 && sckPort == my_GPIOB){
		writePin(sckPin, sckPort, afrRegSck, AF7); //set PB12 to AF07 (SPI3_SCK);
	}
	else if(SPIx == _SPI1 || SPIx == _SPI2 || SPIx == _SPI4){
		writePin(sckPin, sckPort, afrRegSck, AF5); //Set to SPI1/2/4_SCK
	}
	else{
		writePin(sckPin, sckPort, afrRegSck, AF6); //Set to SPI3/5_SCK
	}
}
















