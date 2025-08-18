/*
 * @brief	uart.c
 *
 * 	Created on: Apr 23, 2025
 * 	Updated on: Jul 18, 2025
 * 		Improved program flow
 * 		Improved logical flow
 * 		Added NVIC interrupts
 * 		Added DMA features
 *
 * 		No bug for now:)))
 *
 * 		Author: dobao
 */

#include "uart.h"

/*
 * -----------------------------------------------------------
 * Private Helpers
 * -----------------------------------------------------------
 */
/* Base-Address Lookup */
static inline volatile uartRegOffset_t* uartBase(UART_Name_t uartName){
	switch(uartName){
		case my_UART1: return UART1_REG;
		case my_UART2: return UART2_REG;
		case my_UART6: return UART6_REG;
		default: return NULL;
	}
}



/* Enable GPIOs' Clock */
static inline General_Status_t enableGpioClock(GPIO_PortName_t port){
#if 0
	switch(port){
		case my_GPIOA: my_RCC_GPIOA_CLK_ENABLE(); return OK;
		case my_GPIOB: my_RCC_GPIOB_CLK_ENABLE(); return OK;
		case my_GPIOC: my_RCC_GPIOC_CLK_ENABLE(); return OK;
		case my_GPIOD: my_RCC_GPIOD_CLK_ENABLE(); return OK;
		case my_GPIOE: my_RCC_GPIOE_CLK_ENABLE(); return OK;
		case my_GPIOH: my_RCC_GPIOH_CLK_ENABLE(); return OK;
		default: return INVALID_PORT;
	}
#else
	switch(port){
		case my_GPIOA: __HAL_RCC_GPIOA_CLK_ENABLE(); return OK;
		case my_GPIOB: __HAL_RCC_GPIOB_CLK_ENABLE(); return OK;
		case my_GPIOC: __HAL_RCC_GPIOC_CLK_ENABLE(); return OK;
		case my_GPIOD: __HAL_RCC_GPIOD_CLK_ENABLE(); return OK;
		case my_GPIOE: __HAL_RCC_GPIOE_CLK_ENABLE(); return OK;
		case my_GPIOH: __HAL_RCC_GPIOH_CLK_ENABLE(); return OK;
		default: return INVALID_PORT;
	}
}

#endif

/* Enable UARTs' Clock */
static inline UART_Status_t enableUartClock(UART_Name_t uartName){
#if 0
	switch(uartName){
		case my_UART1: my_RCC_USART1_CLK_ENABLE(); return UART_OK;
		case my_UART2: my_RCC_USART2_CLK_ENABLE(); return UART_OK;
		case my_UART6: my_RCC_USART6_CLK_ENABLE(); return UART_OK;
		default: return UART_NOT_OK;
	}
#else
	switch(uartName){
		case my_UART1: __HAL_RCC_USART1_CLK_ENABLE(); return UART_OK;
		case my_UART2: __HAL_RCC_USART2_CLK_ENABLE(); return UART_OK;
		case my_UART6: __HAL_RCC_USART6_CLK_ENABLE(); return UART_OK;
		default: return UART_NOT_OK;
	}
}
#endif


/*
 * @brief	Helper function to write bit to a specific uart's register
 *
 * @param	bitPosition
 * @param	uartName
 * @param	regName
 * @param	value
 */
UART_Status_t writeUART(uint8_t bitPosition, UART_Name_t uartName, UART_RegName_t regName, uint32_t value){
	volatile uartRegOffset_t* UARTx = uartBase(uartName);
	if(UARTx == NULL) {return INVALID_UART; }

	uint8_t bitWidth = 1;
	volatile uint32_t* reg = NULL;

	switch(regName){
		case UART_SR:
			reg = &UARTx -> UART_SR;
			break;

		case UART_DR:
			uint32_t cr1 = UARTx -> UART_CR1;
			bool nineBits = ((cr1 & UART_CR1_M) && (cr1 & UART_CR1_PCE)) ||
							((cr1 & UART_CR1_M) && !(cr1 & UART_CR1_PCE));

			bitWidth = nineBits ? 9 : 8;
			reg = &UARTx -> UART_DR;
			break;

		case UART_BRR:
			bitWidth = 16;
			reg = &UARTx -> UART_BRR;
			break;

		case UART_CR1:
			reg = &UARTx -> UART_CR1;
			break;

		case UART_CR2:
			if(bitPosition == 0) bitWidth = 4;
			else if(bitPosition == 12) bitWidth = 2;
			reg = &UARTx -> UART_CR2;
			break;

		case UART_CR3:
			reg = &UARTx -> UART_CR3;
			break;

		case UART_GTPR:
			if(bitPosition == 0) bitWidth = 8;
			else if(bitPosition == 8) bitWidth = 8;
			reg = &UARTx -> UART_GTPR;
			break;

		default: return INVALID_REG;
	}

	//Prevent overflow
	if(bitWidth > 32 || bitPosition + bitWidth > 32) return UART_NOT_OK;
	if(bitWidth < 32 && value >= (1U << bitWidth)) return UART_NOT_OK;

	//Mask off the old bit and OR with new value
	uint32_t mask = ((1U << bitWidth) - 1U) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition) & mask;
	*reg = (*reg & ~mask) | shiftedValue;

	return UART_OK;
}



/*
 * @brief	read a UART peripheral or a single flag bit
 *
 * @param	bitPosition		Bit idx to test when you want a single flag
 * 							Pass 0 to get entire register value
 * @param	uartName		Enumerated UART peripheral
 * @param	regName			Which uart register to acccess.
 */
int32_t readUART(uint8_t bitPosition, UART_Name_t uartName, UART_RegName_t regName){
	if(bitPosition >= 32) return -1;
	volatile uartRegOffset_t* UARTx = uartBase(uartName);
	if(UARTx == NULL) return -1;

	volatile uint32_t* reg = NULL;
	switch(regName){
		case UART_SR: reg = &UARTx -> UART_SR; break;
		case UART_DR: reg = &UARTx -> UART_DR; break;
		case UART_BRR: reg = &UARTx -> UART_BRR; break;
		case UART_CR1: reg = &UARTx -> UART_CR1; break;
		case UART_CR2: reg = &UARTx -> UART_CR2; break;
		case UART_CR3: reg = &UARTx -> UART_CR3; break;
		case UART_GTPR: reg = &UARTx -> UART_GTPR; break;
		default: return -1;
	}

	if(regName == UART_DR){
		/*
		 * Wordlength and parity aware
		 */
		uint32_t cr1 = UARTx -> UART_CR1;
		bool nineBits = ((cr1 & UART_CR1_M) && (cr1 & UART_CR1_PCE)) ||
						((cr1 & UART_CR1_M) && !(cr1 & UART_CR1_PCE));

		uint32_t bitMask = nineBits ? 0x1FFu : 0xFFu;

		return (int32_t)(*reg & bitMask); //Return full 9/8 bits depend on setting
	}

	if(bitPosition == 0){
		return (int32_t)* reg;
	}

	return (int32_t)((*reg >> bitPosition) & 1U);
}


/*
 * ------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------
 */

/*
 * @brief	Set up DMA2-Stream2-Channel4 to move incoming UART1 RX bytes into a user-supplier buffer
 *
 * @param	rxBuffer	Pointer to the receive buffer (must be word-aligned if required by ur DMA)
 * 						Its size must be programmed into NDTR before each reception.
 *
 * @routine:
 * 		1. Enable DMA mode for reception in UART1 (UART_CR3)
 * 		2. Enable DMA2 peripheral clock
 * 		3. Programs DMA2 Stream 2 registers:
 * 			PAR 	<- &UART1.DR (feed DR address)
 * 			M0AR	<- rxBuffer	 (destination in RAM)
 * 			NDTR	<- sizeof(buffer)	(number of bytes to receive)
 * 			CR		<- channel 4 | 8bit | MINC | CIRC | TCIE | EN
 * 		4. Enable the NVIC interrupt for DMA2_Stream2
 */
void UART1_DMA_Receiver_Init(char *rxBuffer, uint32_t bufferSize){
	writeUART(6, my_UART1, UART_CR3, SET); //Enable DMA for reception
	/*
	 * According to DMA2 request mapping
	 * 		Choose Stream 2, channel 4 for UART1_RX
	 */
#if 0
	my_RCC_DMA2_CLK_ENABLE(); //Enabling clock for DMA2
#else
	__HAL_RCC_DMA2_CLK_ENABLE(); //Enabling clock for DMA2 using HAL lib
#endif

	/* Assign the address of sender which is UART_DR*/
	writeDMA2(0, DMA_S2CR, RESET); //Disable stream before configuring
	while((readDMA2(0, DMA_S2CR) & 0x1) == SET); //Wait until stream 2 is truly disabled

	writeDMA2(0, DMA_S2PAR, (uint32_t)UART1_GET_REG(UART_DR)); //Write sender address to DMA_S2PAR
	writeDMA2(0, DMA_S2M0AR, (uint32_t)rxBuffer); //Write receiver address to DMA_S2M0AR
	writeDMA2(0, DMA_S2NDTR, bufferSize); //Let DMA knows the size of the transfered package

	writeDMA2(25, DMA_S2CR, 0b100); //Select channel 4

	writeDMA2(11, DMA_S2CR, 0b00); //Set data size is 8-bit
	writeDMA2(8, DMA_S2CR, SET); //Enable circular mode

	if(bufferSize > 1){
		writeDMA2(10, DMA_S2CR, SET); //Set memory increment mode
	} else writeDMA2(10, DMA_S2CR, RESET); //No memory increment mode

	writeDMA2(4, DMA_S2CR, SET); //Enable transfer complete interrupt

	UART1_DMA_Receiver_Start();
}


void UART1_DMA_Receiver_Start(){
	writeDMA2(0, DMA_S2CR, SET); //Enable Stream 2
	NVIC_enableIRQ(DMA2_S2);
}


void UART1_DMA_Transmitter_Init(void){
	/*
	 * According to DMA2 request mapping
	 * 		Choose Stream 7 and channel 4 for UART1_TX
	 */
#if 0
	my_RCC_DMA2_CLK_ENABLE();
#else
	__HAL_RCC_DMA2_CLK_ENABLE();
#endif

	writeDMA2(0, DMA_S7CR, RESET); //Disable stream before configuring
	while((readDMA2(0, DMA_S7CR) & 0x1) == SET); //Wait until stream 2 is truly disabled

	/* Assign the address of sender which is UART_DR*/
	writeDMA2(0, DMA_S7PAR, (uint32_t)UART1_GET_REG(UART_DR)); //Write sender address to DMA_S7PAR
	writeDMA2(25, DMA_S7CR, 0b100); //Select channel 4
	writeDMA2(6, DMA_S7CR, 0b01); //Data transfer direction: memory to peripheral

	uint32_t cr1 = UART1_REG -> UART_CR1;
	bool nineBits = ((cr1 & UART_CR1_M) && (cr1 & UART_CR1_PCE)) ||
					((cr1 & UART_CR1_M) && !(cr1 & UART_CR1_PCE));
	uint8_t dataSizeSel = nineBits ? 0b01 : 0b00;

	writeDMA2(11, DMA_S7CR, 0b00); //Set data size is 8 bit (default)
	writeDMA2(10, DMA_S7CR, SET); //Set memory increment mode
	writeDMA2(8, DMA_S7CR, RESET); //Normal mode
//	writeDMA2(4, DMA_S7CR, SET); //Enable transfer complete interrupt
//	NVIC_enableIRQ(DMA2_S7);
	writeUART(7, my_UART1, UART_CR3, SET); //Enable DMA for transmission
}


void UART1_DMA_Transmitter_Start(const char* txBuffer, uint32_t bufferSize){
	writeDMA2(0, DMA_S7CR, RESET); //Disable stream before configuring
	while((readDMA2(0, DMA_S7CR) & 0x1)); //Wait until stream 2 is truly disabled

	/* Clear the flags */
	writeDMA2(27, DMA_HIFCR, SET); //Clear transfer completed interrupt flag of stream 7
	writeDMA2(26, DMA_HIFCR, SET); //Clear half transfer completed interrupt flag of stream 7
	writeDMA2(25, DMA_HIFCR, SET); //Clear transfer error interrupt flag
	writeDMA2(24, DMA_HIFCR, SET); //Clear direct mode error interrupt flag
	writeDMA2(22, DMA_HIFCR, SET); //Clear FIFO error interrupt flag

	writeDMA2(0, DMA_S7M0AR, (uint32_t)txBuffer); //Write receiver address to DMA_S7M0AR
	writeDMA2(0, DMA_S7NDTR, bufferSize); //Let DMA knows the size of the transfered package
	writeDMA2(0, DMA_S7CR, SET); //Enable Stream 7, ready to start!
}

void UART1_DMA_Transmitter_Complete(void){
	while((readDMA2(27, DMA_HISR)) == 0); //A transfer complete event occured on stream 7
	while((readUART(6, my_UART1, UART_SR) & 1) == 0); //Wait UART TC
}

/*
 * @brief	 High-level helper to configure UART peripheral and its GPIO pins.
 *
 * @param	TXPin		Logical GPIO pin used as UARTx_TX
 * @param	RXPin		Logical GPIO pin used as UARTx_RX
 * @param	portName	GPIO port hosting the pins (my_GPIOA)
 * @param	uartName	Target UART peripheral (my_UART1)
 * @param	baudRate	Desired baud rate in bit/s (e.g. 115200)
 * @param	parity		PARITY_NONE, PARITY_EVEN, OR PARITY_ODD
 * @param	wordLength	_8B_WORDLENGTH or _9B_WORDLENGTH
 *
 * @note	The function assumes SYSCLK_FREQ_100M is set to the current peripheral clock speed.
 * 			Update this constant if the clock tree is changed
 */
void UART_Init(GPIO_Pin_t TXPin,
			   GPIO_Pin_t RXPin,
			   GPIO_PortName_t portName,
			   UART_Name_t uartName,
			   uint32_t baudRate,
			   UART_Parity_t parity,
			   UART_WordLength_t wordLength){
	/* Clock */
	enableUartClock(uartName);
	enableGpioClock(portName);

	/*
	 * Set TX/RXpin of a specific port to MODER mode and set its state to 0x02 (Alternate function mode)
	 */
	writePin(TXPin, portName, MODER, AF_MODE);
	writePin(RXPin, portName, MODER, AF_MODE);

	writePin(TXPin, portName, (TXPin <= 7U) ? AFRL : AFRH, AF7);
	writePin(RXPin, portName, (RXPin <= 7U) ? AFRL : AFRH, AF7);

	/*
	 * CONFIG UART
	 *
	 * Config baud rate
	 * baud = fclk / (8*(2-OVER8)*UARTDIV)
	 *
	 * APB2 supports System Clock upto 100MHz
	 */
	volatile uartRegOffset_t* huart = uartBase(uartName);
	if(huart == NULL) {return;}

	long int f_clk = SYSCLK_FREQ_16M; //Note: Remember to change this when you change clock speed in RCC
	char over8 = 0; //16x oversampling
	float uartDiv = (float)f_clk / (8.0f * (2 - over8) * (float)baudRate);

	int divMantissa = (int)uartDiv; //Extract the integer part
	float divFraction = uartDiv - divMantissa; //Extract the fraction part
	int convert_divFraction = (int)(roundf(divFraction * 16.0f)); //scale and store divFraction in 4 bits
	uint16_t fullBRR = (divMantissa << 4) | (convert_divFraction & 0xF);

	/* Write Calculated full values to BRR */
	writeUART(0, uartName, UART_BRR, fullBRR);

	/* Enable Tx and Rx */
	writeUART(2, uartName, UART_CR1, 1); //Receiver is enabled and begins searching for a start bit
	writeUART(3, uartName, UART_CR1, 1); //Transmitter enable

	/* Parity Configuration	 */
	if(parity == PARITY_NONE){
		writeUART(10, uartName, UART_CR1, 0); //PCE = 0
	}else{
		writeUART(10, uartName, UART_CR1, 1); //PCE = 1

		if(parity == PARITY_EVEN){
			writeUART(9, uartName, UART_CR1, 0); //PS = 0 for EVEN
		} else{
			writeUART(9, uartName, UART_CR1, 1); //PS = 1 for ODD
		}
	}

	/* Word Length */
	if (wordLength == _8B_WORDLENGTH){
		writeUART(12, uartName, UART_CR1, 0); //Set data frame size as 8 bits
	}else if (wordLength == _9B_WORDLENGTH) {
		writeUART(12, uartName, UART_CR1, 1); //Set data frame size as 9 bits
	}else{
		return;
	}
	writeUART(13, uartName, UART_CR1, 1); //Enable UART
}


void UART_RX_Interrupt_Enable(UART_Name_t uartName){
	writeUART(5, uartName, UART_CR1, SET);
	NVIC_enableIRQ(UART1); //Position 37 in vector table
}
void UART_TX_Interrupt_Enable(UART_Name_t uartName){
	writeUART(7, uartName, UART_CR1, SET);
	NVIC_enableIRQ(UART1); //Position 37 in vector table
}

void UART_RX_Interrupt_Disable(UART_Name_t uartName){
	writeUART(5, uartName, UART_CR1, RESET);
	NVIC_disableIRQ(UART1);
}

void UART_TX_Interrupt_Disable(UART_Name_t uartName){
	writeUART(7, uartName, UART_CR1, RESET);
	NVIC_disableIRQ(UART1);
}


/*
 * 	@brief	Send one byte over a selected UART port
 * 	@param 	Target peripheral: my_UART1, my_UART2, my_UART6.
 * 	@param	inputData: write any value/data to but only <= 1 byte
 *
 * 	Routine:
 * 		1. Waits for TXE = 1 (DR empty)
 * 		2. Writes the inputData to DR register
 * 		3. Wait for TC = 1 to ensure the byte has completely left the shift register.
 */
void my_UART_Transmit(UART_Name_t UARTx, uint16_t inputData){
	while((readUART(7, UARTx, UART_SR) & 1) == 0);
	writeUART(0, UARTx, UART_DR, inputData);
	while((readUART(6, UARTx, UART_SR) & 1) == 0);
}



/*
 * @brief	Receives a character over the specified UART interface
 *
 * This function waits until data is available in the receive buffer (RXNE == 1)
 * Checks for a parity error (PE == 1), and if there is no error, returns the received byte
 * If a parity error occurs, it discards the corrupted data and return -1.
 */
int32_t my_UART_Receive(UART_Name_t uartName){
	volatile uartRegOffset_t* huart = uartBase(uartName);
	if(huart == NULL){return INVALID_UART;} //Invalid peripheral

	//Wait until the data is ready
	while(readUART(5, uartName, UART_SR) == 0); //Wait for RXNE == 1

	/*
	 * To clear the PE, FE, NE, or ORE flags, the following sequence must be respected:
	 * Read the SR register
	 * Read the DR register
	 *
	 * If skip DR read:
	 * 		The flag won't be cleared
	 * 		You may get stuck in your parity check loop
	 * 		Worst case: DR wont get new data, leading to data loss or lockup
	 */

	//Check parity error (bitPosition = 0)
	if(readUART(0, uartName, UART_SR) == 1){
		(void)readUART(0, uartName, UART_DR); //read and discard error
		return UART_NOT_OK; //There is parity error
	}

	//No error, return valid data
	int16_t data = readUART(0, uartName, UART_DR);
	return data;
}



/*
 * @brief	Send a zero-terminated ASCII string over the selected UART port
 *
 * @param	uartName	UART peripheral (example: my_UART1, my_UART2, and my_UART6)
 * @param	message		Pointer to a C-string that ends with '\0'
 *
 * @note!	The routine is blocking: it waits inside my_UART_Transmit()
 * 			until each byte has physically left the shift register.
 * 			For short status messages that is fine; for large logs consider
 * 			an IRQ or DMA-drivin Tx buffer so the CPU can do other work.
 */
void uartPrintLog(UART_Name_t uartName, char* message){
	int messageLength = strlen(message);
	for(short i = 0; i < messageLength; i++){
		my_UART_Transmit(uartName, message[i]);
	}
}



/*
 * @brief	The function formats @p val with the number of digits specifies by decimals (0-9)
 * 			Transmits the resulting string through uartPrintLog()
 *
 * @note:
 * 		A 20-byte local buffer, @c buf, temporarily holds the formatted string.
 * 		The conversion format string @c fmt starts as "%.1f"; the precision character
 * 		(index 2) is overwritten with @c '0' + decimal, allowing any precision from %.0f to %.9f
 *
 * 		snprintf)() is used for safe and bounded formatting
 */
void uartPrintFloat(UART_Name_t uartName, float val, uint8_t decimals){
	char buf[20]; //20 bytes array (each index stores a byte value)
	char fmt[] = "%.1f"; //C-string stored in a writable character array
	/*
	 * fmt[0] = '%'		Start of conversion specification
	 * fmt[1] = '.'		Dot introduces the precision field
	 * fmt[2] = '1'		Precision digit (one digit after the decimal)
	 * fmt[3] = 'f'		Format as floating point
	 */
	fmt[2] = '0' + decimals; //Allow 0-9 decimals
	snprintf(buf, sizeof buf, fmt, val);
	uartPrintLog(uartName, buf);
}
