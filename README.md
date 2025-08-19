Project Introduction: This project focuses on achieving stable angular rate and estimated angle readings using the integrated I3G4250D gyroscope sensor on the STM32F411VET6 microcontroller. The firmware is implemented at the register level, featuring a custom SPI driver for configuring and reading the gyroscope, and a UART(DMA) interface to stream both raw and filtered data. Post-processing is performed in Python to evaluate the effectiveness of additional filtering algorithms (e.g., low-pass filtering) for noise reduction. The project avoids reliance on HAL and includes peripheral libraries developed entirely from scratch.

========================================================
Configurations and Setups:
1.	I3G4250D GPIO Configurations (SPI1):
⦁	SCK: PA5
⦁	NSS: PE3
⦁	MOSI: PA7
⦁	MISO: PA6

2.	I3G4250D Basic Technical Configurations:
⦁	Device ID: 0xD3
⦁	Output Data Rate (ODR): 200Hz
⦁	Bandwidth: 12.5Hz
⦁	Data Ready Interrupt: Enabled on INT2
⦁	SPI Mode: 4-wire interface
⦁	Full-scale Range: ±245 dps
⦁	Byte Order: BLE (LSB at low address)

3.	UART GPIO Configurations (UART1)
⦁	TX: PB6
⦁	RX: PB7
⦁	Baud Rate: 9600 bps
⦁	Parity: Odd
⦁	Word Length: 9-bit data frame

4.	DMA Configurations: DMA2 Stream 7

========================================================
Main Functions:
1.	I3G4250D
@note: Only the key functions are listed here:
⦁	i3g4250d_init(): Initializes all Basic Technical Configurations (mentioned above) of the sensor. Parameters can be modified easily to suit the application needs (e.g., ODR_200_BW_12_5 to ODR_800_BW_30 or DPS_245 to DPS_2000, etc.).
⦁	i3g4250d_calibrate(): Collects N samples to calculate and store bias value (biasX, biasY, biasZ), which are then applied as offsets to every new sensor reading.
⦁	i3g4250d_getAngularRate: Gets the current angular velocity (dps)
⦁	i3g4250d_getAngle: Gets the current estimated angle by integrating angular velocity over the sampling interval.
⦁	i3g4250d_softLPF_config(): Configures the software low-pass filter used for smoothing gyroscope readings.
⦁	i3g4250d_getAngularRate_softLPF(): Software low-pass filter helps to have a smoother values in real time.

2.	I3G4250D Helpers:
@note: Only the key functions are listed here:
⦁	i3g4250d_readRegUnsigned(): Reads an unsigned 8-bit register value from the I3G4250D and return content as uint8_t.
⦁	i3g4250d_readRegBurst(): Performs a burst read of multiple consecutive registers starting from the initial register address.
⦁	i3g4250d_writeReg(): Write a single 8-bit value to the specified register.
⦁	i3g4250d_checkRegStatus(): Verifies if a given register contains the expected value.
⦁	i3g4250d_newDataReady(): Check if the new gyroscope data is available by polling the STATUS_REG.

3.	UART
@note: Only the key functions are listed here:
⦁	UART_Init(): Initializes and configures essential UART features, including TX/RX pins, peripheral setup, baud rate, parity and word length.
⦁	UART1_DMA_Transmitter_Init(): Configures the DMA controller to handle UART1 transmission automatically.
⦁	UART1_DMA_Transmitter_Start(): Start a new DMA transfer to send a buffer of data via UART1 TX.
⦁	UART1_DMA_Transmitter_Complete(): Waits until the current DMA-driven UART transmission is finished.

==============================================
Example usage:
1.	Displaying Filtered Roll Angle On Hercules:
static float rollAngleFiltered = 0;
static float pitchAngleFiltered = 0;
static float yawAngleFiltered = 0;
static float sampleRate = 200;
int main(void){
    HAL_Init();
    UART_Init(my_GPIO_PIN_6, my_GPIO_PIN7, my_GPIOB, my_UART1, 9600, PARITY_ODD, _9B_WORDLENGTH);
    i3g4250d_init();
    i3g4250d_calibrate(1000);
    i3g4250d_softLPF_config(10, sampleRate);

 	while(1){
        uartPrintLog(my_UART1, "rollAngle: ");
        i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
        uartPrintFloat(my_UART1, rollAngleFiltered, 2);
        uartPrintLog(my_UART1, "\n");
    }
 }


2.	Send Raw and Filtered Roll Angle Readings to Hercules Using UART DMA:
static float rollAngle = 0;
static float pitchAngle = 0;
static float yawAngle = 0;
static float rollAngleFiltered = 0;
static float pitchAngleFiltered = 0;
static float yawAngleFiltered = 0;
static float sampleRate = 200;
float rawRollBuf[350];
float filteredRollBuf[350];
#define arraySize(a)	(sizeof(a)/sizeof((a)[0]))

int main(void){
 	HAL_Init();
 	UART_Init(my_GPIO_PIN_6, my_GPIO_PIN7, my_GPIOB, my_UART1, 9600, PARITY_ODD, _9B_WORDLENGTH);
    UART1_DMA_Transmitter_Init();
 	i3g4250d_init();
 	i3g4250d_calibrate(1000);
 	i3g4250d_softLPF_config(10, sampleRate);
    for(int i = 0; i < arraySize(rawRollBuf); i++){
        i3g4250d_getAngle(&rollAngle, &pitchAngle, &yawAngle, sampleRate);
        i3g4250d_getAngle_softLPF(&rollAngleFiltered, &pitchAngleFiltered, &yawAngleFiltered, sampleRate);
        rawRollBuf[i] = rollAngle;
        filteredRollBuf[i] = rollAngleFiltered;
    }
    i3g4250d_sendRollToUART(&rawRollBuf, &filteredRollBuf, arraySize(rawRollBuf));
 	
    while(1){
    }
}

==============================================
Current Challenges:
⦁ My custom-built RCC works reliably with all other peripherals; however, it causes a slight bit shift issue when reading from I3G4250D. As a temporary workaround, I am using HAL RCC instead.
