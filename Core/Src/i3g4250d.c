/*
 * i3g4250d.c
 *
 *  Created on: Jul 31, 2025
 *      Author: dobao
 */

#include "i3g4250d.h"

/*
 * ================================================
 * DECLARATION
 * ================================================
 */

/*
 * @brief	Assign SPI/GPIO configurations of I3G4250D on STM32F411 Disco Board
 * 			(Check the schematic for pin mapping)
 */
const SPI_GPIO_Config_t i3g4250d_Config = {
		.SPIx = _SPI1,

		.sckPin = my_GPIO_PIN_5,
		.sckPort = my_GPIOA,

		.nssPin = my_GPIO_PIN_3,
		.nssPort = my_GPIOE,

		.mosiPin = my_GPIO_PIN_7,
		.mosiPort = my_GPIOA,

		.misoPin = my_GPIO_PIN_6,
		.misoPort = my_GPIOA
};

/*
 * @brief	Data container for gyroscope raw counts.
 * 			Holds unprocessed sensor readings and calibration bias values
 */
typedef struct{
	int16_t rawX; //Raw gyroscope X-axis data (counts)
	int16_t rawY;
	int16_t rawZ;

	float biasX; //Calibration bias for X-axis (counts)
	float biasY;
	float biasZ;
}gyroRawCount_t;

/*
 * @brief	Data container for gyroscope converted values
 * 			Store angular rate in dps and the sentitivty factor used
 * 			to convert raw counts to physical units
 */
typedef struct{
	float dpsX; //Converted gyroscope X-axis value (dps)
	float dpsY;
	float dpsZ;

	/*
	 * Gyroscope sensitivity (dps/LSB)
	 * Example: +-2000 dps -> 70mdps/LSB
	 * 			+-245 dps -> 8.75mdps/LSB
	 */
	float gyroSensitivity_dps_per_lsb;
}gyroConverted_t;

/*
 * Global instances of the gyroscope data containers
 */
gyroRawCount_t gyroRawCount; //Hold raw readings and bias
gyroConverted_t gyroConverted; //Hold converted dps and sensitivity

/*
 * ======================================================
 * PRIVATE HELPERS
 * ======================================================
 */

/* Read an unsigned 8-bit register from the I3G4250D */
static inline uint8_t i3g4250d_readRegUnsigned(uint8_t regAddr){
	return SPI_readRegUnsigned(&i3g4250d_Config, regAddr);
}

/* Read a signed 8-bit register from the I3G4250D */
static inline int8_t i3g4250d_readRegSigned(uint8_t regAddr){
	return SPI_readRegSigned(&i3g4250d_Config, regAddr);
}

/* Read multiple consecutive registers (burst read) */
static inline bool i3g4250d_readRegBurst(uint8_t startRegAddr, uint8_t *rxBuf, uint8_t len){
	return SPI_readRegBurst(&i3g4250d_Config, startRegAddr, rxBuf, len);
}

/* Write a value to a register */
static inline void i3g4250d_writeReg(uint8_t regAddr, uint8_t writeValue){
	SPI_writeReg(&i3g4250d_Config, regAddr, writeValue);
}

/* Check if a configured register matches the expected value */
static inline bool i3g4250d_checkRegStatus(uint8_t regAddr, uint8_t cmpVal){
	if(i3g4250d_readRegUnsigned(regAddr) == cmpVal) return true;
	else return false;
}

/* Check if new gyroscope data is available */
static inline bool i3g4250d_newDataReady(){
	return (i3g4250d_readRegUnsigned(I3G4250D_STATUS_REG) & I3G4250D_ZYXDATA_READY) != 0;
}

/* Confirm the sensor and verify if the sensor replies a correct value which has to be 0xD3 */
static inline bool i3g4250d_WHO_I_AM(){
	if(i3g4250d_readRegUnsigned(I3G4250D_WHO_AM_I) == 0xD3) return true;
	else return false; //Sensor not found
}

/* Activate GPIOs and SPI configuration for STM32 to work with I3G4250D sensor */
static void i3g4250d_gpio_spi_init(void){
	SPI_GPIO_init(&i3g4250d_Config);
	SPI_basicConfigInit(&i3g4250d_Config, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV16, SOFTWARE_SLAVE_ENABLE);
}

/*
 * @brief	Read full-scale selection (CTRL_REG4[5:4]) and return the gyroscope sensitivity
 * 			in dps/LSB. Also caches it in gyroConverted.gyroSensitivity_dps_per_lsb for later use.
 *
 * 			FS code → full scale → sensitivity
 *          00 → ±245 dps  → 8.75 mdps/LSB = 0.00875 dps/LSB
 *          01 → ±500 dps  → 17.5 mdps/LSB = 0.01750 dps/LSB
 *          10 → ±2000 dps → 70   mdps/LSB = 0.07000 dps/LSB
 *          11 → Reserved on some docs; commonly treated same as ±2000 dps.
 *               This driver maps 11 → 0.07000 dps/LSB for robustness.
 */
static inline float i3g4250d_getSensitivity(void){
	const uint8_t dpsValExtractedFromReg4 = (i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG4) >> I3G4250D_FULL_SCALE_SEL_POS) & 0b11;
	static const float sensitivityArray[4]  = {
			0.00875f,
			0.0175f,
			0.07f,
			0.07f
	};
	const float sensitivity = sensitivityArray[dpsValExtractedFromReg4];
	return (gyroConverted.gyroSensitivity_dps_per_lsb = sensitivity);
}

/*
 * @brief	Configure CTRL_REG1:
 * 				Enabling X, Y, Z axis
 * 				Selecting power-down mode
 * 				Selecting output data rate
 */
static inline bool i3g4250d_reg1Config(I3G4250D_DRate_BWidth_t ODR_BW){
	uint8_t reg1 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG1);
	/* Clear bits before setting */
	reg1 &= ~((uint8_t)(0xFu << I3G4250D_BW_POS));

	/* Set new values */
	reg1 |= (uint8_t)I3G4250D_XEN |
			(uint8_t)I3G4250D_YEN |
			(uint8_t)I3G4250D_ZEN |
			(uint8_t)I3G4250D_PD_ACTIVE |
			(uint8_t)(ODR_BW << I3G4250D_BW_POS); //Config ODR and BW in oneline

	i3g4250d_writeReg(I3G4250D_CTRL_REG1, reg1); //write reg1 to actual CTRL_REG1 of I3G4250D
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG1, reg1)) return false; //Verify if CTRL_REG1 is configured okay
	return true;
}

/*
 * @brief	Configure CTRL_REG2:
 * 				Highpass Cutoff Frequency
 * 				Highpass Mode
 * 					HPM_NORMAL_MODE			= 0b00,
 * 					HPM_REF_SIGNAL			= 0b01,
 * 					HPM_RESET_ON_INTERRUPT 	= 0b11
 */
static inline bool i3g4250d_reg2Config(I3G4250D_HPCF_t highPassCutoffFreq, I3G4250D_HPM_t highPassMode){
	uint8_t reg2 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG2);
	/* Clear bits before setting */
	reg2 &= ~((uint8_t)((0xFu << I3G4250D_HPCF_POS) | (0x3u << I3G4250D_HPM_POS)));

	/* Set new values */
	reg2 |= (uint8_t)(highPassCutoffFreq << I3G4250D_HPCF_POS) |
			(uint8_t)(highPassMode << I3G4250D_HPM_POS);

	i3g4250d_writeReg(I3G4250D_CTRL_REG2, reg2);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG2, reg2)) return false;
	return true;
}

/*
 * @brief	Configure CTRL_REG3
 * 				Enable data ready interrupt on INT2
 * 				read-modify-write: preserves current polarity/driver and any other interrupt sources already set in CTRL_REG3
 * 				To add more sources, OR with their *_ENABLE masks
 * 				To disable a source, clear its bit with '&= ~MASK'
 */
static inline bool i3g4250d_reg3Config(void){
	uint8_t reg3 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG3);
	reg3 |= (uint8_t)I3G4250D_INT2_DRDY_ENABLE; //Enable DRDY on INT2; OR more *_ENABLE bits if needed
	i3g4250d_writeReg(I3G4250D_CTRL_REG3, reg3);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG3, reg3)) return false;
	return true;
}

/*
 * @brief	Configure CTRL_REG4
 */
static inline bool i3g4250d_reg4Config(SPI_serial_t spiSerial,
									   Self_Test_Mode_Config_t selfTestMode,
									   Full_Scale_Sel_t fullScaleSel,
									   BLE_t bleSel){
	uint8_t reg4 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG4);
	/* Clear bits before setting */
	reg4 &= ~((uint8_t)((0x1u << I3G4250D_SPI_INTERFACE_POS) |
			(0x3u << I3G4250D_SELF_TEST_CONFIG_POS) |
			(0x3u << I3G4250D_FULL_SCALE_SEL_POS) |
			(1u << I3G4250D_BLE_POS)));

	reg4 |= (uint8_t)(spiSerial << I3G4250D_SPI_INTERFACE_POS) |
			(uint8_t)(selfTestMode << I3G4250D_SELF_TEST_CONFIG_POS) |
			(uint8_t)(fullScaleSel << I3G4250D_FULL_SCALE_SEL_POS) |
			(uint8_t)(bleSel << I3G4250D_BLE_POS);
	i3g4250d_writeReg(I3G4250D_CTRL_REG4, reg4);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG4, reg4)) return false;
	return true;
}

/*
 * @brief	Configure CTRL_REG5
 */
static inline bool i3g4250d_reg5Config(Out_Sel_t outSel,
									   INT1_Sel_t int1Sel,
									   HPF_Enable_t hpfEn,
									   FIFO_Enable_t fifoEn){
	uint8_t reg5 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5);
	/* Clear bits before setting */
	reg5 &= ~((uint8_t)(0x3u << I3G4250D_OUT_SEL_POS) |
			  (uint8_t)(0x3u << I3G4250D_INT1_SEL_POS) |
			  (uint8_t)(1u << I3G4250D_HPF_POS) |
			  (uint8_t)(1u << I3G4250D_FIFO_POS));

	/* Set new values */
	reg5 |= (uint8_t)(outSel << I3G4250D_OUT_SEL_POS) |
			(uint8_t)(int1Sel << I3G4250D_INT1_SEL_POS) |
			(uint8_t)(hpfEn << I3G4250D_HPF_POS) |
			(uint8_t)(fifoEn << I3G4250D_FIFO_POS);
	i3g4250d_writeReg(I3G4250D_CTRL_REG5, reg5);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, reg5)) return false;
	return true;
}


/*
 * ======================================================
 * PUBLIC HELPERS
 * ======================================================
 */

/*
 * @brief	Initialize I3G4250D
 *
 * @note	This initialization is configured for serving angle estimation therefore:
 * 				* +-245dps
 * 				* 200Hz ODR and 50Hz cutoff
 */
bool i3g4250d_init(){
	/* Initialize SPI and GPIO for I3G4250D and STM32 */
	i3g4250d_gpio_spi_init();
	HAL_Delay(15);

	/* Check if sensor is found */
	if(i3g4250d_WHO_I_AM() == false) return false;

	/* Simple timeout for waiting I3G4250D to reboot */
	uint32_t t0 = HAL_GetTick();
	while(i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5) & I3G4250D_REBOOT){
		if((HAL_GetTick() - t0) > 30) break;
		HAL_Delay(1);
	}
	HAL_Delay(20); //Wait for finishing internal settling after rebooting, delete this delay will stop the sensor from working

	/* Configure CTRL_REG1 */
	if(!i3g4250d_reg1Config(ODR_200_BW_12_5)) return false; //Choose lower BW to have lower noise

	/* Configure CTRL_REG3 */
	if(!i3g4250d_reg3Config()) return false;

	/* Configure CTRL_REG4 */
	if(!i3g4250d_reg4Config(SPI_4WIRE, SELF_TEST_NORMAL, DPS_245, BLE_LSB_AT_LOW)) return false;

	/* Clear any pending INT1 source flags and reset latches */
	(void)i3g4250d_readRegUnsigned(I3G4250D_INT1_SRC);

	return true;
}

/*
 * @brief	Enable the I3G4250D high-pass filter and route it into the output path
 *
 * Example setup:
 * 		CTRL_REG2: sets HPM = NORMAL and HPCF = HPCF5 (adjust if you need)
 * 		CTRL_REG5: sets HPEN and updates OUT_SEL to include HPF
 * 			* LPF1 -> LPF1_HPF
 * 			* LPF1_LPF2 -> LPF1_HPF_LPF2
 */
bool i3g4250d_HPF_enable(I3G4250D_HPCF_t highPassCutoffFreq,
						 I3G4250D_HPM_t highPassMode,
						 Out_Sel_t outSel,
						 INT1_Sel_t int1Sel){
	if(!i3g4250d_reg2Config(highPassCutoffFreq, highPassMode)) return false;

	/* Enable HPF in CTRL_REG5 and ensure OUT_SEL includes HPF */
	if((outSel == LPF1) || (outSel == LPF1_HPF)) outSel = LPF1_HPF;
	else if((outSel == LPF1_LPF2) || (outSel == LPF1_HPF_LPF2))	outSel = LPF1_HPF_LPF2;

	/* Ensure INT1 generator path includes HPF */
	if((int1Sel == LPF1_INT1) || (int1Sel == LPF1_HPF_INT1)) int1Sel = LPF1_HPF_INT1;
	else if((int1Sel == LPF1_LPF2_INT1) || (int1Sel == LPF1_HPF_LPF2_INT1)) int1Sel = LPF1_HPF_LPF2_INT1;

	/* Ensure FIFO is preserved */
	uint8_t reg5 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5);
	FIFO_Enable_t fifoEn = ((reg5 >> I3G4250D_FIFO_POS) & 1u);

	/* Apply CTRL_REG5: OUT_SEL, INT1_SEL, HPF enable, (FIFO unchanged here) */
	if(!i3g4250d_reg5Config(outSel, int1Sel, HPF_ENABLE, fifoEn)) return false;

	/* Enable INT1 */
	uint8_t reg3 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG3);
	reg3 |= I3G4250D_INT1_ENABLE; //Enable INT1
	i3g4250d_writeReg(I3G4250D_CTRL_REG3, reg3);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG3, reg3)) return false;

	/* Note: you must also set INT1_CFG/THS/DURATION to actually generate INT1 */
	return true;
}

/*
 * @brief	Enable LPF2 in the signal chain
 *
 * @behavior
 * 			OUT path:	LPF1	 -> LPF1_LPF2
 * 						LPF1_HPF -> LPF1_HPF_LPF2
 * 						(already including LPF2 stays unchanged)
 *
 * 			INT1 path:	LPF1_INT1	-> LPF1_LPF2_INT1
 * 						LPF1_HPF_INT1 -> LPF1_HPF_LPF2_INT1
 * 						(already including LPF2 stays unchanged)
 * @note: Does not touch HPF Enable (CTRL_REG5 bit 4) or FIFO Enable (bit 6)
 * 		  LPF1 cutoff is governed by ODR/BW in CTRL_REG1. This helper only routes LPF2 int the chain
 */
bool i3g4250d_LPF_enable(void){
	uint8_t reg5 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5);

	/* Read current selection */
	uint8_t out_sel = (uint8_t)((reg5 >> I3G4250D_OUT_SEL_POS) & 0x3u);
	uint8_t int1_sel = (uint8_t)((reg5 >> I3G4250D_INT1_SEL_POS) & 0x3u);

	/* Upgrade the OUT path to include LPF2 preserving whether HPF is in the path */
	if(out_sel == (uint8_t)LPF1) out_sel = (uint8_t)LPF1_LPF2;
	else if(out_sel == (uint8_t)LPF1_HPF) out_sel = (uint8_t)LPF1_HPF_LPF2;
	/* LPF1_LPF2 / LPF1_HPF_LPF2 already included LPF2 in the path so no need to change */

	/* Upgrade the INT1 path to include LPF2 preserving whether HPF is in the path */
	if(int1_sel == (uint8_t)LPF1_INT1) int1_sel = LPF1_LPF2_INT1;
	else if(int1_sel == (uint8_t) LPF1_HPF_INT1) int1_sel = LPF1_HPF_LPF2_INT1;
	/* LPF1_LPF2_INT1 and LPF1_HPF_LPF2_INT1 included LPF2 so no change needed */

	/* Write back: mask-and-set the two fields only */
	reg5 &= (uint8_t) ~(((uint8_t)(0x3u << I3G4250D_OUT_SEL_POS)) |
			((uint8_t)(0x3u << I3G4250D_INT1_SEL_POS))); //Clear the bits

	reg5 |= (uint8_t)((uint8_t)(out_sel << I3G4250D_OUT_SEL_POS)) |
			((uint8_t)(int1_sel << I3G4250D_INT1_SEL_POS));

	i3g4250d_writeReg(I3G4250D_CTRL_REG5, reg5);
	return i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, reg5);
}

/*
 * @brief	Burst-read six data bytes
 * 			Read X/Y/Z axis raw values (2 bytes each) into gyroRawCount
 */
static bool i3g4250d_readRawData(){
	while(!i3g4250d_newDataReady()); //Wait until there is new XYZ values
	uint8_t buf[6];
	if(!i3g4250d_readRegBurst(I3G4250D_OUT_X_L, buf, sizeof buf)) return false;

	gyroRawCount.rawX = (int16_t) ((buf[1] << 8) | buf[0]);
	gyroRawCount.rawY = (int16_t) ((buf[3] << 8) | buf[2]);
	gyroRawCount.rawZ = (int16_t) ((buf[5] << 8) | buf[4]);

	return true;
}

/*
 * @brief	Calibrate the sensor to find out the biasX, biasY, biasZ
 * 			Averages N samples to compute bias for X/Y/Z axes
 */
bool i3g4250d_calibrate(uint16_t sampleCount){
	if(sampleCount == 0) return false;

	int sumX = 0, sumY = 0, sumZ = 0;
	/* Discard a few samples after enabling sensor */
	for(uint16_t i = 0; i < sampleCount; i++){
		while(!i3g4250d_newDataReady()); //Wait for new data
		if(i3g4250d_readRawData()){
			sumX += gyroRawCount.rawX;
			sumY += gyroRawCount.rawY;
			sumZ += gyroRawCount.rawZ;
		}
		HAL_Delay(4); //a small delay allows sensor to update
	}
	gyroRawCount.biasX = (float)sumX / (float)sampleCount;
	gyroRawCount.biasY = (float)sumY / (float)sampleCount;
	gyroRawCount.biasZ = (float)sumZ / (float)sampleCount;
	return true; //Calibrate process is success
}

/*
 * @brief	Convert raw data to angular velocity in dps
 * 			Subtracts bias and applies sensitivity factor
 */
static bool i3g4250d_convertToDps(void){
	if(!i3g4250d_readRawData()) return false;
	const float k = i3g4250d_getSensitivity();
	gyroConverted.dpsX = (gyroRawCount.rawX - gyroRawCount.biasX) * k;
	gyroConverted.dpsY = (gyroRawCount.rawY - gyroRawCount.biasY) * k;
	gyroConverted.dpsZ = (gyroRawCount.rawZ - gyroRawCount.biasZ) * k;
	return true;
}

/*
 * @brief	Get current angular velocity (dps)
 * 			Outputs X, Y, Z angular rates to user pointer
 */
bool i3g4250d_getAngularRate(float *angularX, float *angularY, float *angularZ){
	if(!i3g4250d_convertToDps()) return false;
	*angularX = gyroConverted.dpsX;
	*angularY = gyroConverted.dpsY;
	*angularZ = gyroConverted.dpsZ;
	return true;
}

void i3g4250d_getAngle(float *rollAngle, float *pitchAngle, float *yawAngle, float sampleRate){
	float dt = 1.0f / sampleRate;
	float angularX, angularY, angularZ;
	if(!i3g4250d_getAngularRate(&angularX, &angularY, &angularZ)) return;

	*rollAngle += angularX * dt; //Unit is degree
	*pitchAngle += angularY * dt;
	*yawAngle += angularZ * dt;

	if(*rollAngle > 180) 	*rollAngle -= 360;
	if(*rollAngle < -180) 	*rollAngle += 360;

	if(*pitchAngle > 180) 	*pitchAngle -= 360;
	if(*pitchAngle < -180) 	*pitchAngle += 360;

	if(*yawAngle > 180) 	*yawAngle -= 360;
	if(*yawAngle < -180) 	*yawAngle += 360;
}

/*
 * ====================================================
 * SOFTWARE LOWPASS FILTER
 * ====================================================
 */
static float lowPassFilter_X = 0;
static float lowPassFilter_Y = 0;
static float lowPassFilter_Z = 0;

static bool lowPassFilterInit = false;
static float lowPassFilterAlpha = 0.0f;

static inline float getAlpha(float cutoffFreq, float sampleHz){
	const float dt = 1.0f/sampleHz;
	return 1.0f - expf(-2.0f * 3.14159f * cutoffFreq * dt);
}

void i3g4250d_softLPF_config(float cutoffFreq, float sampleHz){
	lowPassFilterAlpha = getAlpha(cutoffFreq, sampleHz);
	lowPassFilterInit = false;
}

/*
 * @brief	This applies: y_k = y_{k-1} + alpha * (x_k - y_{k-1})
 * 			newVal		: x_k, the newest raw sample (unfiltered)
 * 			currentVal	: pointer to the filter state y_{k-1}; updated in-place to y_k
 *
 * 			Uses two globals you define elsewhere:
 * 				lowPassFilterAlpha [0,1] -> bigger -> more responsive; smaller -> smoother
 * 				lowPassFilterInit		 -> false on first call; true after seeding
 */
static inline float softLowPassFilter(float newVal, float *currentVal){
	//If this is the very first sample, initializa current value to the new input
	//So the first output equals to the first input (avoid a big first-step jump)
	if(!lowPassFilterInit) *currentVal = newVal;
	*currentVal = *currentVal + lowPassFilterAlpha * (newVal - *currentVal);
	return *currentVal;
}

/*
 * @brief	Call this instead of i3g4250d_angularVelocity when you want smoothed
 */
bool i3g4250d_getAngularRate_softLPF(float *angularX, float *angularY, float *angularZ){
	float newAngularX, newAngularY, newAngularZ;
	if(!i3g4250d_getAngularRate(&newAngularX, &newAngularY, &newAngularZ)) return false;

	*angularX = softLowPassFilter(newAngularX, &lowPassFilter_X);
	*angularY = softLowPassFilter(newAngularY, &lowPassFilter_Y);
	*angularZ = softLowPassFilter(newAngularZ, &lowPassFilter_Z);

	lowPassFilterInit = true;
	return true;
}

void i3g4250d_getAngle_softLPF(float *rollAngle, float *pitchAngle, float *yawAngle, float sampleRate){
	float dt = 1.0f / sampleRate;
	float angularX, angularY, angularZ;
	if(!i3g4250d_getAngularRate_softLPF(&angularX, &angularY, &angularZ)) return;

	*rollAngle += angularX * dt; //Unit is degree
	*pitchAngle += angularY * dt;
	*yawAngle += angularZ * dt;

	if(*rollAngle > 180) 	*rollAngle -= 360;
	if(*rollAngle < -180) 	*rollAngle += 360;

	if(*pitchAngle > 180) 	*pitchAngle -= 360;
	if(*pitchAngle < -180) 	*pitchAngle += 360;

	if(*yawAngle > 180) 	*yawAngle -= 360;
	if(*yawAngle < -180) 	*yawAngle += 360;
}

/*
 * ==============================================================
 * SENDING DATA TO UART FOR PLOTTING GRAPH
 * ==============================================================
 */
void i3g4250d_sendRollToUART(const float *rawRoll, const float *filteredRoll, uint32_t sampleCount){
	const char header[] = "i, rawRoll, filteredRoll\r\n";
	UART1_DMA_Transmitter_Start(header, sizeof(header)-1); //Send every visible character except the terminating NUL byte
	UART1_DMA_Transmitter_Complete(); //Blocks until DMA is actually done

	char line[64];
	for(uint32_t i = 0; i < sampleCount; i++){ //Loop over all sample
		// %lu: format unsigned long
		// %.4f: printfs four decimal points
		int len = snprintf(line, sizeof line, "%lu,%.4f,%.4f\r\n", (unsigned long)i, (double)rawRoll[i], (double)filteredRoll[i]);
		UART1_DMA_Transmitter_Start(line, (uint32_t)len); //txBuffer and bufferSize
		UART1_DMA_Transmitter_Complete();
	}
}






