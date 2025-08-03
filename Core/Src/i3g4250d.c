/*
 * i3g4250d.c
 *
 *  Created on: Jul 31, 2025
 *      Author: dobao
 */

#include "i3g4250d.h"

/*
 * @brief	SPI configuration of I3G4250D on STM32F411 DISCO board
 */
SPI_GPIO_Config_t i3g4250d_spiConfig = {
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

typedef struct{
	int16_t rawX, rawY, rawZ;
	int16_t	biasX, biasY, biasZ;
	float sensitivity_mdps;
	float dpsX, dpsY, dpsZ;
}Gyro_Context_t;

static Gyro_Context_t gyroContext; //Global instance

static float rollAngle = 0, pitchAngle = 0, yawAngle = 0;

/*
 * -------------------------------------------
 * Help functions
 * -------------------------------------------
 */
static void i3g4250d_gpioInit(void){
	SPI_GPIO_init(i3g4250d_spiConfig);
	SPI_basicConfigInit(i3g4250d_spiConfig, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV16, SOFTWARE_SLAVE_ENABLE, SPI_ENABLE);
}

static char i3g4250d_regRead(char regAddr){
	return SPI_readReceivedData(i3g4250d_spiConfig, regAddr);
}

static bool i3g4250d_regReadBurst(uint8_t startAddr, uint8_t *buf, uint8_t len){
	return SPI_readBurstBuf(i3g4250d_spiConfig, startAddr, buf, len);
}

static void i3g4250d_regWrite(char regAddr, char value){
	SPI_write2Device(i3g4250d_spiConfig, regAddr, value);
}

/*
 * -------------------------------------------
 * Public APIs
 * -------------------------------------------
 */
bool i3g4250d_init(){
	gyroContext.sensitivity_mdps = 70; //Check mechanical characteristic section datasheet of I3G4250D

	bool retVal = false;
	uint8_t temp = 0;

	i3g4250d_gpioInit();

	/* Verify chip identity	 */
	temp = i3g4250d_regRead(I3G4250D_WHO_AM_I);
	if(temp != 0xD3) goto exit; //Sensor not found

	/* Software reboot */
	i3g4250d_regWrite(I3G4250D_CTRL_REG5, I3G4250D_REBOOT);
	HAL_Delay(5);

	/* CTRL_REG1: PD = 1,  ODR = 800, Cutoff = 35Hz, enable all axes*/
	temp = I3G4250D_PD_ACTIVE | (ODR_800_BW_35 << I3G4250D_BW_POS) | I3G4250D_XEN | I3G4250D_YEN | I3G4250D_ZEN;
	i3g4250d_regWrite(I3G4250D_CTRL_REG1, temp);

	/* CTRL_REG4: +- 2000dps, 4-wire SPI, LSB @ lower address */
	temp = I3G4250D_SPI_4_WIRE | (DPS_2000 << I3G4250D_FULL_SCALE_SEL_POS) | I3G4250D_BLE_LSB_AT_LOW;
	i3g4250d_regWrite(I3G4250D_CTRL_REG4, temp);

	/* CTRL_REG3: route DRDY to INT2 */
	temp = I3G4250D_INT2_DRDY_ENABLE;
	i3g4250d_regWrite(I3G4250D_CTRL_REG3, temp);

	/*
	 * CTRL_REG5: disable HPF and boot normal
	 */
	temp = I3G4250D_HIGHPASS_DISABLE | I3G4250D_FIFO_DISABLE | I3G4250D_BOOT_NORMAL;
	i3g4250d_regWrite(I3G4250D_CTRL_REG5, temp);

	/*
	 * Clear any pending INT1 source flags by reading INT1_SRC -> reset latches
	 */
	(void)i3g4250d_regRead(I3G4250D_INT1_SRC);

	if(i3g4250d_regRead(I3G4250D_CTRL_REG1) != (I3G4250D_PD_ACTIVE |
											   (ODR_800_BW_35 << I3G4250D_BW_POS) |
											   I3G4250D_XEN | I3G4250D_YEN | I3G4250D_ZEN)) goto exit;
	/* Success */
	retVal = true;

exit:
	return retVal;
}


/*
 * @brief	Burst-read six data bytes
 */
bool i3g4250d_readRawData(){
	uint8_t buf[6];
	if(!i3g4250d_regReadBurst(I3G4250D_OUT_X_L, buf, 6)) {
		return false;
	}

	gyroContext.rawX = (int16_t) ((buf[1] << 8) | buf[0]);
	gyroContext.rawY = (int16_t) ((buf[3] << 8) | buf[2]);
	gyroContext.rawZ = (int16_t) ((buf[5] << 8) | buf[4]);
	return true;
}


/*
 * @brief	Calibrate the sensor to find out the biasX, biasY, biasZ
 */
void i3g4250d_calibrate(uint16_t sampleCount){
	int32_t sumX = 0, sumY = 0, sumZ = 0;
	for(uint16_t i = 0; i < sampleCount; i++){
		if(i3g4250d_readRawData()){
			sumX += gyroContext.rawX;
			sumY += gyroContext.rawY;
			sumZ += gyroContext.rawZ;
		}
		HAL_Delay(4);
	}
	gyroContext.biasX = sumX / sampleCount;
	gyroContext.biasY = sumY / sampleCount;
	gyroContext.biasZ = sumZ / sampleCount;
}


/*
 * @brief	Convert raw counts (rawX,Y,Z) to degree per sec (dps)
 */
static void i3g4250d_convertRawToDps(void){
	const float k = gyroContext.sensitivity_mdps * 0.001f; //Convert mdps to dps
	gyroContext.dpsX = (gyroContext.rawX - gyroContext.biasX) * k;
	gyroContext.dpsY = (gyroContext.rawY - gyroContext.biasY) * k;
	gyroContext.dpsZ = (gyroContext.rawZ - gyroContext.biasZ) * k;
}


/*
 *
 */
bool i3g4250d_getDps(float *xDps, float *yDps, float *zDps){
	if(!i3g4250d_readRawData()) return false;

	i3g4250d_convertRawToDps();
	*xDps = gyroContext.dpsX;
	*yDps = gyroContext.dpsY;
	*zDps = gyroContext.dpsZ;
	return true;
}


void i3g4250d_updateAngle(void){
	static uint32_t lastTick = 0;
	uint32_t currentTick = HAL_GetTick();

	if(lastTick == 0){
		lastTick = currentTick;
		return;
	}

	float dt = (currentTick - lastTick) * 0.001f; //ms -> s
	lastTick = currentTick;

	float gx, gy, gz;
	if(!i3g4250d_getDps(&gx, &gy, &gz)) return;

	rollAngle = rollAngle + (gx * dt);
	pitchAngle = pitchAngle + (gy * dt);
	yawAngle = yawAngle + (gz * dt);

	if(rollAngle > 180) rollAngle -= 360;
	if(rollAngle < -180) rollAngle += 360;

	if(pitchAngle > 180) pitchAngle -= 360;
	if(pitchAngle < -180) pitchAngle += 360;

	if(yawAngle > 180) yawAngle -= 360;
	if(yawAngle < -180) yawAngle += 360;
}













