/*
 * i3g4250d.c
 *
 *  Created on: Jul 31, 2025
 *      Author: dobao
 */

#include "i3g4250d.h"

/*
 * Assign suitable SPI/GPIO configurations of I3G4250D on STM32F411 Disco board
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

typedef struct {
	uint16_t rawX, rawY, rawZ;
	float gyroSensitivity;
}gyroRawCnt_t;

gyroRawCnt_t gyroRaw;

/*
 * =============================================================
 * Private Helpers
 * =============================================================
 */
static inline uint8_t i3g4250d_readRegUnsigned(uint8_t regAddr){
	return SPI_readRegUnsigned(&i3g4250d_Config, regAddr);
}

static inline int8_t i3g4250d_readRegSigned(uint8_t regAddr){
	return SPI_readRegSigned(&i3g4250d_Config, regAddr);
}

static inline void i3g4250d_readRegBurst(uint8_t startRegAddr, uint8_t *rxBuf, uint8_t len){
	SPI_readRegBurst(&i3g4250d_Config, startRegAddr, rxBuf, len);
}

static inline void i3g4250d_writeReg(uint8_t regAddr, uint8_t writeValue){
	SPI_writeReg(&i3g4250d_Config, regAddr, writeValue);
}

static inline bool i3g4250d_checkRegStatus(uint8_t regAddr, uint8_t cmpVal){
	if(i3g4250d_readRegUnsigned(regAddr) == cmpVal) return true;
	else return false;
}


/*
 * =============================================================
 * Public Helpers
 * =============================================================
 */

/*
 * Generally initialize all SPI features/configurations for I3G4250D to work
 */
void i3g4250dGPIO_init(void){
	SPI_GPIO_init(&i3g4250d_Config);
	SPI_basicConfigInit(&i3g4250d_Config, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV16, SOFTWARE_SLAVE_ENABLE);
}


bool i3g4250d_init(initConfig_t filterEn, initConfig_t fifoEn){
	bool retVal = false;
	gyroRaw.gyroSensitivity = 70 * 0.001; //70mdps to 0.07dps

	/* Initialize SPI for I3G4250D */
	i3g4250dGPIO_init();

	/* Confirm the sensor and verify if the sensor replies a correct value which has to be 0xD3 */
	uint8_t id = i3g4250d_readRegUnsigned(I3G4250D_WHO_AM_I);
	if(id != 0xD3) goto exit; //Sensor is not found

	i3g4250d_writeReg(I3G4250D_CTRL_REG5, I3G4250D_REBOOT); //Reboot memory content
	while(i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5) & I3G4250D_REBOOT); //Wait until sensor reboot process completely done

	/*
	 * Configure CTRL_REG1
	 * Enable XYZ, Enable Power, ODR = 800Hz, BandWidth = 30Hz
	 */
	uint8_t ctrlReg1 = I3G4250D_XEN |
					   I3G4250D_YEN |
					   I3G4250D_ZEN |
					   I3G4250D_PD_ACTIVE |
					   (ODR_800_BW_30 << I3G4250D_BW_POS);
	i3g4250d_writeReg(I3G4250D_CTRL_REG1, ctrlReg1);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG1, ctrlReg1)) goto exit;

	/* Configure CTRL_REG4 */
	uint8_t ctrlReg4 = I3G4250D_SPI_4_WIRE |
					   (DPS_2000 << I3G4250D_FULL_SCALE_SEL_POS) |
					   I3G4250D_BLE_LSB_AT_LOW;
	i3g4250d_writeReg(I3G4250D_CTRL_REG4, ctrlReg4);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG4, ctrlReg4)) goto exit;

	if(filterEn == FILTER_ENABLE){
		/*
		 * CTRL_REG2
		 * HPM -> Normal mode
		 * High-pass cutoff freq = 0.5Hz
		 */
		uint8_t ctrlReg2 = (HPM_NORMAL_MODE << I3G4250D_HPM_POS) |
						   (HPCF7 << I3G4250D_HPCF_POS);
		i3g4250d_writeReg(I3G4250D_CTRL_REG2, ctrlReg2);
	}

	/*
	 * Config CTRL_REG5
	 * 		HPF route + FIFO enable
	 */
	uint8_t ctrlReg5 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5);
	uint8_t ctrlReg3 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG3);

	if(filterEn == FILTER_ENABLE){
		ctrlReg5 |= (LPF1_HPF << I3G4250D_OUT_SEL_POS) |
					(LPF1_HPF_INT1 << I3G4250D_INT1_SEL_POS) |
					(I3G4250D_HIGHPASS_ENABLE);
	}
	if(fifoEn == FIFO_ENABLE){
		ctrlReg5 |= I3G4250D_FIFO_ENABLE;
		ctrlReg3 |= I3G4250D_INT2_FIFO_WTM_ENABLE;
	}
	i3g4250d_writeReg(I3G4250D_CTRL_REG5, ctrlReg5);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, ctrlReg5)) goto exit;

	/*
	 * Configure CTRL_REG3
	 * Data ready on DRDY/INT2 enable, Enable interrupt on INT1 pin
	 */
	i3g4250d_writeReg(I3G4250D_CTRL_REG3, ctrlReg3);

	(void)i3g4250d_readRegUnsigned(I3G4250D_REFERENCE);
	HAL_Delay(2); // 1/ 800Hz = 0.00125s

	retVal = true;
exit:
	return retVal;
}
