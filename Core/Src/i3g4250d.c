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

static void i3g4250d_regWrite(char regAddr, char value){
	SPI_write2Device(i3g4250d_spiConfig, regAddr, value);
}


/*
 * -------------------------------------------
 * Public APIs
 * -------------------------------------------
 */
bool i3g4250d_init(){
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



