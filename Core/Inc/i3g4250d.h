/*
 * @file	i3g4250d.h
 * @brief	Register map, bit-field masks, and helper enums for the ST I3G4250D 3-axis MEMS Gyroscope
 *
 *  Created on: Jul 31, 2025
 *      Author: dobao
 */

#ifndef INC_I3G4250D_H_
#define INC_I3G4250D_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "spi.h"


#define I3G4250D_SPI_READ	0x80u

typedef enum{
	FILTER_ENABLE,
	FILTER_DISABLE,

	FIFO_ENABLE,
	FIFO_DISABLE
}initConfig_t;



/*
 * ---------------------------------------------------------
 * Core Register Address
 * ---------------------------------------------------------
 */
#define I3G4250D_WHO_AM_I		0x0FU //Read only

#define I3G4250D_CTRL_REG1		0x20U
#define I3G4250D_CTRL_REG2		0X21U
#define I3G4250D_CTRL_REG3		0x22U
#define I3G4250D_CTRL_REG4		0x23U
#define I3G4250D_CTRL_REG5		0x24U

#define I3G4250D_REFERENCE		0x25U
#define I3G4250D_OUT_TEMP		0x26U //Read
#define I3G4250D_STATUS_REG		0x27U //Read

#define I3G4250D_OUT_X_L		0x28U //Read
#define I3G4250D_OUT_X_H		0x29U //Read

#define I3G4250D_OUT_Y_L		0x2AU //Read
#define I3G4250D_OUT_Y_H		0x2BU //Read

#define I3G4250D_OUT_Z_L		0x2CU //Read
#define I3G4250D_OUT_Z_H		0x2DU //Read

#define I3G4250D_FIFO_CTRL_REG		0x2EU
#define I3G4250D_FIFO_SRC_REG		0x2FU //Read

#define I3G4250D_INT1_CFG			0x30U
#define I3G4250D_INT1_SRC			0x31U //Read

#define I3G4250D_INT1_THS_XH		0x32U
#define I3G4250D_INT1_THS_XL		0x33U

#define I3G4250D_INT1_THS_YH		0x34U
#define I3G4250D_INT1_THS_YL		0x35U

#define I3G4250D_INT1_THS_ZH		0x36U
#define I3G4250D_INT1_THS_ZL		0x37U

#define I3G4250D_DURATION			0x38U


/*
 * --------------------------------------------------------
 * CTRL_REG1 (0x20) bit-fields
 * --------------------------------------------------------
 */
#define I3G4250D_XEN			(1U << 0)
#define I3G4250D_YEN			(1U << 1)
#define I3G4250D_ZEN			(1U << 2)
#define I3G4250D_PD_ACTIVE		(1U << 3) //PD = 1 (sleep or normal)
#define I3G4250D_BW_POS			4

typedef enum{
	ODR_100_BW_12_5		= 0b00,
	ODR_100_BW_25		= 0b01,

	ODR_200_BW_12_5 	= 0b100,
	ODR_200_BW_25 		= 0b101,
	ODR_200_BW_50		= 0b110,
	ODR_200_BW_70		= 0b111,

	ODR_400_BW_20		= 0b1000,
	ODR_400_BW_25		= 0b1001,
	ODR_400_BW_50		= 0b1010,
	ODR_400_BW_110		= 0b1011,

	ODR_800_BW_30		= 0b1100,
	ODR_800_BW_35		= 0b1101,
	ODR_800_BW_50		= 0b1110,
	ODR_800_BW_110		= 0b1111
}I3G4250D_DRate_BWidth_t;


/*
 * --------------------------------------------------------
 * CTRL_REG2 (0x21) bit-fields
 * --------------------------------------------------------
 */

/* High-pass filter means signals below that frequency start being reduced */
#define I3G4250D_HPCF_POS	0	//HPCF = High-pass filter cutoff frequency
#define I3G4250D_HPM_POS	4	//HPM = High-pass Mode

typedef enum{
	HPM_NORMAL_MODE			= 0b00,
	HPM_REF_SIGNAL			= 0b01,
	HPM_RESET_ON_INTERRUPT 	= 0b11
}I3G4250D_HPM_t;

typedef enum{
	HPCF0 = 0b0000,
	HPCF1 = 0b0001,
	HPCF2 = 0b0010,
	HPCF3 = 0b0011,
	HPCF4 = 0b0100,
	HPCF5 = 0b0101,
	HPCF6 = 0b0110,
	HPCF7 = 0b0111,
	HPCF8 = 0b1000,
	HPCF9 = 0b1001
}I3G4250D_HPCF_t; //Check the I3G4250D datasheet for more details HPCF


/*
 * --------------------------------------------------------
 * CTRL_REG3 (0x22) bit-fields
 * --------------------------------------------------------
 */
#define I3G4250D_INT2_FIFO_EMPTY_DISABLE	(0U << 0)
#define I3G4250D_INT2_FIFO_EMPTY_ENABLE		(1U << 0)

#define I3G4250D_INT2_FIFO_ORUN_DISABLE		(0U << 1)
#define I3G4250D_INT2_FIFO_ORUN_ENABLE		(1U << 1)

#define I3G4250D_INT2_FIFO_WTM_DISABLE	(0U << 2)
#define I3G4250D_INT2_FIFO_WTM_ENABLE	(1U << 2)

#define I3G4250D_INT2_DRDY_DISABLE		(0U << 3)
#define I3G4250D_INT2_DRDY_ENABLE		(1U << 3)

#define I3G4250D_PUSH_PULL		(0U << 4)
#define I3G4250D_OPEN_DRAIN		(1U << 4)

#define I3G4250D_INT1_INTERRUPT_ACTIVE_HIGH	(0U << 5)
#define I3G4250D_INT1_INTERRUPT_ACTIVE_LOW	(1U << 5)

#define I3G4250D_INT1_BOOT_STATUS_DISABLE	(0U << 6)
#define I3G4250D_INT1_BOOT_STATUS_ENABLE	(1U << 6)

#define I3G4250D_INT1_DISABLE		(0U << 7)
#define I3G4250D_INT1_ENABLE		(1U << 7)


/*
 * --------------------------------------------------------
 * CTRL_REG4 (0x23) bit-fields
 * --------------------------------------------------------
 */
#define I3G4250D_SPI_4_WIRE		(0U << 0)
#define I3G4250D_SPI_3_WIRE		(1U << 0)

#define I3G4250D_BLE_LSB_AT_LOW	(0U << 6)
#define I3G4250D_BLE_MSB_AT_LOW (1U << 6)

#define I3G4250D_SELF_TEST_CONFIG_POS	1
#define I3G4250D_FULL_SCALE_SEL_POS		4

/*
 * DST = Self-test Output Change
 *
 * FS = +- 245dps -> DST = 130dps
 * FS = +- 500dps -> DST = 200dps
 * FS = +- 2000dps -> DST = 530dps
 */
typedef enum{
	SELF_TEST_NORMAL = 0b00,
	SELF_TEST_0 = 0b01,
	SELF_TEST_1 = 0b11
}Self_Test_Mode_Config_t;

typedef enum{
	DPS_245	= 0b00,
	DPS_500 = 0b01,
	DPS_2000 = 0b10
}Full_Scale_Sel_t;


/*
* --------------------------------------------------------
* CTRL_REG5 (0x24) bit-fields
* --------------------------------------------------------
*/
#define I3G4250D_OUT_SEL_POS	0
#define I3G4250D_INT1_SEL_POS	2

#define I3G4250D_HIGHPASS_DISABLE	(0U << 4)
#define I3G4250D_HIGHPASS_ENABLE	(1U << 4)

#define I3G4250D_FIFO_DISABLE	(0U << 6)
#define I3G4250D_FIFO_ENABLE	(1U << 6)

#define I3G4250D_BOOT_NORMAL	(0U << 7)
#define I3G4250D_REBOOT			(1U << 7)

typedef enum{
	LPF1 = 0b00,
	LPF1_HPF = 0b01,
	LPF1_LPF2 = 0b10,
	LPF1_HPF_LPF2 = 0b11
}Out_Sel_t;

typedef enum{
	LPF1_INT1 = 0b00,
	LPF1_HPF_INT1 = 0b01,
	LPF1_LPF2_INT1 = 0b10,
	LPF1_HPF_LPF2_INT1 = 0b11
}INT1_Sel_t;


/*
* --------------------------------------------------------
* STATUS_REG (0x27) Read Only Register
* --------------------------------------------------------
*/
#define I3G4250D_XDATA_NOTREADY	(0U << 0)
#define I3G4250D_XDATA_READY	(1U << 0) //New data for X-axis is available

#define I3G4250D_YDATA_NOTREADY	(0U << 1)
#define I3G4250D_YDATA_READY	(1U << 1) //New data for Y-axis is available

#define I3G4250D_ZDATA_NOTREADY (0U << 2)
#define I3G4250D_ZDATA_READY	(1U << 2) //New data for Z-axis is available

#define I3G4250D_ZYXDATA_NOTREADY	(0U << 3)
#define I3G4250D_ZYXDATA_READY		(1U << 3) //A new set of data is available

#define I3G4250D_XDATA_NO_ORUN	(0U << 4)
#define I3G4250D_XDATA_ORUN		(1U << 4) //New data for X-axis has overwritten the prev data

#define I3G4250D_YDATA_NO_ORUN	(0U << 5)
#define I3G4250D_YDATA_ORUN		(1U << 5) //New data for Y-axis has overwritten the prev data

#define I3G4250D_ZDATA_NO_ORUN 	(0U << 6)
#define I3G4250D_ZDATA_ORUN		(1U << 6) //New data for Z-axis has overwritten the prev data

#define I3G4250D_ZYXDATA_NO_ORUN	(0U << 7)
#define I3G4250D_ZYXDATA_ORUN		(1U << 7) //New set of data has overwritten the prev data before it was read


/*
 * --------------------------------------------------------
 * FIFO_CTRL_REG(0x2E)
 * --------------------------------------------------------
 */
#define I3G4250D_FIFO_MODE_POS	5
#define I3G4250D_FIFO_THRS_POS	0
typedef enum{
	BYPASS_MODE	= 0b000,
	FIFO_MODE 	= 0b001,
	STREAM_MODE = 0b010
}FIFO_Mode_Config_t;


/*
 * --------------------------------------------------------
 * FIFO_SRC_REG(0x2F) Read Only Register
 * --------------------------------------------------------
 */
#define I3G4250D_FIFO_EMPTY		(0U << 5)
#define I3G4250D_FIFO_NOT_EMPTY	(1U << 5)

#define I3G4250D_FIFO_NOT_FILLED	(0U << 6)
#define I3G4250D_FIFO_FILLED		(1U << 6)

#define I3G4250D_WTM_NOT_REACHED	(0U << 7)
#define I3G4250D_WTM_REACHED		(1U << 7)


/*
 * --------------------------------------------------------
 * INT1_CFG(0x30)
 * --------------------------------------------------------
 */
#define I3G4250D_XLOW_INT_DISABLE	(0U << 0)
#define I3G4250D_XLOW_INT_ENABLE	(1U << 0)

#define I3G4250D_XHIGH_INT_DISABLE	(0U << 1)
#define I3G4250D_XHIGH_INT_ENABLE	(1U << 1)

#define I3G4250D_YLOW_INT_DISABLE	(0U << 2)
#define I3G4250D_YLOW_INT_ENABLE	(1U << 2)

#define I3G4250D_YHIGH_INT_DISABLE	(0U << 3)
#define I3G4250D_YHIGH_INT_ENABLE	(1U << 3)

#define I3G4250D_ZLOW_INT_DISABLE	(0U << 4)
#define I3G4250D_ZLOW_INT_ENABLE	(1U << 4)

#define I3G4250D_ZHIGH_INT_DISABLE	(0U << 5)
#define I3G4250D_ZHIGH_INT_ENABLE	(1U << 5)

#define I3G4250D_LATCH_INT_DISABLE	(0U << 6)
#define I3G4250D_LATCH_INT_ENABLE	(1U << 6)

#define I3G4250D_OR_INT		(0U << 7)
#define I3G4250D_AND_INT	(1U << 7)


/*
 * --------------------------------------------------------
 * INT1_SRC(0x31) Read-only Register
 * --------------------------------------------------------
 */
#define I3G4250D_XLOW_OCCUR		(1U << 0)
#define I3G4250D_XHIGH_OCCUR	(1U << 1)

#define I3G4250D_YLOW_OCCUR		(1U << 2)
#define I3G4250D_YHIGH_OCCUR	(1U << 3)

#define I3G4250D_ZLOW_OCCUR		(1U << 4)
#define I3G4250D_ZHIGH_OCCUR	(1U << 5)

#define I3G4250D_IA_ACTIVE		(1U << 6)


/*
 * --------------------------------------------------------
 * INT1_DURATION(0x38)
 * --------------------------------------------------------
 */
#define I3G4250D_WAIT_DISABLE	(0U << 7)
#define I3G4250D_WAIT_ENABLE	(1U << 7)


/*
 * -------------------------------------------------------
 * Public APIs
 * -------------------------------------------------------
 */
bool i3g4250d_init(initConfig_t filterEn, initConfig_t fifoEn);
bool i3g4250d_filterInit();
bool i3g4250d_fifoInit();

#endif /* INC_I3G4250D_H_ */
