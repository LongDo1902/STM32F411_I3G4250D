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
	int16_t rawX, rawY, rawZ;
	float biasX, biasY, biasZ;
	float dpsX, dpsY, dpsZ; //dps must handle negative
	float gyroSensitivity_dps_per_lsb; //+-2000dps -> 70mdps/LSB, +-245dps -> 8.75mdps/LSB
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

static inline bool i3g4250d_readRegBurst(uint8_t startRegAddr, uint8_t *rxBuf, uint8_t len){
	return SPI_readRegBurst(&i3g4250d_Config, startRegAddr, rxBuf, len);
}

static inline void i3g4250d_writeReg(uint8_t regAddr, uint8_t writeValue){
	SPI_writeReg(&i3g4250d_Config, regAddr, writeValue);
}

static inline bool i3g4250d_checkRegStatus(uint8_t regAddr, uint8_t cmpVal){
	if(i3g4250d_readRegUnsigned(regAddr) == cmpVal) return true;
	else return false;
}

static inline bool i3g4250d_newDataReady(){
	return (i3g4250d_readRegUnsigned(I3G4250D_STATUS_REG) & I3G4250D_ZYXDATA_READY) != 0;
}

/*
 * =============================================================
 * Public Helpers
 * =============================================================
 */

/*
 * Generally initialize all SPI features/configurations for I3G4250D to work
 */
static void i3g4250dGPIO_init(void){
	SPI_GPIO_init(&i3g4250d_Config);
	SPI_basicConfigInit(&i3g4250d_Config, STM32_AS_MASTER, DFF_8BITS, FPCLK_DIV16, SOFTWARE_SLAVE_ENABLE);
}

/*
 * @brief	Basic I3G4250D Init Helper
 */
bool i3g4250d_init(){
	bool ok = false;
	gyroRaw.gyroSensitivity_dps_per_lsb = 8.75 * 0.001f; //70mdps to 0.07dps

	/* Initialize SPI for I3G4250D */
	i3g4250dGPIO_init();
	HAL_Delay(10);

	/* Confirm the sensor and verify if the sensor replies a correct value which has to be 0xD3 */
	if(i3g4250d_readRegUnsigned(I3G4250D_WHO_AM_I) != 0xD3) goto exit; //Sensor not found

	i3g4250d_writeReg(I3G4250D_CTRL_REG5, I3G4250D_REBOOT); //Reboot memory content
	HAL_Delay(5);

	/*
	 * Simple timeout so we cannot hang forever if a read returns junk
	 */
	uint32_t t0 = HAL_GetTick();
	while(i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5) & I3G4250D_REBOOT){//Wait until sensor reboot process completely done
		if((HAL_GetTick() - t0) > 30) break;
		HAL_Delay(1);
	}

	HAL_Delay(20); //Wait for finishing internal settling after rebooting

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

	uint8_t ctrlReg3 = I3G4250D_INT2_DRDY_ENABLE;
	i3g4250d_writeReg(I3G4250D_CTRL_REG3, ctrlReg3);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG3, ctrlReg3)) goto exit;

	/* Configure CTRL_REG4 */
	uint8_t ctrlReg4 = (I3G4250D_SPI_4_WIRE) | (DPS_245 << I3G4250D_FULL_SCALE_SEL_POS) | I3G4250D_BLE_LSB_AT_LOW;
	i3g4250d_writeReg(I3G4250D_CTRL_REG4, ctrlReg4);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG4, ctrlReg4)) goto exit;

	/*
	 * Clear any pending INT1 source flags by reading INT1_SRC -> reset latches
	 */
	(void)i3g4250d_readRegUnsigned(I3G4250D_INT1_SRC);

	ok = true;
exit:
	return ok;
}

/*
 * @brief	Enable High-Pass Filter and FIFO
 */
bool i3g4250d_enable_HPF(void){
	/*
	 * CTRL_REG2
	 * HPM -> Normal mode
	 * High-pass cutoff freq = 0.5Hz
	 */
	uint8_t ctrlReg2 = (HPM_NORMAL_MODE << I3G4250D_HPM_POS) | (HPCF7 << I3G4250D_HPCF_POS);
	i3g4250d_writeReg(I3G4250D_CTRL_REG2, ctrlReg2);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG2, ctrlReg2)) return false;

	/*
	 * Configure CTRL_REG5
	 * HPF route + FIFO enable
	 */
	uint8_t ctrlReg5 = (LPF1_HPF << I3G4250D_OUT_SEL_POS) | (LPF1_HPF_INT1 << I3G4250D_INT1_SEL_POS) |	(I3G4250D_HIGHPASS_ENABLE);
	i3g4250d_writeReg(I3G4250D_CTRL_REG5, ctrlReg5);
	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, ctrlReg5)) return false;

	//If HPF used, read REFERENCE once
	(void)i3g4250d_readRegUnsigned(I3G4250D_REFERENCE);
	HAL_Delay(3); // 1/ 800Hz = 0.00125s

	return true;
}

bool i3g4250d_route_LPF(void){
	//Preserve other CTRL_REG5 bits
	uint8_t ctrlReg5 = i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG5);
	ctrlReg5 &= ~((uint8_t)(0x3u << I3G4250D_OUT_SEL_POS)); //Clear Out_Sel[1:0]
	ctrlReg5 |= (uint8_t)(LPF1_LPF2 << I3G4250D_OUT_SEL_POS);
	ctrlReg5 &= ~(uint8_t)I3G4250D_HIGHPASS_ENABLE;
	i3g4250d_writeReg(I3G4250D_CTRL_REG5, ctrlReg5);
	return i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, ctrlReg5);
}

/*
 * @brief	Burst-read six data bytes
 */
static bool i3g4250d_readRawData(){
	uint8_t buf[6];
	if(!i3g4250d_readRegBurst(I3G4250D_OUT_X_L, buf, 6)) return false;

	gyroRaw.rawX = (int16_t) ((buf[1] << 8) | buf[0]);
	gyroRaw.rawY = (int16_t) ((buf[3] << 8) | buf[2]);
	gyroRaw.rawZ = (int16_t) ((buf[5] << 8) | buf[4]);
	return true;
}

/*
 * @brief	Calibrate the sensor to find out the biasX, biasY, biasZ
 */
bool i3g4250d_calibrate(uint16_t sampleCount){
	if(sampleCount == 0) return false;

	//Discard a few samples after enabling sensor
	for(uint16_t i = 0; i < 40; i++){
		while(!i3g4250d_newDataReady()); //Wait for new data
		(void)i3g4250d_readRawData();
	}

	int sumX = 0, sumY = 0, sumZ = 0;
	for(uint32_t i = 0; i < sampleCount; i++){
		while(!i3g4250d_newDataReady()); //Wait for new data
		if(i3g4250d_readRawData()){
			sumX += gyroRaw.rawX;
			sumY += gyroRaw.rawY;
			sumZ += gyroRaw.rawZ;
		}
		HAL_Delay(4);
	}
	gyroRaw.biasX = (float) sumX / (float)sampleCount;
	gyroRaw.biasY = (float) sumY / (float)sampleCount;
	gyroRaw.biasZ = (float) sumZ / (float)sampleCount;
	return true; //Calibrate process is success
}

/*
 * @brief	Convert raw data to dps
 */
static bool i3g4250d_convertToDps(void){
	while(!i3g4250d_newDataReady());
	if(!i3g4250d_readRawData()) return false;
	const float k = gyroRaw.gyroSensitivity_dps_per_lsb;
	gyroRaw.dpsX = (float)(gyroRaw.rawX - gyroRaw.biasX) * k;
	gyroRaw.dpsY = (float)(gyroRaw.rawY - gyroRaw.biasY) * k;
	gyroRaw.dpsZ = (float)(gyroRaw.rawZ - gyroRaw.biasZ) * k;
	return true;
}

bool i3g4250d_getGps(float *xDps, float *yDps, float *zDps){
	if(!i3g4250d_convertToDps()) return false;
	*xDps = gyroRaw.dpsX;
	*yDps = gyroRaw.dpsY;
	*zDps = gyroRaw.dpsZ;
	return true;
}

/*
 * =====================================================================
 * Advanced algorithm to get more reliable angles
 * =====================================================================
 */

typedef struct{
	float w;
	float x;
	float y;
	float z;
} quat_t;

static quat_t s_q = {
		1.0f,
		0.0f,
		0.0f,
		0.0f
};

//Set this 3x3 to map sensor axes (X,Y,Z) to vehicle/body (roll, pitch, yaw)
//Entries are -1, 0, +1 only. Adjust to match ur board orientation.
static int8_t Rsv[3][3] = {
		{ +1, 0, 0}, //Body p (roll) comes from +sensor X
		{ 0, +1, 0}, //Body q (pitch) comes from +sensor Y
		{ 0 , 0, +1} //Body r (yaw) comes from +sensor Z
};

static inline void quat_normalize(quat_t *q){
	float n = sqrtf(((q -> w) * (q -> w)) + ((q -> x) * (q -> x)) + ((q -> y) * (q -> y)) + ((q -> z) * (q -> z)));
	float inv = 1.0f/n;
	q -> w *= inv;
	q -> x *= inv;
	q -> y *= inv;
	q -> z *= inv;
}

/*
 * Integrate attitude from body angular rate (rad/s)
 */
static inline void quat_update_from_gyro(quat_t *q, float p, float qv, float r, float dt){
	float qw = q -> w;
	float qx = q -> x;
	float qy = q -> y;
	float qz = q -> z;

	float half_dt = 0.5f * dt;

	q -> w = qw + (-qx*p - qy*qv - qz*r) * half_dt;
	q -> x = qx + (qw*p + qy*r - qz*qv) * half_dt;
	q -> y = qy + (qw*qv - qx*r + qz*p) * half_dt;
	q -> z = qz + (qw*r + qx*qv - qy*p) * half_dt;

	quat_normalize(q);
}

static inline void quat_to_euler(const quat_t *q, float *rollDeg, float *pitchDeg, float *yawDeg){
	//ZYX (yaw-pitch-roll) convention
	float qw = q -> w;
	float qx = q -> x;
	float qy = q -> y;
	float qz = q -> z;

	float sinr_cosp = 2.0f*(qw*qx + qy*qz);
	float cosr_cosp = 1.0f - 2.0f*(qx*qx + qy*qy);
	float roll = atan2f(sinr_cosp, cosr_cosp);

	float sinp = 2.0f*(qw*qy - qz*qx);
	float pitch = (fabsf(sinp) >= 1.0f) ? copysignf(3.14159/2.0f, sinp) : asinf(sinp);

	float siny_cosp = 2.0f*(qw*qz + qx*qy);
	float cosy_cosp = 1.0f - 2.0f*(qy*qy + qz*qz);
	float yaw = atan2f(siny_cosp, cosy_cosp);

	*rollDeg = roll * RAD2DEG;
	*pitchDeg = pitch * RAD2DEG;
	*yawDeg = yaw * RAD2DEG;
}

static inline void quat_to_euler_zyx_gl(const quat_t *q, float *rollDeg, float *pitchDeg, float *yawDeg){
	 float qw = q->w;
	 float qx = q->x;
	 float qy = q->y;
	 float qz = q->z;

	 // pitch
	float sinp = 2.0f*(qw*qy - qz*qx);
	// Clamp numeric noise
	if (sinp >  1.0f) sinp =  1.0f;
	if (sinp < -1.0f) sinp = -1.0f;
	float pitch = asinf(sinp);

	// Check for gimbal lock (|pitch| ≈ 90°)
	if (fabsf(sinp) > 0.9999f) {
		// Convention: roll = 0, yaw absorbs roll
		float yaw = atan2f(2.0f*(qw*qz + qx*qy), 1.0f - 2.0f*(qy*qy + qz*qz));
		*rollDeg  = 0.0f;
		*pitchDeg = pitch * RAD2DEG;
		*yawDeg   = yaw * RAD2DEG;
		return;
	}
	 // Regular case
	float sinr_cosp = 2.0f*(qw*qx + qy*qz);
	float cosr_cosp = 1.0f - 2.0f*(qx*qx + qy*qy);
	float roll = atan2f(sinr_cosp, cosr_cosp);

	float siny_cosp = 2.0f*(qw*qz + qx*qy);
	float cosy_cosp = 1.0f - 2.0f*(qy*qy + qz*qz);
	float yaw = atan2f(siny_cosp, cosy_cosp);

	*rollDeg  = roll  * RAD2DEG;
	*pitchDeg = pitch * RAD2DEG;
	*yawDeg   = yaw   * RAD2DEG;
}

static inline void quat_to_fused_angles(const quat_t *q,
                                        float *rollDeg, float *pitchDeg, float *yawDeg)
{
    float w=q->w, x=q->x, y=q->y, z=q->z;

    // Gravity in body frame (third column of rotation matrix)
    float gx = 2.0f*(x*z + y*w);
    float gy = 2.0f*(y*z - x*w);
    float gz = 1.0f - 2.0f*(x*x + y*y);

    // Clamp numeric noise
    if (gx >  1.0f) gx =  1.0f; if (gx < -1.0f) gx = -1.0f;
    if (gy >  1.0f) gy =  1.0f; if (gy < -1.0f) gy = -1.0f;

    float fusedPitch = asinf(-gx);                     // (-π/2 .. π/2)
    float fusedRoll  = asinf( gy);                     // (-π/2 .. π/2)
    float yaw        = atan2f(2.0f*(w*z + x*y), 1.0f - 2.0f*(y*y + z*z)); // standard yaw

    *rollDeg  = fusedRoll  * RAD2DEG;
    *pitchDeg = fusedPitch * RAD2DEG;
    *yawDeg   = yaw        * RAD2DEG;
}

bool i3g4250d_getAngle(float *roll, float *pitch, float *yaw){
	if(!i3g4250d_convertToDps()) return false; //Get the lastest rate in dps (bias removed)

	/*
	 * Map sensor frame -> vehicel/body frame (p, q, r) in dps
	 */
	float sx = gyroRaw.dpsX;
	float sy = gyroRaw.dpsY;
	float sz = gyroRaw.dpsZ;

	float p_dps = Rsv[0][0]*sx + Rsv[0][1]*sy + Rsv[0][2]*sz;
	float q_dps = Rsv[1][0]*sx + Rsv[1][1]*sy + Rsv[1][2]*sz;
	float r_dps = Rsv[2][0]*sx + Rsv[2][1]*sy + Rsv[2][2]*sz;

	/*
	 * Convert to rad/s
	 */
	float p = p_dps * DEG2RAD;
	float qv = q_dps * DEG2RAD;
	float r = r_dps * DEG2RAD;

	//Integrate quaternion with fixed dt
	quat_update_from_gyro(&s_q, p, qv, r, DT_SAMPLE);

	//Convert to Euler (deg)
	quat_to_fused_angles(&s_q, roll, pitch, yaw);

	return true;
}







//static uint8_t fifoWatermarkDefault = 10;
//
///*
// * @brief	Enable High-Pass Filter and FIFO
// */
//bool i3g4250d_enable_HPF_FIFO(FIFO_Mode_Config_t fifoMode, uint8_t watermark){
//	/*
//	 * CTRL_REG2
//	 * HPM -> Normal mode
//	 * High-pass cutoff freq = 0.5Hz
//	 */
//	uint8_t ctrlReg2 = (HPM_NORMAL_MODE << I3G4250D_HPM_POS) | (HPCF7 << I3G4250D_HPCF_POS);
//	i3g4250d_writeReg(I3G4250D_CTRL_REG2, ctrlReg2);
//	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG2, ctrlReg2)) return false;
//
//	/*
//	 * Configure CTRL_REG3
//	 * REG3: Data ready on DRDY/INT2 enable
//	 */
//	uint8_t ctrlReg3 = (i3g4250d_readRegUnsigned(I3G4250D_CTRL_REG3)) | I3G4250D_INT2_FIFO_WTM_ENABLE;
//	i3g4250d_writeReg(I3G4250D_CTRL_REG3, ctrlReg3);
//	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG3, ctrlReg3)) return false;
//
//	/*
//	 * Configure CTRL_REG5
//	 * HPF route + FIFO enable
//	 */
//	uint8_t ctrlReg5 = (LPF1_HPF << I3G4250D_OUT_SEL_POS) | (LPF1_HPF_INT1 << I3G4250D_INT1_SEL_POS) |	(I3G4250D_HIGHPASS_ENABLE) | I3G4250D_FIFO_ENABLE;
////	uint8_t ctrlReg5 = (LPF1 << I3G4250D_OUT_SEL_POS) | (LPF1_HPF_INT1 << I3G4250D_INT1_SEL_POS) |	(I3G4250D_HIGHPASS_ENABLE) | I3G4250D_FIFO_ENABLE;
//	i3g4250d_writeReg(I3G4250D_CTRL_REG5, ctrlReg5);
//	if(!i3g4250d_checkRegStatus(I3G4250D_CTRL_REG5, ctrlReg5)) return false;
//
//	//If HPF used, read REFERENCE once
//	(void)i3g4250d_readRegUnsigned(I3G4250D_REFERENCE);
//	HAL_Delay(3); // 1/ 800Hz = 0.00125s
//
//	/*
//	 * Configure FIFO_CTRL_REG
//	 * Set WTM = 10
//	 * Select FIFO mode
//	 */
//	watermark = (watermark == 0) ? 1 : (watermark & 0x1F);
//	fifoWatermarkDefault = watermark;
//
//	//Always reset FIFO first, then set the desired mode
//	i3g4250d_writeReg(I3G4250D_FIFO_CTRL_REG, (BYPASS_MODE << I3G4250D_FIFO_MODE_POS)); //Reset FIFO
//	uint8_t fifoCtrlReg = ((fifoMode & 0x7) << I3G4250D_FIFO_MODE_POS) | (fifoWatermarkDefault << I3G4250D_FIFO_THRS_POS);
//	i3g4250d_writeReg(I3G4250D_FIFO_CTRL_REG, fifoCtrlReg);
//
//	return true;
//}
//
///*
// * @brief	Check FIFO level
// */
//static inline uint8_t i3g4250d_fifoLevel(void){
//	return i3g4250d_readRegUnsigned(I3G4250D_FIFO_SRC_REG) & 0x1F;
//}
//
///*
// * @brief	Rearm the FIFO
// */
//static inline void i3g4250d_fifoRearm(void){
//	//Always reset FIFO first, then set the desired mode
//	i3g4250d_writeReg(I3G4250D_FIFO_CTRL_REG, (BYPASS_MODE << I3G4250D_FIFO_MODE_POS)); //Reset FIFO
//	uint8_t fifoCtrlReg = (FIFO_MODE << I3G4250D_FIFO_MODE_POS) | (fifoWatermarkDefault << I3G4250D_FIFO_THRS_POS);
//	i3g4250d_writeReg(I3G4250D_FIFO_CTRL_REG, fifoCtrlReg);
//}






