### STM32F411 Discovery + I3G4250D (ST 3-Axis Gyro) — Register-Level SPI Driver

#### 

#### Notes \& Assumptions

* The first version of this program is in "History".
* This program is now back to re-develop process to increase the reliability, stable of the results.
* Assume you understand other functions in other libraries such as GPIO, I2C, ADC, etc. So this README only mention about SPI and I3G4250D sensor.
* This driver currently leaves CPOL/CPHA at reset defaults. If your I3G4250D module requires SPI Mode 3, set those bits in SPI\_CR1 before enabling SPI.

PE3 is used as a manual CS; if you prefer hardware NSS, change the config and clear SSM/SSI accordingly.





#### Start:

A small, dependency-light driver for reading an I3G4250D gyroscope over SPI using an STM32F411 Discovery board. The SPI layer is written around bit-field helpers and simple polling-mode transfers (single byte and burst). The gyro layer brings the sensor up, verifies WHO\_AM\_I, and configures ODR/filters/FIFO options.



* Tested target: STM32F411 Discovery (STM32F411VET6)
* Sensor: I3G4250D (STMicroelectronics, 2000 dps max)



#### Highlights

* Minimal HAL usage (RCC enable + HAL\_Delay); data path is register-level
* Clean bit-mask helpers for any peripheral register
* Blocking SPI read/write (single and burst) with BSY/TXE/RXNE checks
* Verified WHO\_AM\_I = 0xD3 and soft-reboot sequence
* Configurable: ODR/BW, 4-wire SPI, endianness, optional HPF and FIFO bits
* Works out-of-the-box on SPI1 of F411-Discovery, pins pre-mapped



#### SPI Configuration (what the driver sets)

* Instance: SPI1 (configurable in i3g4250d\_Config)
* Mode: Master
* Frame size: 8-bit (DFF=0)
* Baud: FPCLK\_DIV16 (≈ 6.25 MHz on 100 MHz APB2; adjust as needed)
* NSS: Software-managed (SSM=1, SSI=1), manual CS on PE3
* Duplex: Full-duplex
* Ordering: MSB first (default)
* Clock polarity/phase: Left at reset defaults in this code (Mode 0). Some ST MEMS gyros prefer Mode 3 (CPOL=1, CPHA=1). If you see only 0xFF/0x00, set CPOL/CPHA bits in SPI\_CR1 accordingly.



#### Gyro Setup (what i3g4250d\_init() does)

* Enables GPIO and SPI clocks and alternate functions
* Checks WHO\_AM\_I == 0xD3
* Reboots memory (CTRL\_REG5.REBOOT)
* CTRL\_REG1: enables X/Y/Z + Power-up, sets ODR=800 Hz / BW=30 Hz
* CTRL\_REG4: 4-wire SPI, ±2000 dps full scale, BLE=LSB@low
* Optional HPF (CTRL\_REG2) and FIFO/routing bits (CTRL\_REG5, CTRL\_REG3)
* Reads REFERENCE and waits a short settling delay
* Sensitivity for ±2000 dps is 70 mdps/LSB (0.07 dps/LSB).



#### API Overview (public pieces used in this project)

##### SPI (from spi.c/spi.h)

* void SPI\_GPIO\_init(const SPI\_GPIO\_Config\_t \*config);
* void SPI\_basicConfigInit(const SPI\_GPIO\_Config\_t \*config, SPI\_MSTR\_t, SPI\_DFF\_t, SPI\_BaudRate\_t, SPI\_SSM\_t);



#### Transfers

* uint8\_t SPI\_readRegUnsigned(const SPI\_GPIO\_Config\_t\*, uint8\_t regAddr);
* int8\_t SPI\_readRegSigned (const SPI\_GPIO\_Config\_t\*, uint8\_t regAddr);
* bool SPI\_readRegBurst (const SPI\_GPIO\_Config\_t\*, uint8\_t startRegAddr, uint8\_t \*rxBuf, uint8\_t len);
* void SPI\_writeReg (const SPI\_GPIO\_Config\_t\*, uint8\_t regAddr, uint8\_t value);



Internally we use TXE/BSY/RXNE waits to guarantee the byte shifted out and the return byte is ready before reading SPI\_DR.





#### I3G4250D (from i3g4250d.c/i3g4250d.h)

* void i3g4250dGPIO\_init(void);
* bool i3g4250d\_init(initConfig\_t filterEn, initConfig\_t fifoEn);



The file also contains static helpers that wrap the SPI layer for this specific sensor. If you want to call “read XYZ” from outside, either expose a function in the header (see below) or call the generic SPI helpers with the sensor’s register map.

