STM32F411 I3G4250D Gyroscope Driver:

Author: Do Bao Long



A lightweight, register-level drivers that lets any STM32F411 board talk to ST-Micro's I3G4250D 3-axis digital gyroscope.



Ideal for quick classroom demos, hobby projects or as a starting point for your own IMU fusion.



Features:

&nbsp;	\* Init/Reset - Verifies WHO\_AM\_I, set ODR = 800Hz/35Hz BW, +- 2000dps, DRDY routed to INT2.

&nbsp;	\* Burst read (6-byte auto-increment) for atomic X/Y/Z snapshots.

&nbsp;	\* Bias calibration - Averages N stationary samples to remove zero-rate offset.

&nbsp;	\* Counts: deg/sec conversion with on-the-fly sensitivity selection (8.75/17.5/70mdps/LSB)

&nbsp;	\* Simple integration helper to accumulate angle (deg)

