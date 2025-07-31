Program Objectives
  Implement a UART command interface that lets a terminal toggle the onboard LEDs.
  Enable firmware updates for the STM32F411 over UART.
  Support bootloader-assisted firmware uploads to a specified Flash sector via UART.
  Read, process, and transmit temperature and gyroscope data over UART for logging.

Constraint
  All of the above tasks must run concurrently under FreeRTOS.
