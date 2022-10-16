# FM25W256 Driver for STM32Cube Library

The FM25W256 is a 256-Kbit nonvolatile memory employing an advanced ferroelectric process. A ferroelectric random access memory or F-RAM is nonvolatile and performs reads and writes similar to a RAM. It provides reliable data retention for 151 years while eliminating the complexities, overhead, and system level reliability problems caused by serial flash, EEPROM, and other nonvolatile memories.

# Functionality 

Simple driver for the FM25W256 to be used with the STM32 HAL from STElectronics that allows:

* Checking the status of the FRAM
* Reading from the FRAM 
* Writing to the FRAM

# Integration TODO
 
Driver has the interrupts TBD that need to be configure
The function TBD shall be called every TBD to keep the logic working

# Limtations
* Tested only on STM32F7
* UT Only tested on windows, should work on linux 
* ```stm32f7xx_hal.h``` contains a cropped version of the HAL to allow simple and portable UT.
* Not exaustive testing