## LedDriver
MSP430 based Local Dimming LED Driver for LCD TV. 
### File Struct:
std.h			: Standard type define & basic macros.
system_pre_init.c	: Boot program.
main.c			: Main program.
hal.c / hal.h		: Hardware abstract layer , interface to MCU hardware.
drv_*.c			: Driver for hardware devices.
api_*.c			: Application program interface.

