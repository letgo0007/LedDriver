#ifndef APP_I2C_INTERFACE_H_
#define APP_I2C_INTERFACE_H_

#include "driverlib.h"
#include "driver_iw7027.h"
#include "driver_mcu.h"
#include "driver_scheduler.h"

//Struct
enum SpiRxFormatModel
{
	HISIV600_8BIT,
	MFC11_12BIT_80CH,
	CITRUS_12BIT_78CH
};

//Function Calls
uint8_t SpiSlave_Handler(uint8_t *spirx, uint16_t *outputduty, enum SpiRxFormatModel model);
uint8_t I2cSlave_handleMap(uint8_t *i2cmap);
uint8_t I2cSlave_initMap(uint8_t *i2cmap);

#endif /* APP_I2C_INTERFACE_H_ */
