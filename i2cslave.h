#ifndef I2CSLAVE_H_
#define I2CSLAVE_H_

#include "driverlib.h"
#include "board.h"
#include "iw7027.h"
#include "scheduler.h"

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

#endif /* I2CSLAVE_H_ */
