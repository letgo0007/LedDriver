#ifndef APP_SPI_INTERFACE_H_
#define APP_SPI_INTERFACE_H_

#include "std.h"

//Supported format model
enum SpiRxFormatModel
{
	HISIV600_8BIT,
	MFC11_12BIT_80CH,
	CITRUS_12BIT_78CH
};

//Function Calls
uint8 SpiSlave_handle(uint8 *spirx, uint16 *outputduty, enum SpiRxFormatModel model);


#endif /* APP_SPI_INTERFACE_H_ */
