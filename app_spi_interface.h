/******************************************************************************
 * @file 	[app_spi_interface.c]
 *
 * Spi slave interface format convert function.
 * Convert different input format into 16bit duty martix buffer.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160530 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
#ifndef APP_SPI_INTERFACE_H_
#define APP_SPI_INTERFACE_H_

/***1 Includes ****************************************************************/
#include "std.h"

/***2.1 External Macros *******************************************************/

/***2.2 External Structures ***************************************************/
enum SpiRxFormatModel
{
	HISIV600_8BIT, MFC11_12BIT_80CH, CITRUS_12BIT_78CH
};

/***2.3 External Variables ****************************************************/

/***2.4 External Functions ****************************************************/
extern uint8 SpiSlave_handle(uint8 *spirx, uint16 *outputduty, enum SpiRxFormatModel model);

#endif /* APP_SPI_INTERFACE_H_ */
