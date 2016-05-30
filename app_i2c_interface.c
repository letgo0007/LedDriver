/******************************************************************************
 * @file 	[app_i2c_interface.c]
 *
 * I2C slave interface Special Function Hander.
 * The I2C slave have 2 ways to do system control.
 * 1 : DMA Mode , i2c slave data direct access to RAM , modify params .
 * 2 : Special Function Mode , for some function, not all params is memory
 * acessable or memory too large, function is called in certain order.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/
#include "app_i2c_interface.h"

#include "driver_iw7027.h"
#include "driver_mcu.h"

uint8 I2cSlave_handleSpecialFunction(uint8 *sfbuff)
{
	/*Special Function 1 [SPI direct access function]
	 * Examples:
	 * Single Write: write 0x11 to reg 0xAA of IW7027_0.
	 * 					[WRMODE]	[CHIP_SEL] 	[REGADD] 	[TXDATA] 	[RXDATA]
	 * I2C Write:		0x80		0x01		0xAA		0x11
	 *
	 * Single Read: read byte from 0xAA of IW7027_0.
	 * 					[WRMODE]	[CHIP_SEL] 	[REGADD] 	[TXDATA] 	[RXDATA]
	 * I2C Write:		0x81		0x01		0xAA
	 * I2C Read:														XX
	 */
	switch (sfbuff[I2C_SPIACCESS_WRMODE])
	{
	case 0x80: //Single Read
		sfbuff[ I2C_SPIACCESS_RXDATA] = Iw7027_readSingleByte(sfbuff[I2C_SPIACCESS_CHIPSEL], sfbuff[I2C_SPIACCESS_REGADD]);
		break;
	case 0x81: //Single Write
		Iw7027_writeSingleByte(sfbuff[I2C_SPIACCESS_CHIPSEL], sfbuff[I2C_SPIACCESS_REGADD], sfbuff[I2C_SPIACCESS_TXDATA]);
		break;
	default: //No operation
		break;
	}
	sfbuff[I2C_SPIACCESS_WRMODE] = 0;

	/*Special Function 2 [Manual pattern]
	 * Examples:
	 * Set area 5~8 duty 0x0ABC
	 * 					[WRMODE]	[DUTY_H] 	[DUTY_L] 	[START_CH] 	[END_CH]
	 * I2C Write:		0x80		0x0A		0xBC		0x05		0x08
	 */
	switch (sfbuff[I2C_MANUAL_WRMODE])
	{
	case 0x80:// Write Manual Pattern.
	{
		uint16 i = 0;
		uint16 duty = 0x0100 * sfbuff[I2C_MANUAL_DUTY_H] + sfbuff[I2C_MANUAL_DUTY_L];

		for (i = sfbuff[ I2C_MANUAL_START_CH]; i < sfbuff[ I2C_MANUAL_END_CH]; i++)
		{
			HwBuf_TestDuty[i] = duty;
		}

		break;
	}
	default: //No operation
		break;
	}
	sfbuff[I2C_MANUAL_WRMODE] = 0x00;

	return FLAG_SUCCESS;

}

