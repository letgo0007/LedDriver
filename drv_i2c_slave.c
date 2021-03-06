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

/***1 Includes ***************************************************************/

#include "drv_i2c_slave.h"

#include "drv_iw7027.h"
#include "drv_uart.h"
#include "hal.h"

/***2.1 Internal Marcos ******************************************************/

/***2.2 Internal Struct ******************************************************/

/***2.3 Internal Variables ***************************************************/

static const uint16 TestPattern_Logo[80] =
{
/*ROW0*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW1*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW2*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW3*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW4*/0x0000, 0x0000, 0x0600, 0x0600, 0x0000, 0x0000,
/*ROW5*/0x0000, 0x0600, 0x0F00, 0x0F00, 0x0600, 0x0000,
/*ROW6*/0x0000, 0x0600, 0x0F00, 0x0F00, 0x0600, 0x0000,
/*ROW7*/0x0000, 0x0600, 0x0F00, 0x0F00, 0x0600, 0x0000,
/*ROW8*/0x0000, 0x0000, 0x0600, 0x0600, 0x0000, 0x0000,
/*ROW9*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROWA*/0x0000, 0x0000, 0x0800, 0x0800, 0x0000, 0x0000,
/*ROWB*/0x0000, 0x0000, 0x0800, 0x0800, 0x0000, 0x0000,
/*ROWC*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*NC  */0x0000, 0x0000, };

/***2.4 External Variables ***************************************************/

/***2.5 Internal Functions ***************************************************/

/***2.6 External Functions ***************************************************/
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
	sfbuff[I2C_SPIACCESS_WRMODE] = 0x00;

	/*Special Function 2 [Manual pattern]
	 * Examples:
	 * Set area 5~8 duty 0x0ABC
	 * 					[WRMODE]	[DUTY_L] 	[DUTY_H] 	[START_CH] 	[END_CH]
	 * I2C Write:		0x80		0xBC		0x0A		0x05		0x08
	 * 					0x81		//Mute
	 * 					0x82		//Logo
	 */
	switch (sfbuff[I2C_MANUAL_WRMODE])
	{
	case 0x80: // Write Manual Pattern.
	{
		uint16 duty = 0x0100 * sfbuff[I2C_MANUAL_DUTY_H] + sfbuff[I2C_MANUAL_DUTY_L];
		PrintString("Test Pattern");
		PrintArray(&sfbuff[I2C_MANUAL_WRMODE], 5);
		Hal_Mem_set16((uint32) &u16Hal_Buf_TestDuty[sfbuff[I2C_MANUAL_START_CH]], duty,
				sfbuff[I2C_MANUAL_END_CH] - sfbuff[I2C_MANUAL_START_CH]);
		break;
	}
	case 0x81: //Mute Pattern
		PrintString("Test Pattern = Mute.\r\n");
		Hal_Mem_set16((uint32) &u16Hal_Buf_TestDuty[0], 0x0000, sizeof(u16Hal_Buf_TestDuty) / 2);
		break;
	case 0x82: //Logo Pattern
		PrintString("Test Pattern = Logo.\r\n");
		Hal_Mem_copy((uint32) &u16Hal_Buf_TestDuty[0], (uint32) &TestPattern_Logo[0], sizeof(u16Hal_Buf_TestDuty));
		break;
	default: //No operation
		break;
	}
	sfbuff[I2C_MANUAL_WRMODE] = 0x00;

	/*Special Function 3 [Direct Memory Access]
	 * Examples:
	 * Write 0x05 to RAM address 0x2400
	 * 					[I2C_DMA_WRMODE]	[I2C_DMA_ADD_H] 	[I2C_DMA_ADD_L] 	[I2C_DMA_TXDATA] 	[I2C_DMA_RXDATA]
	 * I2C Write:		0x80				0x24				0x00				0x05
	 * Read Byte from RAM address 0x2400
	 * 					[I2C_DMA_WRMODE]	[I2C_DMA_ADD_H] 	[I2C_DMA_ADD_L] 	[I2C_DMA_TXDATA] 	[I2C_DMA_RXDATA]
	 * I2C Write:		0x80				0x24				0x00
	 * I2C Read:		0x81				0x24				0x00									XX
	 */
	switch (sfbuff[I2C_DMA_WRMODE])
	{
	case 0x80: //Read
	{
		uint16 add = 0x0100 * sfbuff[I2C_DMA_ADD_H] + sfbuff[I2C_DMA_ADD_L];
		sfbuff[I2C_DMA_RXDATA] = HAL_REG8(add);
		break;
	}
	case 0x81: //Write
	{
		uint16 add = 0x0100 * sfbuff[I2C_DMA_ADD_H] + sfbuff[I2C_DMA_ADD_L];
		HAL_REG8(add) = sfbuff[I2C_DMA_TXDATA];
		break;
	}
	default: //No operation
		break;
	}
	sfbuff[I2C_DMA_WRMODE] = 0x00;

	return FLAG_SUCCESS;

}

