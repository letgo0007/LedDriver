/******************************************************************************
 * @file 	[driver_uartdebug.c]
 *
 * Debug uart port print functions.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
/***1 Includes ****************************************************************/

#include <string.h>

#include "drv_uart.h"

#include "drv_iw7027.h"
#include "hal.h"
#include "std.h"

/*2.1 Internal Marcos */

/*2.2 Internal Struct */

/*2.3 Internal Variables */

static const uint8 ASCII_TABLE[16] =
{ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', };

/*2.4 Internal Functions */

/* Brief	: Uart Send 2 byte (1 int)
 * Example	:
 *  - PrintChar(0xABCD);				-->  0xAB 0xCD
 */
void UartSendInt(uint16 data)
{
	Hal_Uart_sendSingleByte((uint8) (data >> 8));
	Hal_Uart_sendSingleByte((uint8) data);
}

uint8 AsciiToHex(uint8 ascii)
{
	uint8 hex = 0;
	if ((ascii >= '0') && (ascii <= '9'))
	{
		hex = ascii - '0';
	}
	else if ((ascii >= 'a') && (ascii <= 'f'))
	{
		hex = ascii - 'a' + 0x0A;
	}
	else if ((ascii >= 'A') && (ascii <= 'F'))
	{
		hex = ascii - 'A' + 0x0A;
	}
	return hex;
}

/*3	External Functions */
void Uart_Console(uint8 *uartrxbuf)
{
	if (!memcmp(uartrxbuf, "test", 4))
	{
		PrintString("\r\nTEST\r\n");

	}
	else if (!memcmp(uartrxbuf, "spi", 3))
	{
		PrintString("\r\nSpi Slave buffer:\r\n");
		PrintArray(u8Hal_Buf_SpiSlaveRx, 256);
	}
	else if (!memcmp(uartrxbuf, "i2c", 3))
	{
		PrintString("\r\nI2C Slave buffer:\r\n");
		PrintArray((uint8*) (0x4000), 256);
	}
	else if (!memcmp(uartrxbuf, "input", 5))
	{
		uint8 i;
		PrintString("\r\n Input Duty Buffer: \r\n");
		for (i = 0; i < 80; i++)
		{
			PrintString(" ");
			PrintInt(u16Hal_Buf_InputDuty[i]);
			if (i % 6 == 5)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if (!memcmp(uartrxbuf, "output", 6))
	{
		uint8 i;
		PrintString("\r\n Out Duty Buffer: \r\n");
		for (i = 0; i < 80; i++)
		{
			PrintString(" ");
			PrintInt(u16Hal_Buf_OutputDuty[i]);
			if (i % 6 == 5)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if (!memcmp(uartrxbuf, "iw", 2))
	{
		tDrv_Iw7027Param.fIwRunErrorCheck = 1;
		Iw7027_updateWorkParams(&tDrv_Iw7027Param);
		PrintString("IW7027 Error Status");
		PrintArray((uint8 *) &tDrv_Iw7027Param, sizeof(tDrv_Iw7027Param));
		PrintEnter();
	}
	else if (!memcmp(uartrxbuf, "error", 5))
	{
		PrintString("\r\n Current Error Detect Param:\r\n");
		PrintArray((uint8 *) &tHal_BoardErrorParam, sizeof(tHal_BoardErrorParam));
		PrintString("\r\n Current Board Status:\r\n");
		PrintArray((uint8 *) &tHal_BoardInfo, sizeof(tHal_BoardInfo));
		PrintString("\r\n Flash Stored Error Param:\r\n");
		PrintArray(HAL_ERROR_INFO_FLASH_PTR, sizeof(tHal_BoardErrorParam));
		PrintString("\r\n Flash Stored Board Info:\r\n");
		PrintArray(HAL_ERROR_INFO_FLASH_PTR + 0x20, sizeof(tHal_BoardInfo));
	}
	else if (!memcmp(uartrxbuf, "erase", 5))
	{
		PrintString("\r\nErase Error Info.\r\n");
		Hal_Flash_eraseSegment(HAL_ERROR_INFO_FLASH_PTR);
	}
	else if (!memcmp(uartrxbuf, "mem ", 4) && uartrxbuf[8] == '=')
	{
		uint16 add;
		uint16 val;
		add = AsciiToHex(uartrxbuf[4]) * 0x1000 + AsciiToHex(uartrxbuf[5]) * 0x0100 + AsciiToHex(uartrxbuf[6]) * 0x0010
				+ AsciiToHex(uartrxbuf[7]);
		val = AsciiToHex(uartrxbuf[9]) * 0x10 + AsciiToHex(uartrxbuf[10]);
		HAL_REG8(add) = val;
		PrintString("\r\n Set Address [0x");
		PrintInt(add);
		PrintString("] = [0x");
		PrintChar(val);
		PrintString("]\r\n");
	}
	else if (!memcmp(uartrxbuf, "reboot", 6))
	{
		PrintString("\r\n MCU manual reboot. \r\n");
		Hal_Mcu_reset();
	}
	else
	{
		PrintString("\r\nHelp:\r\n"
				"spi   : Spi slave buffer\r\n"
				"i2c   : I2c slave buffer\r\n"
				"input : Duty input\r\n"
				"output: Duty output\r\n"
				"iw    : IW7027 Error Check\r\n"
				"error : Error Status\r\n"
				"erase : Clear Error Info\r\n"
				"mem   : Memory direct modify\r\n"
				"reboot: Reboot Mcu\r\n");

	}
}



/* Brief	: Uart Send 1 char (1 byte)  in ASCII format (2 byte)
 * Example	:
 *  - PrintChar(0xAB);					-->  0x41 0x42 ("AB")
 */
void PrintChar(uint8 data)
{
	Hal_Uart_sendSingleByte(ASCII_TABLE[data >> 4]);		//high 4 bits
	Hal_Uart_sendSingleByte(ASCII_TABLE[data & 0x0F]);		//low 4 bits
}

void PrintCharBCD(uint8 data)
{
	uint8 a = data / 100;
	uint8 b = (data / 10) % 10;
	uint8 c = data % 10;

	Hal_Uart_sendSingleByte(ASCII_TABLE[a]);
	Hal_Uart_sendSingleByte(ASCII_TABLE[b]);
	Hal_Uart_sendSingleByte(ASCII_TABLE[c]);
}

/* Brief	: "Enter" in windows format
 * Example	:
 *  - PrintEnter();						-->  0x0D,0x0A
 */
void PrintEnter(void)
{
// windows 	: CR + LF
// Mac		: CR
// Linux	: LF
	Hal_Uart_sendSingleByte('\r');
	Hal_Uart_sendSingleByte('\n');

}

/* Brief	: Uart send 1 int (2 byte)  in ASCII format (4 byte)
 * Example	:
 *  - PrintChar(0xABCD);					-->  0x41,0x42,0x43,0x44 ("ABCD")
 */
void PrintInt(uint16 data)
{
	PrintChar((uint8) (data >> 8));					//send high byte
	PrintChar((uint8) data);						//send low byte
}

/* Brief	: Uart send an array with selected length
 * Example	:
 *  - uint8 TEST[5] = {0x12,0x34,0x56,0x78,0x9A};
 *  - PrintArray((uint8*)TEST , 5);		-->  0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37 0x38 0x39 0x40 ("123456789A")
 */
void PrintArray(uint8 *array, uint16 length)
{
	uint16 i;
	for (i = 0; i < length; i++)
	{
		PrintChar(*array++);
		if (i % 16 == 15)
		{
			PrintEnter();
		}
	}
}

/* Brief	: Uart send a string
 * Example	:
 *  - PrintString("Good");		-->  0x47 0x6F 0x6F 0x64 ("Good")
 */
void PrintString(uint8 *string)
{
	while (*string)
	{
		Hal_Uart_sendSingleByte(*string++);
	}
}

void PrintTime(Hal_Time *time)
{
	PrintString("[");
	PrintChar(time->Hours);
	PrintString(":");
	PrintChar(time->Minutes);
	PrintString(":");
	PrintChar(time->Seconds);
	PrintString("]");
}


