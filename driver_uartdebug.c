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
/*1 Includes */

#include "driver_uartdebug.h"

#include <flashctl.h>
#include <msp430f5249.h>
#include <rtc_a.h>
#include <string.h>
#include <usci_a_uart.h>

#include "driver_iw7027.h"
#include "driver_mcu.h"
#include "std.h"

/*2.1 Internal Marcos */

/*2.2 Internal Struct */

/*2.3 Internal Variables */
static const uint8 HEX_ASCII_TABLE[16] =
{ '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', };
/*2.4 Internal Functions */

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
		PrintArray(HwBuf_SpiSlaveRx, 256);
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
			PrintInt(HwBuf_InputDuty[i]);
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
			PrintInt(HwBuf_OutputDuty[i]);
			if (i % 6 == 5)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if (!memcmp(uartrxbuf, "iw", 2))
	{
		SysParam_Iw7027.fIwRunErrorCheck = 1;
		Iw7027_updateWorkParams(&SysParam_Iw7027);
		PrintString("IW7027 Error Status");
		PrintArray((uint8 *) &SysParam_Iw7027, sizeof(SysParam_Iw7027));
		PrintEnter();
	}
	else if (!memcmp(uartrxbuf, "error", 5))
	{
		PrintString("\r\n Current Error Detect Param:\r\n");
		PrintArray((uint8 *) &SysParam_Error, sizeof(SysParam_Error));
		PrintString("\r\n Current Board Status:\r\n");
		PrintArray((uint8 *) &SysParam_BoardInfo, sizeof(SysParam_BoardInfo));
		PrintString("\r\n Flash Stored Error Param:\r\n");
		PrintArray(BOARD_ERROR_INFO_FLASH_PTR, sizeof(SysParam_Error));
		PrintString("\r\n Flash Stored Board Info:\r\n");
		PrintArray(BOARD_ERROR_INFO_FLASH_PTR + 0x20, sizeof(SysParam_BoardInfo));
	}
	else if (!memcmp(uartrxbuf, "erase", 5))
	{
		PrintString("\r\nErase Error Info.\r\n");
		FlashCtl_eraseSegment(BOARD_ERROR_INFO_FLASH_PTR);
	}
	else if (!memcmp(uartrxbuf, "mem ", 4) && uartrxbuf[8] == '=')
	{
		uint16 add;
		uint16 val;
		add = AsciiToHex(uartrxbuf[4]) * 0x1000 + AsciiToHex(uartrxbuf[5]) * 0x0100 + AsciiToHex(uartrxbuf[6]) * 0x0010
				+ AsciiToHex(uartrxbuf[7]);
		val = AsciiToHex(uartrxbuf[9]) * 0x10 + AsciiToHex(uartrxbuf[10]);
		HWREG8(add) = val;
		PrintString("\r\n Set Address [0x");
		PrintInt(add);
		PrintString("] = [0x");
		PrintChar(val);
		PrintString("]\r\n");
	}
	else if (!memcmp(uartrxbuf, "reboot", 6))
	{
		PrintString("\r\n MCU manual reboot. \r\n");
		Mcu_reset();
	}
	else
	{
		PrintString("\r\nHelp:\r\n"
				"spi   : Spi slave buffer\r\n"
				"i2c   : I2c slave buffer\r\n"
				"input : Duty input\r\n"
				"output: Duty output\r\n"
				"error : Error Status\r\n"
				"erase : Clear Error Info\r\n"
				"mem   : Memory direct modify\r\n"
				"reboot: Reboot Mcu\r\n");

	}
}

/* Brief	: Uart Send 1 byte
 * Example	:
 *  - Uart_Monitor_Send_Char(0x0D);  	--> 0x0D
 */
void UartSendChar(uint8 data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE, data);
}
/* Brief	: Uart Send 2 byte (1 int)
 * Example	:
 *  - PrintChar(0xABCD);				-->  0xAB 0xCD
 */
void UartSendInt(uint16 data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE,		//Send High byte
			(uint8) (data >> 4));
	USCI_A_UART_transmitData(USCI_A1_BASE,		//Send Low byte
			(uint8) data);
}

/* Brief	: Uart Send 1 char (1 byte)  in ASCII format (2 byte)
 * Example	:
 *  - PrintChar(0xAB);					-->  0x41 0x42 ("AB")
 */
void PrintChar(uint8 data)
{
	UartSendChar(HEX_ASCII_TABLE[data >> 4]);		//high 4 bits
	UartSendChar(HEX_ASCII_TABLE[data & 0x0F]);		//low 4 bits
}

void PrintCharBCD(uint8 data)
{
	uint8 a = data / 100;
	uint8 b = (data / 10) % 10;
	uint8 c = data % 10;

	UartSendChar(HEX_ASCII_TABLE[a]);
	UartSendChar(HEX_ASCII_TABLE[b]);
	UartSendChar(HEX_ASCII_TABLE[c]);
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
	UartSendChar('\r');
	UartSendChar('\n');

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
		UartSendChar(*string++);
	}
}

void PrintTime(Calendar *time)
{
	PrintString("[");
	PrintChar(time->Hours);
	PrintString(":");
	PrintChar(time->Minutes);
	PrintString(":");
	PrintChar(time->Seconds);
	PrintString("]");
}

uint8 AsciiToHex(uint8 hex)
{
	uint8 ascii = 0;
	if ((hex >= '0') && (hex <= '9'))
	{
		ascii = hex - '0';
	}
	if ((hex >= 'a') && (hex <= 'f'))
	{
		ascii = hex - 'a' + 0x0A;
	}
	if ((hex >= 'A') && (hex <= 'F'))
	{
		ascii = hex - 'A' + 0x0A;
	}
	return ascii;
}
