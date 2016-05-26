#include "app_uartdebug.h"
#include "string.h"


void Uart_Console(uint8_t *uartrxbuf)
{
	if( ! memcmp(uartrxbuf , "test" , 4) )
	{
		PrintString("\r\nTEST\r\n");

	}
	else if( ! memcmp(uartrxbuf , "spi" , 3) )
	{
		PrintString("\r\nSpi Slave buffer:\r\n");
		PrintArray(SpiSlave_RxBuff,256);
	}
	else if( ! memcmp(uartrxbuf , "i2c" , 3) )
	{
		PrintString("\r\nI2C Slave buffer:\r\n");
		PrintArray((uint8_t*)(0x4000) , 256 );
	}
	else if( ! memcmp(uartrxbuf , "input" , 5) )
	{
		uint8_t i;
		PrintString("\r\n Input Duty Buffer: \r\n");
		for(i = 0 ; i < 80 ; i ++ )
		{
			PrintString(" ");
			PrintInt(System_InputDutyBuff[i]);
			if( i % 6 == 5)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if( ! memcmp(uartrxbuf , "output" , 6) )
	{
		uint8_t i;
		PrintString("\r\n Out Duty Buffer: \r\n");
		for(i = 0 ; i < 80 ; i ++ )
		{
			PrintString(" ");
			PrintInt(System_OutputDutyBuff[i]);
			if( i % 6 == 5)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if( ! memcmp(uartrxbuf , "gamma" , 5) )
	{

		uint16_t i;
		PrintString("\r\nDPL Input Gamma\r\n");
		for(i = 0 ; i < 256 ; i ++ )
		{
			PrintString(" ");
			PrintInt(DPL_InputGamma[i]);
			if( i % 16 == 15)
			{
				PrintString("\r\n");
			}
		}
		PrintString("\r\n");
	}
	else if( ! memcmp(uartrxbuf , "iw" , 2) )
	{
		System_Iw7027Param.iwRunErrorCheck = 1 ;
		Iw7027_updateWorkParams(&System_Iw7027Param);
		PrintString("IW7027 Error Status");
		PrintArray((uint8_t *)&System_Iw7027Param,sizeof(System_Iw7027Param));
		PrintEnter();
	}
	else if( ! memcmp(uartrxbuf , "error" , 5) )
	{
		PrintString("\r\n Current Error Detect Param:\r\n");
		PrintArray((uint8_t *)&System_ErrorParam,sizeof(System_ErrorParam));
		PrintString("\r\n Current Board Status:\r\n");
		PrintArray((uint8_t *)&System_BoardInfo,sizeof(System_BoardInfo));
		PrintString("\r\n Flash Stored Error Param:\r\n");
		PrintArray(BOARD_ERROR_INFO_FLASH_PTR,sizeof(System_ErrorParam));
		PrintString("\r\n Flash Stored Board Info:\r\n");
		PrintArray(BOARD_ERROR_INFO_FLASH_PTR + 0x20,sizeof(System_BoardInfo));
	}
	else if( ! memcmp(uartrxbuf , "erase" , 5) )
	{
		PrintString("\r\nErase Error Info.\r\n");
		FlashCtl_eraseSegment(BOARD_ERROR_INFO_FLASH_PTR);
	}
	else if( ! memcmp(uartrxbuf , "reboot" , 6) )
	{
		PrintString("\r\n MCU manual reboot. \r\n");
		Mcu_reset();
	}
	else
	{
		PrintString(
				"\r\nHelp:\r\n"
				"spi   : Spi slave buffer\r\n"
				"i2c   : I2c slave buffer\r\n"
				"input : Duty input\r\n"
				"output: Duty output\r\n"
				"error : Error Status\r\n"
				"erase : Clear Error Info\r\n"
				"reboot: Reboot Mcu\r\n");

	}
}


/* Brief	: Uart Send 1 byte
 * Example	:
 *  - Uart_Monitor_Send_Char(0x0D);  	--> 0x0D
 */
void UartSendChar(unsigned char data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE,
							data
							);
}
/* Brief	: Uart Send 2 byte (1 int)
 * Example	:
 *  - PrintChar(0xABCD);				-->  0xAB 0xCD
 */
void UartSendInt(unsigned int data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE,		//Send High byte
			(unsigned char)(data>>4)
							);
	USCI_A_UART_transmitData(USCI_A1_BASE,		//Send Low byte
			(unsigned char)data
							);
}

/* Brief	: Uart Send 1 char (1 byte)  in ASCII format (2 byte)
 * Example	:
 *  - PrintChar(0xAB);					-->  0x41 0x42 ("AB")
 */
void PrintChar(unsigned char data)
{
	UartSendChar(HEX_ASCII_TABLE[data>>4]);		//high 4 bits
	UartSendChar(HEX_ASCII_TABLE[data&0x0F]);		//low 4 bits
}

void PrintCharBCD(unsigned char data)
{
	 uint8_t a = data/100;
	 uint8_t b = (data/10) % 10;
	 uint8_t c = data%10;

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
void PrintInt(unsigned int data)
{
	PrintChar((unsigned char)(data>>8));					//send high byte
	PrintChar((unsigned char)data);						//send low byte
}

/* Brief	: Uart send an array with selected length
 * Example	:
 *  - unsigned char TEST[5] = {0x12,0x34,0x56,0x78,0x9A};
 *  - PrintArray((unsigned char*)TEST , 5);		-->  0x30 0x31 0x32 0x33 0x34 0x35 0x36 0x37 0x38 0x39 0x40 ("123456789A")
 */
void PrintArray(unsigned char *array , unsigned int length)
{
	unsigned int i;
	for(i = 0 ; i < length ; i++)
	{
		PrintChar(*array++);
		if( i % 16 == 15)
		{
			PrintEnter();
		}
	}
}

/* Brief	: Uart send a string
 * Example	:
 *  - PrintString("Good");		-->  0x47 0x6F 0x6F 0x64 ("Good")
 */
void PrintString(unsigned char *string)
{
	while(*string)
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


