#include "uartdebug.h"
#include "board.h"

//Need access to Uart rxed buffer
extern unsigned char Uart_Monitor_Rx_Buffer[256];

void Uart_Console(void)
{


    //Compare buffer for certain command
	if(	Uart_Buffer_Compare("test"))
	{
		PrintStringL("test ok");
		PrintArray(SpiSlave_RxBuff,256);
	}
	else if(Uart_Buffer_Compare("i2cw "))
	{
		Printi2cw();
	}
	else if(Uart_Buffer_Compare("time"))
	{
		PrintTime(&System_Time);
	}
	else if(Uart_Buffer_Compare("reboot"))
	{
		WDT_A_initWatchdogTimer(	WDT_A_BASE,		//Set watch dog timer to 64 cpu cycles
		                            WDT_A_CLOCKSOURCE_SMCLK,
									WDT_A_CLOCKDIVIDER_64);
		WDT_A_start(WDT_A_BASE);
		__delay_cycles(100);						//Wait watch dog reboot system
	}
	else
	{
		PrintHelp();
	}


}

/* Compare Uart buffer with certain string
 * If all same , return 1
 * any byte different, return 0
 */
unsigned char Uart_Buffer_Compare(char *string)
{


	unsigned char i=0;

	while(*string)
	{
		if(Uart_RxBuff[i++]==*string++){
			;
		}
		else{
			return 0;
		}

	}

	return 1;
}


void PrintHelp(void){
	PrintString(
			"CONSOLE HELP:\r\n"
			"test1\r\n"
			"test2\r\n"
			);
}

void Printi2cw(void)
{
	//				   123456789ABCDEF
	//command format : i2cw A0 55 42 43
	PrintString("I2C Write Dev:");

}

/* Brief	: Uart Send 1 byte
 * Example	:
 *  - Uart_Monitor_Send_Char(0x0D);  	--> 0x0D
 */
void Uart_Monitor_Send_Char(unsigned char data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE,
							data
							);
}
/* Brief	: Uart Send 2 byte (1 int)
 * Example	:
 *  - PrintChar(0xABCD);				-->  0xAB 0xCD
 */
void Uart_Monitor_Send_Int(unsigned int data)
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
	Uart_Monitor_Send_Char(HEX_ASCII_TABLE[data>>4]);		//high 4 bits
	Uart_Monitor_Send_Char(HEX_ASCII_TABLE[data&0x0F]);		//low 4 bits
}

void PrintCharBCD(unsigned char data)
{
	 uint8_t a = data/100;
	 uint8_t b = (data/10) % 10;
	 uint8_t c = data%10;

	Uart_Monitor_Send_Char(HEX_ASCII_TABLE[a]);
	Uart_Monitor_Send_Char(HEX_ASCII_TABLE[b]);
	Uart_Monitor_Send_Char(HEX_ASCII_TABLE[c]);
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
	Uart_Monitor_Send_Char('\r');
	Uart_Monitor_Send_Char('\n');

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
	while(length--)
	{
		PrintChar(*array++);
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
		Uart_Monitor_Send_Char(*string++);
	}
}

/* Brief	: Uart send a string end with en enter
 * Example	:
 *  - PrintString("Good");		-->  0x47 0x6F 0x6F 0x64 ("Good")
 */
void PrintStringL(unsigned char *string)
{
	while(*string)
	{
		Uart_Monitor_Send_Char(*string++);
	}
	PrintEnter();
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


