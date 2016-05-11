#ifndef UARTDEBUG_H_
#define UARTDEBUG_H_

#include "driverlib.h"
#include "scheduler.h"
#define UART_CONSOLE_VER	(1)
//Main program for uart cosole
void Uart_Console(void);

//Compare UART RX buffer with certain string
unsigned char Uart_Buffer_Compare(char *string);
void PrintHelp(void);
void Printi2cw(void);

// HEX format uart out
void Uart_Monitor_Send_Char(unsigned char data);
void Uart_Monitor_Send_Int(unsigned int data);

// ASCII format uart out
void PrintChar(unsigned char data);
void PrintCharBCD(unsigned char data);
void PrintEnter(void);
void PrintInt(unsigned int data);
void PrintArray(unsigned char *array , unsigned int length);
void PrintString(unsigned char *string);
void PrintStringL(unsigned char *string);
void PrintTime(Calendar *time);

static const unsigned char HEX_ASCII_TABLE[16]={
		0x30	,	//0
		0x31	,	//1
		0x32	,	//2
		0x33	,	//3
		0x34	,	//4
		0x35	,	//5
		0x36	,	//6
		0x37	,	//7
		0x38	,	//8
		0x39	,	//9
		0x41	,	//A
		0x42	,	//B
		0x43	,	//C
		0x44	,	//D
		0x45	,	//E
		0x46		//F
};
#endif /* UARTDEBUG_H_ */
