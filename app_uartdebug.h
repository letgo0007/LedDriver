#ifndef APP_UARTDEBUG_H_
#define APP_UARTDEBUG_H_

#include "driver_scheduler.h"
#include "driverlib.h"

//Main program for uart cosole
void Uart_Console(uint8_t *uartrxbuf);

//Compare UART RX buffer with certain string
unsigned char Uart_Buffer_Compare(char *string);


// HEX format uart out
void UartSendChar(unsigned char data);
void UartSendInt(unsigned int data);

// ASCII format uart out
void PrintChar(unsigned char data);
void PrintCharBCD(unsigned char data);
void PrintEnter(void);
void PrintInt(unsigned int data);
void PrintArray(unsigned char *array , unsigned int length);
void PrintString(unsigned char *string);


// Advanced print function
void PrintTime(Calendar *time);
void PrintHelp(void);


static const unsigned char HEX_ASCII_TABLE[16]={
		 '0',	 '1',	 '2',	 '3',	 '4',	 '5',	 '6',	 '7',
		 '8',	 '9',	 'A',	 'B',	 'C',	 'D',	 'E',	 'F',
};
#endif /* APP_UARTDEBUG_H_ */
