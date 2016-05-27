/******************************************************************************
 * @file 	[driver_uartdebug.h]
 *
 * Debug uart port function headers.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
#ifndef DRIVER_UARTDEBUG_H_
#define DRIVER_UARTDEBUG_H_

/*1 Includes */

#include "driverlib/MSP430F5xx_6xx/rtc_a.h"
#include "std.h"

/*2.1 External Macros */
#define	UART_DEBUG_ON					(1)

/*2.2 External Structures */

/*2.3 External Variables */

/*2.4 External Functions */
extern void Uart_Console(uint8 *uartrxbuf);
extern void UartSendChar(unsigned char data);
extern void UartSendInt(unsigned int data);
extern void PrintChar(unsigned char data);
extern void PrintCharBCD(unsigned char data);
extern void PrintEnter(void);
extern void PrintInt(unsigned int data);
extern void PrintArray(unsigned char *array, unsigned int length);
extern void PrintString(unsigned char *string);
extern void PrintTime(Calendar *time);

#endif /* DRIVER_UARTDEBUG_H_ */
