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
 *****************************************************************************/
#ifndef DRV_UART_H_
#define DRV_UART_H_

/***1 Includes ***************************************************************/

#include "hal.h"
#include "std.h"

/***2.1 External Macros ******************************************************/
#ifndef UART_DEBUG_ON
#define UART_DEBUG_ON						(1)
#endif

/***2.2 External Structures **************************************************/

/***2.3 External Variables ***************************************************/

/***2.4 External Functions ***************************************************/
extern void Uart_Console(uint8 *uartrxbuf);

extern void PrintChar(unsigned char data);
extern void PrintCharBCD(unsigned char data);
extern void PrintEnter(void);
extern void PrintInt(unsigned int data);
extern void PrintArray(unsigned char *array, unsigned int length);
extern void PrintString(unsigned char *string);
extern void PrintTime(Hal_Time *time);

#endif /* DRV_UART_H_ */
