/******************************************************************************
 * @file 	[std.h]
 *
 * Standard of Sharp coding rule.
 * Formater base = BSD/Allman format , line width modified from 80 to 120 .
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
#ifndef STD_H_
#define STD_H_

/*Standard int define , note that on 16bit MCU , int = 16bit .*/
typedef signed char int8;
typedef unsigned char uint8;
typedef int int16;
typedef unsigned int uint16;
typedef long int32;
typedef unsigned long uint32;
typedef long long int64;
typedef unsigned long long uint64;
typedef _Bool flag;

/*API standard return value for flag*/
#define FLAG_SUCCESS			(1)
#define FLAG_FAIL				(0)
/*BIT Mask*/
#define BIT0                   (0x0001)
#define BIT1                   (0x0002)
#define BIT2                   (0x0004)
#define BIT3                   (0x0008)
#define BIT4                   (0x0010)
#define BIT5                   (0x0020)
#define BIT6                   (0x0040)
#define BIT7                   (0x0080)
#define BIT8                   (0x0100)
#define BIT9                   (0x0200)
#define BITA                   (0x0400)
#define BITB                   (0x0800)
#define BITC                   (0x1000)
#define BITD                   (0x2000)
#define BITE                   (0x4000)
#define BITF                   (0x8000)

/*Simple math marco*/
inline int max(int a, int b)
{
	return a > b ? a : b;
}
inline int min(int a, int b)
{
	return a < b ? a : b;
}

/*Direct access to memory access*/
#ifndef HWREG32
#define HWREG32(x)    (*((volatile uint32 *)((uint16)x)))
#endif
#ifndef HWREG16
#define HWREG16(x)    (*((volatile uint16 *)((uint16)x)))
#endif
#ifndef HWREG8
#define HWREG8(x)     (*((volatile uint8 *)((uint16)x)))
#endif

/*.h file common structure*/
/***1 Includes ***************************************************************/
/***2.1 External Macros ******************************************************/
/***2.2 External Structures **************************************************/
/***2.3 External Variables ***************************************************/
/***2.4 External Functions ***************************************************/

/*.c file common structure*/
/***1 Includes ***************************************************************/
/***2.1 Internal Marcos ******************************************************/
/***2.2 Internal Struct ******************************************************/
/***2.3 Internal Variables ***************************************************/
/***2.4 External Variables ***************************************************/
/***2.5 Internal Functions ***************************************************/
/***2.6 External Functions ***************************************************/

#endif /* STD_H_ */
