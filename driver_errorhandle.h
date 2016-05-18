#ifndef DRIVER_ERRORHANDLE_H_
#define DRIVER_ERRORHANDLE_H_

#include "driver.h"

#define	ERROR_SEGMENT_PTR	 ( 0x1800 )		//Flash segment pointer to store error info

typedef struct ErrorParam
{
	uint8_t eIsError;
	uint8_t eCount;
	uint8_t	eDc60vMax;
	uint8_t	eDc60vMin;
	uint8_t	eDc13vMax;
	uint8_t	eDc13vMin;
	uint8_t	eSpiRxFreqMin;
	uint8_t eSpiRxFormatCheck;
	uint8_t eIw7027ErrorCheck;
	uint8_t eIW7027ErrorInfo;
}ErrorParam;

ErrorParam System_ErrorParam ;

uint8_t Error_storeErrorInfo(void);
<<<<<<< HEAD
uint8_t Mcu_setErrorOut(BoardInfo *boardinfo , ErrorParam *errorparam);
=======
uint8_t Board_setErrorOut(BoardInfo *boardinfo , ErrorParam *errorparam);
>>>>>>> origin/master

#endif /* DRIVER_ERRORHANDLE_H_ */
