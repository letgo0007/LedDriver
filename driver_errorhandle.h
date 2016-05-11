#ifndef DRIVER_ERRORHANDLE_H_
#define DRIVER_ERRORHANDLE_H_

#include "driver.h"

#define	ERROR_SEGMENT_PTR	 ( 0x1800 )		//Flash segment pointer to store error info

typedef struct ErrorInfo
{
	uint8_t errorCount;
	BoardInfo errorBoard;
	Calendar errorTime;
	Iw7027_ErrorInfo errorIW7027;
}ErrorInfo;

ErrorInfo System_ErrorInfo ;

uint8_t Error_storeErrorInfo(ErrorInfo error);
ErrorInfo Error_readErrorInfo(void);

#endif /* DRIVER_ERRORHANDLE_H_ */
