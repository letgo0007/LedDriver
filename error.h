#ifndef ERROR_H_
#define ERROR_H_

#include "driverlib.h"
#include "board.h"
#include "iw7027.h"

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

#endif /* ERROR_H_ */
