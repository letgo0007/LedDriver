#include "driver_errorhandle.h"

#include "string.h"

uint8_t Error_storeErrorInfo(void)
{
	return 1;
}

<<<<<<< HEAD
uint8_t Mcu_setErrorOut(BoardInfo *boardinfo , ErrorParam *errorparam)
=======
uint8_t Board_setErrorOut(BoardInfo *boardinfo , ErrorParam *errorparam)
>>>>>>> origin/master
{
	uint8_t status = 1;

	//If power error ,reset MCU
	status &=  (boardinfo->boardD13V <= errorparam->eDc13vMax) || (boardinfo->boardD13V >= errorparam->eDc13vMin) ;
	status &=  (boardinfo->boardD60V <= errorparam->eDc60vMax) || (boardinfo->boardD60V >= errorparam->eDc60vMin) ;

	if(status == 0)
	{
		PrintString("\r\nLow Power Detect....Shut Down...\r\n");
		Mcu_reset();
	}

	//If Spi data error
	status &=  (boardinfo->boardSpiRxFreq >= errorparam->eSpiRxFreqMin) ;
	status &=  (boardinfo->boardSpiRxValid || !errorparam->eSpiRxFormatCheck) ;
	status &=  boardinfo->boardIw7027Falut ;

	if(status)
	{
		errorparam->eIsError = 0;
<<<<<<< HEAD
		SET_ERROR_OUT_HIGH ;
=======
		SET_ERROR_OUT_LOW ;
>>>>>>> origin/master
	}
	else
	{
		errorparam->eIsError = 1;
<<<<<<< HEAD
		SET_ERROR_OUT_LOW;
=======
		SET_ERROR_OUT_HIGH;
>>>>>>> origin/master
	}


	return 1;
}
