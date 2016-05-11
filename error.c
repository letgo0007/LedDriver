#include "error.h"
#include "string.h"

uint8_t Error_storeErrorInfo(ErrorInfo error)
{
	FlashCtl_eraseSegment( (uint8_t *)HWREG8(ERROR_SEGMENT_PTR) );
	FlashCtl_write8( (uint8_t *)&error ,(uint8_t *)HWREG8(ERROR_SEGMENT_PTR) , sizeof(error) );
	return STATUS_SUCCESS;
}



ErrorInfo Error_readErrorInfo(void)
{
	ErrorInfo errtemp;

	memcpy(&errtemp,(const void*)HWREG8(ERROR_SEGMENT_PTR),sizeof(errtemp));

	return errtemp;
}

uint8_t Error_checkErrorStatus(void)
{
	return STATUS_SUCCESS;
}
