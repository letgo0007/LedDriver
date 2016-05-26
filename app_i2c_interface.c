#include "app.h"
#include "driver.h"

uint8_t I2cSlave_handleSpecialFunction(uint8_t *sfbuff)
{
	/*Special Function 1 [SPI direct access function]
	#define I2C_SPIACCESS_WRMODE		(0x00)
	#define I2C_SPIACCESS_CHIPSEL		(0x01)
	#define I2C_SPIACCESS_REGADD		(0x02)
	#define I2C_SPIACCESS_TXDATA		(0x03)
	#define I2C_SPIACCESS_RXDATA		(0x04)
	*/
	switch( sfbuff[I2C_SPIACCESS_WRMODE] )
	{
		case 0x80://Single Read
			sfbuff[ I2C_SPIACCESS_RXDATA ] = Iw7027_readSingleByte( sfbuff[I2C_SPIACCESS_CHIPSEL] , sfbuff[I2C_SPIACCESS_REGADD] );
			break;
		case 0x81://Single Write
			Iw7027_writeSingleByte(	sfbuff[I2C_SPIACCESS_CHIPSEL] ,	sfbuff[I2C_SPIACCESS_REGADD] ,	sfbuff[I2C_SPIACCESS_TXDATA] );
			break;
		default://No operation
			break;
	}
	sfbuff[I2C_SPIACCESS_WRMODE] = 0;

	//I2C Function 2 [Manual pattern]
	/* Value @ I2C_MANUAL_WRMODE
	 * [00] Read param from System_ManualDutyBuff (default)
	 * [80] Write param to System_ManualDutyBuff
	 *
	#define I2C_MANUAL_WRMODE			(0x10)
	#define I2C_MANUAL_DUTY_H			(0x11)
	#define I2C_MANUAL_DUTY_L			(0x12)
	#define I2C_MANUAL_START_CH			(0x13)
	#define I2C_MANUAL_END_CH			(0x14)
	*/
	switch( sfbuff[I2C_MANUAL_WRMODE] )
	{
		case 0x80://Write buffer
		{
			uint16_t i = 0;
			uint16_t duty = 0x0100 * sfbuff[I2C_MANUAL_DUTY_H] + sfbuff[I2C_MANUAL_DUTY_L] ;

			for( i = sfbuff[ I2C_MANUAL_START_CH ]; i < sfbuff[ I2C_MANUAL_END_CH ] ; i++ )
			{
				System_ManualDutyBuff[ i ] = duty ;
			}

			break;
		}
		default ://Read buffer
			break;
	}
	sfbuff[I2C_MANUAL_WRMODE] = 0x00 ;

	return STATUS_SUCCESS;

}

