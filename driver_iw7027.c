#include "driver_iw7027.h"

#include "string.h"

#define debuglog	(1)

void Iw7027_writeMultiByte(uint8_t chipsel, uint8_t regaddress, uint8_t length , uint8_t *txdata)
{
	//Selest chip
	SpiMaster_setCsPin(chipsel);
	delay_us(IW7027_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte( 0x01 );
	SpiMaster_sendSingleByte( length );
	SpiMaster_sendSingleByte( regaddress );

	//Send multi data
	SpiMaster_sendMultiByte( txdata , length );

	//Unselest all chip
	delay_us(IW7027_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin( 0x00 );

}

void Iw7027_writeSingleByte(uint8_t chipsel, uint8_t regaddress, uint8_t txdata)
{
	//Selest chip
	SpiMaster_setCsPin(chipsel);
	delay_us(IW7027_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte( 0xC0 );
	SpiMaster_sendSingleByte( regaddress );

	//Send Data
	SpiMaster_sendSingleByte( txdata );

	//Unselest all chip
	delay_us(IW7027_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin( 0x00 );
}

uint8_t Iw7027_readSingleByte(uint8_t chipsel, uint8_t regaddress)
{
	uint8_t readbyte;

	//Selest chip
	SpiMaster_setCsPin(chipsel);
	delay_us(IW7027_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte( 0x41 );
	SpiMaster_sendSingleByte( regaddress | 0x80 );

	//2 dummy clocks to read byte
	SpiMaster_sendSingleByte( 0x00 );
	readbyte = SpiMaster_sendSingleByte( 0x00 );

	//Unselest all chip
	delay_us(IW7027_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin(0x00);

	return readbyte;
}

uint8_t Iw7027_checkReadWithTimeout(uint8_t chipsel, uint8_t regaddress , uint8_t checkvalue ,uint8_t bitmask)
{
	uint8_t retrytime;
	uint8_t val;
	uint8_t chipmask;
	uint8_t status;

	//Sequence check from IW_0 -> IW_N
	for( chipmask = IW_0 ; chipmask < IW_ALL ; chipmask = chipmask<<1 )
	{
		//Every selected chip check 10 times.
		//If any of the chip check timeout ,return fail.
		if(chipsel & chipmask)
		{
			retrytime = 10;
			status = 1;
			while( --retrytime && status)
			{
				val = Iw7027_readSingleByte ( (chipsel & chipmask) , regaddress ) ;

				if ( ( val & bitmask ) == checkvalue )
				{
					//Correct , set status = 0 to stop loop
					status = 0;
				}
				else
				{
					//Wrong , delay 100us to retry.
					delay_us(100);
				}
			}
			if (retrytime == 0)
			{
				//if time out , return fail
				return STATUS_FAIL;
			}
		}
	}

	//Check loop finish ,return success .
	return STATUS_SUCCESS;

}

uint8_t Iw7027_init(const uint8_t *workmodetable)
{
	uint8_t status = 0;
	uint8_t i;

#if debuglog
	PrintTime(&System_Time);
	PrintString("Start IW7027 Initial Sequence.\r\n");
#endif

	//Step 1 : Check Power
	do{
		Board_getBoardInfo(&System_BoardInfo);

#if debuglog
		PrintTime(&System_Time);
		PrintString("Check DC Power , NO.");
		PrintChar(i++);
		PrintString("\r\nD60V = ");
		PrintCharBCD(System_BoardInfo.boardD60V);
		PrintString("  D13V = ");
		PrintCharBCD(System_BoardInfo.boardD13V);
		PrintEnter();
		delay_ms(100);
#endif

	}while( ( System_BoardInfo.boardD60V < 50 ) || ( System_BoardInfo.boardD13V < 10 ) );

	i = 0;
	//Step 1: check chip ID to ensure IW7027 is working .
	do{
		status = Iw7027_checkReadWithTimeout( IW_ALL , 0x6B , 0x24 , 0xFF );
		PrintTime(&System_Time);
		PrintString("Check IW7027 CHIP ID NO.");
		PrintChar(i++);
		PrintEnter();
	}while (status == 0);

	//Step 2 : Write Initial setting in sequence  from chip IW0 to IW_N
	for( i = 0 ; i < IW7027_DEVICE_AMOUNT ; i ++ )
	{
		PrintTime(&System_Time);
		PrintString("Writing Initial Reg to IW7027 NO.");
		PrintChar(i);
		PrintEnter();
		Iw7027_writeMultiByte( IW_0<<i , 0x00 , 0x60 , (uint8_t *)(workmodetable + 0x60 * i) );
	}

	//Step 3 :wait STB
#if 0
	do{
		Board_getBoardInfo(&System_BoardInfo);
#if debuglog
		PrintTime(&System_Time);
		PrintString("Waiting STB...\r\n");
		delay_ms(100);
#endif
	}while( GPIO_getInputPinValue(GPIOPORT_STB_HW,GPIOPIN_STB_HW) == 0 );
#endif

	//Step 4 : Delay & turn on IW7027
	delay_ms(500);
	Iw7027_writeSingleByte(IW_ALL,0x00,0x07);

	return STATUS_SUCCESS;
}

uint8_t Iw7027_updateDuty(uint16_t *dutymatrix ,const uint8_t *ledsortmap)
{
	uint8_t i;

	static uint8_t Iw7027_SortBuff[ IW7027_DEVICE_AMOUNT * 32 ];

	//Sort duty matrix by LED_sort_map
	for( i = 0 ; i < IW7027_LED_CHANNEL ; i++ )
	{
		//convert 1 16bit data -> 2 8bit data . with resorted in LED hardware order
		Iw7027_SortBuff [ 2 * ledsortmap [ i ]  ] 	= dutymatrix[ i ] >> 8;
		Iw7027_SortBuff [ 2 * ledsortmap [ i ] + 1 ] = dutymatrix[ i ] & 0xFF;
	}

	//Sequence write chip IW0 ~ IW_N
	for( i = 0 ; i < IW7027_DEVICE_AMOUNT ; i ++ )
	{
		Iw7027_writeMultiByte( IW_0<<i  , 0x40 , 32 , Iw7027_SortBuff + 32 * i );
	}

	return STATUS_SUCCESS;
}

uint8_t Iw7027_updateCurrent(enum Iw7027_Current current)
{
	uint8_t ival;

	//Determine current register value accroding to Hw design
	switch (current)
	{
	case i100mA:
		ival =  0x30;
		break;
	case i200mA:
		ival =  0x42;
		break;
	case i300mA:
		ival =  0xA2;
		break;
	case i350mA:
		ival =  0xD0;
		break;
	default :
		return STATUS_FAIL;
	}

	//Write data to IW7027
	//1 . Disable Protect , Set [FAUL_LOCK] (0x62  BIT0)to 1
	Iw7027_writeSingleByte( IW_ALL , 0x62 , 0x01 );

	//2 . Write current to 0x27
	Iw7027_writeSingleByte( IW_ALL , 0x27 , ival );

	//3 . Check status. Low 4 bit of 0xB3 = 0x05
	Iw7027_checkReadWithTimeout( IW_ALL , 0xB3 , 0x05 , 0x0F);

	//4 . Enable Protect , Set [FAULT LOCK] (0x62  BIT0)to 0 ,IDAC_REMAP + FAUL_LOCK
	Iw7027_writeSingleByte( IW_ALL , 0x62 , 0x00 );

	return ival;
}

uint8_t Iw7027_updateFrequency(enum Iw7027_Frequency freq)
{
	uint8_t pllval;

	//Determine current register value accroding to Iw7027 datasheet
	switch (freq)
	{
	case f50Hz:
		pllval = 48;
		break;
	case f60Hz:
		pllval = 40;
		break;
	case f100Hz:
		pllval = 24;
		break;
	case f120Hz:
		pllval = 20;
		break;
	default :
		return STATUS_FAIL;
	}

	//1 .	Set [PLL_OUTDIV] (0x31 BIT0~BIT4) , Set [PLL_EN] (0x31 BIT7) = 1
	Iw7027_writeSingleByte( IW_ALL , 0x31 , pllval + BIT7 );

	//2.	Set [PWM_PER_SEL] (0x2F BIT7) =1,Load PWM period from register 0x21[4:0] and 0x22[7:0]
	Iw7027_writeSingleByte( IW_ALL , 0x2F , BIT7) ;

	//3.	Set [PWM_PER] (0x21 BIT0~BIT4 , 0x22 BIT0~BIT7 ) to 4095 (0x0fff)
	Iw7027_writeSingleByte( IW_ALL , 0x21 , 0x0F );
	Iw7027_writeSingleByte( IW_ALL , 0x22 , 0xFF );

	//5.	Check [RO_PLL_LOCK]	(0x84 BIT4)	to ensuer PLL work well
	if ( Iw7027_checkReadWithTimeout( IW_ALL , 0x84 , BIT4 , BIT4 ) )
	{
		//return pll value when success
		return pllval;
	}
	else
	{
		return STATUS_FAIL;
	}
}

uint8_t Iw7027_updateDelayTable(enum Iw7027_Delay delay)
{
	return STATUS_SUCCESS;
}


Iw7027_ErrorInfo Iw7027_getErrorStatus(void)
{
	uint8_t i;
	uint16_t temph,templ;
	Iw7027_ErrorInfo errortemp = {0} ;

	//Enable Error Read
	Iw7027_writeSingleByte(IW_ALL,0x78, 0x80);

	//Loop check from IW_0 -> IW_N
	for( i = 0 ; i < IW7027_DEVICE_AMOUNT ; i++ )
	{
		templ = Iw7027_readSingleByte ( IW_0<<1 , 0x85) ;
		temph = Iw7027_readSingleByte ( IW_0<<1 , 0x86) ;
		errortemp.iwShort[i] = temph << 8 + templ ;

		templ = Iw7027_readSingleByte ( IW_0<<1 , 0x87) ;
		temph = Iw7027_readSingleByte ( IW_0<<1 , 0x88) ;
		errortemp.iwOpen[i] = temph << 8 + templ ;

		templ = Iw7027_readSingleByte ( IW_0<<1 , 0x89) ;
		temph = Iw7027_readSingleByte ( IW_0<<1 , 0x8A) ;
		errortemp.iwDsShort[i] = temph << 8 + templ ;

		//If detect any error , set iwIsError flag
		if( errortemp.iwShort[i] || errortemp.iwShort[i] || errortemp.iwShort[i] )
		{
			errortemp.iwIsError = 0x01 ;
		}
	}

	//Disable Error¡¡Read
	Iw7027_writeSingleByte(IW_ALL,0x78, 0x00);

	return errortemp;
}
