#ifndef IW7027_H_
#define IW7027_H_

#include "driverlib.h"
#include "board.h"

// IW7027 Hardware info
#define IW7027_DEVICE_AMOUNT			(5)		//IW7027 IC chip amount
#define IW7027_LED_CHANNEL				(78)	//Total valid LED channel number
#define IW7027_SPI_MASTER_TRANS_START_DELAY	(50)	//CS -> SPI delay(unit:us) , see IW7027 application note
#define IW7027_SPI_MASTER_TRANS_STOP_DELAY		(10)	//SPI -> CS delay(unit:us) , see IW7027 application note
#define IW_0							(BIT0)	//Chip Select bit ,orderd from BIT0~BIT7
#define IW_1							(BIT1)
#define IW_2							(BIT2)
#define IW_3							(BIT3)
#define IW_4							(BIT4)
#define IW_ALL							(0x1F)	//All device select bit

//Struct
enum Iw7027_Current
{
	i100mA,
	i150mA,
	i200mA,
	i250mA,
	i300mA,
	i350mA,
	i400mA
};

enum Iw7027_Frequency
{
	f50Hz,//0
	f60Hz,
	f100Hz,
	f120Hz
};

enum Iw7027_Delay
{
	d2D,
	d3D,
	d2Dscan,
	d3Dscan,
};

typedef struct Iw7027_ErrorInfo
{
	uint8_t iwIsError;
	uint16_t iwShort[IW7027_DEVICE_AMOUNT];
	uint16_t iwOpen[IW7027_DEVICE_AMOUNT];
	uint16_t iwDsShort[IW7027_DEVICE_AMOUNT];
}Iw7027_ErrorInfo;

//Buffers & Const Tables


static const uint8_t Iw7027_DefaultRegMap_70XU30A_78CH[ IW7027_DEVICE_AMOUNT * 0x60 ] =
{
/*IW_0		0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F	*/
/*0x0x*/	0x06,	0x00,	0x00,	0x00,	0x27,	0x00,	0x4E,	0x00,	0x76,	0x00,	0x9D,	0x00,	0xC4,	0x00,	0xEC,	0x01,
/*0x1x*/	0x13,	0x55,	0x3B,	0x62,	0x89,	0xB1,	0x40,	0xD8,	0x00,	0x27,	0x4E,	0x0F,	0x19,	0xD5,	0x0F,	0x18,
/*0x2x*/	0x1E,	0x0F,	0xFF,	0xFF,	0xFF,	0x00,	0x00,	0x42,	0x42,	0x0F,	0x12,	0x7F,	0xFF,	0xFF,	0xE5,	0xBC,
/*0x3x*/	0x7D,	0xD3,	0x01,	0x16,	0xC8,	0x80,	0x44,	0x00,	0xC0,	0xA0,	0x00,	0x78,	0x08,	0x28,	0x88,	0x88,
/*0x4x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
/*0x5x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,

/*IW_1		0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F	*/
/*0x0x*/	0x06,	0x00,	0x76,	0x00,	0x9D,	0x00,	0xC4,	0x00,	0xEC,	0x01,	0x13,	0x01,	0x3B,	0x01,	0x62,	0x01,
/*0x1x*/	0x89,	0x50,	0xB1,	0xD8,	0x00,	0x27,	0x00,	0x4E,	0x76,	0x9D,	0xC4,	0x33,	0xAA,	0x1E,	0x30,	0x3C,
/*0x2x*/	0x67,	0x0F,	0xFF,	0xFF,	0xFF,	0x00,	0x00,	0x42,	0x42,	0x0F,	0x12,	0x7F,	0xFF,	0xFF,	0xE5,	0xBC,
/*0x3x*/	0x7D,	0xD3,	0x01,	0x16,	0xC8,	0x80,	0x44,	0x00,	0xC0,	0xA0,	0x00,	0x78,	0x08,	0x28,	0x88,	0x88,
/*0x4x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
/*0x5x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,

/*IW_2		0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F	*/
/*0x0x*/	0x06,	0x00,	0xEC,	0x01,	0x13,	0x01,	0x3B,	0x01,	0x62,	0x01,	0x89,	0x01,	0xB1,	0x01,	0xD8,	0x00,
/*0x1x*/	0x00,	0x00,	0x27,	0x4E,	0x76,	0x9D,	0x05,	0xC4,	0xEC,	0x13,	0x3B,	0x54,	0x3C,	0x60,	0x78,	0xCE,
/*0x2x*/	0xA8,	0x0F,	0xFF,	0x3F,	0xFF,	0x00,	0x00,	0x42,	0x42,	0x0F,	0x12,	0x7F,	0xFF,	0xFF,	0xE5,	0xBC,
/*0x3x*/	0x7D,	0xD3,	0x01,	0x16,	0xC8,	0x80,	0x44,	0x00,	0xC0,	0xA0,	0x00,	0x78,	0x08,	0x28,	0x88,	0x88,
/*0x4x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
/*0x5x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,

/*IW_3		0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F	*/
/*0x0x*/	0x06,	0x01,	0x62,	0x01,	0x89,	0x01,	0xB1,	0x01,	0xD8,	0x00,	0x00,	0x00,	0x27,	0x00,	0x4E,	0x00,
/*0x1x*/	0x76,	0x01,	0x9D,	0xC4,	0xEC,	0x13,	0x55,	0x3B,	0x62,	0x89,	0xB1,	0x78,	0xC0,	0xF1,	0x9D,	0x50,
/*0x2x*/	0xF1,	0x0F,	0xFF,	0xFF,	0xFF,	0x00,	0x00,	0x42,	0x42,	0x0F,	0x12,	0x7F,	0xFF,	0xFF,	0xE5,	0xBC,
/*0x3x*/	0x7D,	0xD3,	0x01,	0x16,	0xC8,	0x80,	0x44,	0x00,	0xC0,	0xA0,	0x00,	0x78,	0x08,	0x28,	0x88,	0x88,
/*0x4x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
/*0x5x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,

/*IW_4		0		1		2		3		4		5		6		7		8		9		A		B		C		D		E		F	*/
/*0x0x*/	0x06,	0x01,	0xD8,	0x00,	0x00,	0x00,	0x27,	0x00,	0x4E,	0x00,	0x76,	0x00,	0x9D,	0x00,	0xC4,	0x00,
/*0x1x*/	0xEC,	0x55,	0x13,	0x3B,	0x62,	0x89,	0x50,	0xb1,	0xD8,	0x00,	0x00,	0x81,	0xE3,	0x3A,	0xA1,	0xE3,
/*0x2x*/	0x00,	0x0F,	0xFF,	0xFF,	0xFF,	0x00,	0x00,	0x42,	0x42,	0x0F,	0x12,	0x7F,	0xFF,	0xFF,	0xE5,	0xBC,
/*0x3x*/	0x7D,	0xD3,	0x01,	0x16,	0xC8,	0x80,	0x44,	0x00,	0xC0,	0xA0,	0x00,	0x78,	0x08,	0x28,	0x88,	0x88,
/*0x4x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
/*0x5x*/	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,	0x01,	0x99,
};

static const uint8_t Iw7027_LedSortMap_70XU30A_78CH[80]=
{
		//13 x 6 LED Matirx order in TV front view
		17		,		16		,		15		,		77		,		78		,		79		,
		14		,		13		,		12		,		74		,		75		,		76		,
		11		,		10		,		9		,		71		,		72		,		73		,
		8		,		7		,		6		,		68		,		69		,		70		,
		5		,		4		,		3		,		65		,		66		,		67		,
		2		,		1		,		0		,		62		,		63		,		64		,
		38		,		37		,		36		,		59		,		60		,		61		,
		35		,		34		,		33		,		56		,		57		,		58		,
		32		,		31		,		30		,		53		,		54		,		55		,
		29		,		28		,		27		,		50		,		51		,		52		,
		26		,		25		,		24		,		45		,		48		,		49		,
		23		,		22		,		21		,		42		,		43		,		44		,
		20		,		19		,		18		,		39		,		40		,		41		,
		//not used ports
		46		,		47
};

//Function Calls
/**********************************************************
 * @Brief Iw7027_writeMultiByte
 * 		Write Multiple bytes to IW7027 , bsaed on SPI interface spec.
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress	: Register address to write.
 * 		length		: Amount of multiple bytes to transfer.
 * 		*txdata		: Multiple bytes pointer.
 * @Return
 * 		NONE
 **********************************************************/
void Iw7027_writeMultiByte(uint8_t chipsel, uint8_t regaddress, uint8_t length , uint8_t *txdata);

/**********************************************************
 * @Brief Iw7027_writeSingleByte
 * 		Write Single byte to IW7027 , bsaed on SPI interface spec.
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress 	: Register address to write.
 * 		txdata		: 1 byte of data.
 * @Return
 * 		NONE
 **********************************************************/
void Iw7027_writeSingleByte(uint8_t chipsel, uint8_t regaddress, uint8_t txdata);

/**********************************************************
 * @Brief Iw7027_readSingleByte
 * 		Read single byte to IW7027 , bsaed on SPI interface spec.
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress 	: Register address to write.
 * @Return
 * 		readbyte 	: Reveived bytes.
 **********************************************************/
uint8_t Iw7027_readSingleByte(uint8_t chipsel, uint8_t regaddress);

/**********************************************************
 * @Brief Iw7027_checkReadWithTimeout
 * 		Loop checking IW7027 read value with timeout function.
 * 		Check max 10 times for very selected IW device .
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress 	: Register address to write.
 * 		checkvalue 	: Target value to be compared with.
 * 		bitmask		: Select which bit to be compared with.
 * @Return
 * 		STATUS_SUCCESS 	: Read value correct . ( readbyte & bitmask == checkvalue )
 * 		STATUS_FAIL		: Read value wrong for 10 times .
 **********************************************************/
uint8_t Iw7027_checkReadWithTimeout(uint8_t chipsel, uint8_t regaddress , uint8_t checkvalue ,uint8_t bitmask);

/**********************************************************
 * @Brief Iw7027_init
 * 		Initialize IW7027 under Start Sequence Spec.
 * 		Over write IW7027 initial register map to all device.
 * @Param
 * 		*workmodetable 	: Pointer to Register map table for IW7027.
 * 						  Note that this table size must be [ IW7027_DEVICE_AMOUNT * 0x60 ].
 * @Return
 * 		STATUS_SUCCESS 	: Initial success.
 * 		STATUS_FAIL		: Initial fail.
 **********************************************************/
uint8_t Iw7027_init(const uint8_t *workmodetable);

/**********************************************************
 * @Brief Iw7027_updateDuty
 * 		Update duty info to all Iw7027 (from 0x40~0x5F).
 * 		Generally this function should be run every frame.
 * @Param
 * 		*dutymatrix 	: Pointer to Duty info matrix.
 * 						  Note that this table is orderd by TV front view .
 * 						  Dutymatrix[0] is the Top Left local of TV front view.
 * 						  Dutymatrix[LAST] is the Bottom Right local of TV front view.
 * 		*ledsortmap		: Pointer to Led_Sort_Map const table.
 * 						  This table is to convert duty matrix from TV front view order to Hadware order.
 * 						  Refer to LED_Sort_Map_Generator.xlsx
 * @Buffer
 * 		Iw7027_SortBuff : Temp buffer to store sorted duty info.
 * 						  Buff[0x00] is for 0x40 of IW_0
 * 						  Buff[0x1F] is for 0x5F of IW_0
 * 						  Buff[last] is for 0x5F of IW_N
 * @Return
 * 		STATUS_SUCCESS 	: Initial success.
 * 		STATUS_FAIL		: Initial fail.
 **********************************************************/
uint8_t Iw7027_updateDuty(uint16_t *dutymatrix ,const uint8_t *ledsortmap);

/**********************************************************
 * @Brief Iw7027_updateDuty
 * 		Update current setting to all Iw7027 (0x27).
 * @Param
 * 		current			: enum of Iw7027Current i100mA ~ i400mA
 * @Return
 * 		STATUS_SUCCESS 	: Update success.
 * 		STATUS_FAIL		: Current not support.
 **********************************************************/
uint8_t Iw7027_updateCurrent(enum Iw7027_Current current);

/**********************************************************
 * @Brief Iw7027_updateFrequency
 * 		Update PLL setting to all Iw7027 (0x21 0x22 0x2F 0x31).
 * 		Gereranlly this function should run every time input frequency change.
 * @Param
 * 		freq			: enum of Iw7027Frequency f50Hz f60Hz ...
 * @Return
 * 		STATUS_SUCCESS 	: Update success.
 * 		STATUS_FAIL		: Frequency not support.
 **********************************************************/
uint8_t Iw7027_updateFrequency(enum Iw7027_Frequency freq);


/**********************************************************
 * @Brief Iw7027_updateDelayTable
 *
 * On working.
 * ********************************************************/
uint8_t Iw7027_updateDelayTable(enum Iw7027_Delay delay);

/**********************************************************
 * @Brief Iw7027_getErrorStatus
 * 		Detect IW7027 all device open / short / d-s short status.
 * @Param
 * 		NONE			: enum form i100mA ~ i400mA
 * @Return
 * 		Iw7027Error 	: Error info struct of Iw7027Error
 **********************************************************/
Iw7027_ErrorInfo Iw7027_getErrorStatus(void);



#endif /* IW7027_H_ */
