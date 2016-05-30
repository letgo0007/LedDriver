/******************************************************************************
 * @file 	[driver_iw7027.h]
 *
 * IW7027 LED Driver IC driver.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
#ifndef DRIVER_IW7027_H_
#define DRIVER_IW7027_H_

/***1 Includes ***************************************************************/

#include "std.h"

/***2.1 External Macros ******************************************************/
//Driver Software Version
#define	IW_DRIVER_VERSION				(0x01)
//IW7027 IC amount to be controled.
#define IW_DEVICE_AMOUNT				(0x05)
//Total available LED channel amount.
#define IW_LED_CHANNEL					(78)
//Single chip select bit ,orderd from BIT0~BIT7.
#define IW_0							(0x01)
#define IW_1							(0x02)
#define IW_2							(0x04)
#define IW_3							(0x08)
#define IW_4							(0x10)
//All chip select bit .
#define IW_ALL							(0x1F)

/***2.2 External Structures **************************************************/
//IW7027 delay table select.
enum Iw7027_Delay
{
	d2D = 0, d3D = 1, d2Dscan = 2, d3Dscan = 3,
};
//IW7027 working paramters struct
typedef struct Iw7027Param
{
	//IW7027 PLL frequency,unit in Hz.
	uint8 iwFrequency;
	//IW7027 current.
	uint8 iwCurrent;
	//IW7027 delaytable seletc.
	enum Iw7027_Delay iwDelayTableSelet;
	//IW7027 Vsync frequency, unit in Hz.
	uint8 iwVsyncFrequency;
	//IW7027 Vsync delay, unit in 30.5us(32786Hz).
	uint8 iwVsyncDelay;
	//IW7027 Open/Short check enable. Set to 1 to run error check once.
	uint8 iwRunErrorCheck;
	//IW7027 Error status , [1] = Error , [0] = Normal.
	uint8 iwIsError;
	//IW7027 Open/Short Status. Each IW7027 has 6bytes
	// 6 bytes : 	[open 0~7][open 8~15][short 0~7][short 8~15]
	// 				[D-S short 0~7][D-S short 8~15]
	uint8 iwOpenShortStatus[IW_DEVICE_AMOUNT * 6];
	//RESERVED , fix the size of the struct to 0x30 bytes for I2C slave access.
	uint8 reserved[0x08];
} Iw7027Param;

/***2.3 External Variables ***/
extern Iw7027Param SysParam_Iw7027;

/***2.4 External Functions ***/
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
extern void Iw7027_writeMultiByte(uint8 chipsel, uint8 regaddress, uint8 length, uint8 *txdata);

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
extern void Iw7027_writeSingleByte(uint8 chipsel, uint8 regaddress, uint8 txdata);

/**********************************************************
 * @Brief Iw7027_readSingleByte
 * 		Read single byte to IW7027 , bsaed on SPI interface spec.
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress 	: Register address to write.
 * @Return
 * 		readbyte 	: Reveived bytes.
 **********************************************************/
extern uint8 Iw7027_readSingleByte(uint8 chipsel, uint8 regaddress);

/**********************************************************
 * @Brief Iw7027_readMultiByte
 * 		Read multiple byte to IW7027 , bsaed on SPI interface spec 3.10. Read N data
 * @Param
 * 		chipsel 	: Chip active select , valid from IW_0~IW_ALL.
 * 		regaddress 	: Register address to write.
 * 		length		: Amount of multiple bytes to transfer.
 * 		*rxdata		: Multiple bytes read pointer.
 * @Return
 * 		readbyte 	: Reveived bytes.
 **********************************************************/
extern uint8 Iw7027_readMultiByte(uint8 chipsel, uint8 regaddress, uint8 length, uint8 *rxdata);

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
 * 		FLAG_SUCCESS 	: Read value correct . ( readbyte & bitmask == checkvalue )
 * 		FLAG_FAIL		: Read value wrong for 10 times .
 **********************************************************/
extern uint8 Iw7027_checkReadWithTimeout(uint8 chipsel, uint8 regaddress, uint8 checkvalue, uint8 bitmask);

/**********************************************************
 * @Brief Iw7027_init
 * 		Initialize IW7027 under Start Sequence Spec.
 * 		Over write IW7027 initial register map to all device.
 * @Param
 * 		*workmodetable 	: Pointer to Register map table for IW7027.
 * 						  Note that this table size must be [ IW7027_DEVICE_AMOUNT * 0x60 ].
 * @Return
 * 		FLAG_SUCCESS 	: Initial success.
 * 		FLAG_FAIL		: Initial fail.
 **********************************************************/
extern uint8 Iw7027_init(void);

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
 * 		FLAG_SUCCESS 	: Initial success.
 * 		FLAG_FAIL		: Initial fail.
 **********************************************************/
extern uint8 Iw7027_updateDuty(uint16 *dutymatrix);

/**********************************************************
 * @Brief Iw7027_updateDuty
 * 		Update current setting to all Iw7027 (0x27).
 * @Param
 * 		current			: enum of Iw7027Current i100mA ~ i400mA
 * @Return
 * 		FLAG_SUCCESS 	: Update success.
 * 		FLAG_FAIL		: Current not support.
 **********************************************************/
extern uint8 Iw7027_updateCurrent(uint8 current);

/**********************************************************
 * @Brief Iw7027_updateFrequency
 * 		Update PLL setting to all Iw7027 (0x21 0x22 0x2F 0x31).
 * 		Gereranlly this function should run every time input frequency change.
 * @Param
 * 		freq			: enum of Iw7027Frequency f50Hz f60Hz ...
 * @Return
 * 		FLAG_SUCCESS 	: Update success.
 * 		FLAG_FAIL		: Frequency not support.
 **********************************************************/
extern uint8 Iw7027_updateFrequency(uint8 freq);

/**********************************************************
 * @Brief Iw7027_updateDelayTable
 *
 * On working.
 * ********************************************************/
extern uint8 Iw7027_updateDelayTable(enum Iw7027_Delay delay);

/**********************************************************
 * @Brief Iw7027_getErrorStatus
 * 		Check IW7027 Open/Short/Ds Short status from IW7027 registers.
 * @Param
 * 		iwparam		: target IW7027_WorkParam .
 * @Return
 * 		1 			: IW7027 has Open Short Error
 * 		0			: IW7027 ok , no error .
 **********************************************************/
extern uint8 Iw7027_checkOpenShorStatus(Iw7027Param *iwparam);

/**********************************************************
 * @Brief Iw7027_getErrorStatus
 * 		This fucntion include the following sub function with change check function.
 * 		Only update changed parameters.
 * 		Iw7027_updateCurrent
 * 		Iw7027_updateFrequency
 * 		Iw7027_updateDelayTable
 * 		PwmOut_init
 * @Param
 * 		iwparam			: target IW7027_WorkParam .
 * @Return
 * 		Iw7027Error 	: Error info struct of Iw7027Error
 **********************************************************/
extern uint8 Iw7027_updateWorkParams(Iw7027Param *iwparam);

/**********************************************************
 * @Brief DPL_GammaUpdate
 * 		Update Input Gamma curve accroding to 5 Gamma point input.
 * @Param
 * 		gp0				: Gamma point @ 0
 * 		gp63			: Gamma point @ 63
 * 		gp127			: Gamma point @ 127
 * 		gp191			: Gamma point @ 191
 * 		gp255			: Gamma point @ 255
 * @Return
 * 		NONE
 **********************************************************/

#endif /* DRIVER_IW7027_H_ */
