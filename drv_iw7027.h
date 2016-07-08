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
#ifndef DRV_IW7027_H_
#define DRV_IW7027_H_

/***1 Includes ***************************************************************/

#include "std.h"

/***2.1 External Macros ******************************************************/
/* Model information
 * 65SX970A + MSP430F5529 LaunchPad
 * 8*15(120ch) LED Matrix
 */
#define IW_DEVICE_AMOUNT				(8)
#define IW_LED_COL						(15)
#define IW_LED_ROW						(8)
#define IW_LED_CHANNEL					(IW_LED_COL * IW_LED_ROW)
#define Iw7027_DefaultRegMap			(Iw7027_DefaultRegMap_65SX970A)
#define Iw7027_LedSortMap				(Iw7027_LedSortMap_65SX970A)
//Chip select bit ,orderd from BIT0~BIT7.
#define IW_0							(0x0001)
#define IW_1							(0x0002)
#define IW_2							(0x0004)
#define IW_3							(0x0008)
#define IW_4							(0x0010)
#define IW_5							(0x0020)
#define IW_6							(0x0040)
#define IW_7							(0x0080)
#define IW_ALL							(0x00FF)
//Error return type
#define IW_ERR_SHORT					(0x01)
#define IW_ERR_OPEN						(0x02)
#define IW_ERR_DSSHORT					(0x04)

/***2.2 External Structures **************************************************/
//IW7027 delay table select.
enum Iw7027_Delay_e
{
	d2D = 0, d3D = 1, d2Dscan = 2, d3Dscan = 3,
};
//IW7027 working paramters struct
typedef struct Drv_Iw7027Param_t
{
	//IW7027 current.
	uint8 u8IwCurrent;
	//IW7027 Vsync frequency, unit in Hz.
	uint16 u16IwInputFreq;
	//IW7027 PLL frequency,unit in Hz.
	uint16 u16IwOutputFreq;
	//IW7027 Vsync delay, unit in 30.5us(32786Hz).
	uint16 u16IwOutputDelay;
	//IW7027 delaytable seletc.
	enum Iw7027_Delay_e eIwDelayTableSelect;
	//IW7027 Open/Short check enable. Set to 1 to run error check once.
	flag fIwRunErrorCheck;
	//IW7027 Error status , [BIT0] = Open , [BIT1] = Short , [BIT2] = DS Short
	uint8 fIwErrorType;
	//IW7027 Open/Short Status , max 128 channel , 16 byte = 128 bit.
	uint8 u8IwOpenShortStatus[16];
	//RESERVED , fix the size of the struct to 0x30 bytes for I2C slave access.
	uint8 RESERVED[0x05];
} Drv_Iw7027Param_t;

/***2.3 External Variables ***/
extern Drv_Iw7027Param_t tDrv_Iw7027Param;

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
extern uint8 Iw7027_updateDelayTable(enum Iw7027_Delay_e delay);

/**********************************************************
 * @Brief Iw7027_getErrorStatus
 * 		Check IW7027 Open/Short/Ds Short status from IW7027 registers.
 * @Param
 * 		iwparam		: target IW7027_WorkParam .
 * @Return
 * 		1 			: IW7027 has Open Short Error
 * 		0			: IW7027 ok , no error .
 **********************************************************/
extern uint8 Iw7027_checkOpenShortStatus(Drv_Iw7027Param_t *iwparam);

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
extern uint8 Iw7027_updateWorkParams(Drv_Iw7027Param_t *iwparam);

/**********************************************************
 * @Brief Iw7027_checkChipId
 * 		Check IW7027 Chip ID to get validation status of IC with timeout.
 * 		If can not read chip ID for 10 times retry , set as fail.
 * @Param
 * 		iw_all			: All IW7027 device that need check,usually put IW_ALL.
 * @Return
 * 		FLAG_SUCCESS 	: All IW7027 chip ID read OK.
 * 		FLAG_FAIL		: 1 or more IW7027 chip ID read fail.
 **********************************************************/
extern flag Iw7027_checkChipId(uint16 iw_all);

#endif /* DRV_IW7027_H_ */
