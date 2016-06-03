/******************************************************************************
 * @file 	[driver_iw7027.c]
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

/***1 Includes ****************************************************************/

#include "driver_iw7027.h"

#include "driverlib.h"

#include "driver_mcu.h"

/***2.1 Internal Marcos *******************************************************/

#ifndef UART_DEBUG_ON
#define UART_DEBUG_ON						(0)
#endif
//CS -> SPI delay(unit:us) , see IW7027 application note
#define IW_SPI_MASTER_TRANS_START_DELAY		(50)
//SPI -> CS delay(unit:us) , see IW7027 application note
#define IW_SPI_MASTER_TRANS_STOP_DELAY		(10)
//Current change max step .
#define	IW_CURRENT_CHANGE_STEP				(0x30)

//Select model specified const .
#define Iw7027_DefaultRegMap				(Iw7027_DefaultRegMap_70XU30A)
#define Iw7027_LedSortMap					(Iw7027_LedSortMap_70XU30A)

/***2.2 Internal Struct ******************************************************/

/***2.3 Internal Variables ***************************************************/

//Default Reg map for all IW7027 devices. 0x60 bytes for each device.
static const uint8 Iw7027_DefaultRegMap_70XU30A[IW_DEVICE_AMOUNT * 0x60] =
{
/*IW_0*/
/*0x0x*/0x06, 0x00, 0x00, 0x00, 0x27, 0x00, 0x4E, 0x00, 0x76, 0x00, 0x9D, 0x00, 0xC4, 0x00, 0xEC, 0x01,
/*0x1x*/0x13, 0x55, 0x3B, 0x62, 0x89, 0xB1, 0x40, 0xD8, 0x00, 0x27, 0x4E, 0x0F, 0x19, 0xD5, 0x0F, 0x18,
/*0x2x*/0x1E, 0x0F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x42, 0x42, 0x0F, 0x12, 0x7F, 0xFF, 0xFF, 0xE5, 0xBC,
/*0x3x*/0x7D, 0xD3, 0x01, 0x16, 0xC8, 0x80, 0x44, 0x00, 0xC0, 0xA0, 0x00, 0x78, 0x08, 0x28, 0x88, 0x88,
/*0x4x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,
/*0x5x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,

/*IW_1*/
/*0x0x*/0x06, 0x00, 0x76, 0x00, 0x9D, 0x00, 0xC4, 0x00, 0xEC, 0x01, 0x13, 0x01, 0x3B, 0x01, 0x62, 0x01,
/*0x1x*/0x89, 0x50, 0xB1, 0xD8, 0x00, 0x27, 0x00, 0x4E, 0x76, 0x9D, 0xC4, 0x33, 0xAA, 0x1E, 0x30, 0x3C,
/*0x2x*/0x67, 0x0F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x42, 0x42, 0x0F, 0x12, 0x7F, 0xFF, 0xFF, 0xE5, 0xBC,
/*0x3x*/0x7D, 0xD3, 0x01, 0x16, 0xC8, 0x80, 0x44, 0x00, 0xC0, 0xA0, 0x00, 0x78, 0x08, 0x28, 0x88, 0x88,
/*0x4x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,
/*0x5x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,

/*IW_2*/
/*0x0x*/0x06, 0x00, 0xEC, 0x01, 0x13, 0x01, 0x3B, 0x01, 0x62, 0x01, 0x89, 0x01, 0xB1, 0x01, 0xD8, 0x00,
/*0x1x*/0x00, 0x00, 0x27, 0x4E, 0x76, 0x9D, 0x05, 0xC4, 0xEC, 0x13, 0x3B, 0x54, 0x3C, 0x60, 0x78, 0xCE,
/*0x2x*/0xA8, 0x0F, 0xFF, 0x3F, 0xFF, 0x00, 0x00, 0x42, 0x42, 0x0F, 0x12, 0x7F, 0xFF, 0xFF, 0xE5, 0xBC,
/*0x3x*/0x7D, 0xD3, 0x01, 0x16, 0xC8, 0x80, 0x44, 0x00, 0xC0, 0xA0, 0x00, 0x78, 0x08, 0x28, 0x88, 0x88,
/*0x4x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,
/*0x5x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,

/*IW_3*/
/*0x0x*/0x06, 0x01, 0x62, 0x01, 0x89, 0x01, 0xB1, 0x01, 0xD8, 0x00, 0x00, 0x00, 0x27, 0x00, 0x4E, 0x00,
/*0x1x*/0x76, 0x01, 0x9D, 0xC4, 0xEC, 0x13, 0x55, 0x3B, 0x62, 0x89, 0xB1, 0x78, 0xC0, 0xF1, 0x9D, 0x50,
/*0x2x*/0xF1, 0x0F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x42, 0x42, 0x0F, 0x12, 0x7F, 0xFF, 0xFF, 0xE5, 0xBC,
/*0x3x*/0x7D, 0xD3, 0x01, 0x16, 0xC8, 0x80, 0x44, 0x00, 0xC0, 0xA0, 0x00, 0x78, 0x08, 0x28, 0x88, 0x88,
/*0x4x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,
/*0x5x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,

/*IW_4*/
/*0x0x*/0x06, 0x01, 0xD8, 0x00, 0x00, 0x00, 0x27, 0x00, 0x4E, 0x00, 0x76, 0x00, 0x9D, 0x00, 0xC4, 0x00,
/*0x1x*/0xEC, 0x55, 0x13, 0x3B, 0x62, 0x89, 0x50, 0xb1, 0xD8, 0x00, 0x00, 0x81, 0xE3, 0x3A, 0xA1, 0xE3,
/*0x2x*/0x00, 0x0F, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x42, 0x42, 0x0F, 0x12, 0x7F, 0xFF, 0xFF, 0xE5, 0xBC,
/*0x3x*/0x7D, 0xD3, 0x01, 0x16, 0xC8, 0x80, 0x44, 0x00, 0xC0, 0xA0, 0x00, 0x78, 0x08, 0x28, 0x88, 0x88,
/*0x4x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99,
/*0x5x*/0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, 0x01, 0x99, };

//Mapping const to convert LED channel mark form "TV Front View" to "Hardware View" .
static const uint8 Iw7027_LedSortMap_70XU30A[80] =
{
/*ROW0*/0x11, 0x10, 0x0F, 0x4D, 0x4E, 0x4F,
/*ROW1*/0x0E, 0x0D, 0x0C, 0x4A, 0x4B, 0x4C,
/*ROW2*/0x0B, 0x0A, 0x09, 0x47, 0x48, 0x49,
/*ROW3*/0x08, 0x07, 0x06, 0x44, 0x45, 0x46,
/*ROW4*/0x05, 0x04, 0x03, 0x41, 0x42, 0x43,
/*ROW5*/0x02, 0x01, 0x00, 0x3E, 0x3F, 0x40,
/*ROW6*/0x26, 0x25, 0x24, 0x3B, 0x3C, 0x3D,
/*ROW7*/0x23, 0x22, 0x21, 0x38, 0x39, 0x3A,
/*ROW8*/0x20, 0x1F, 0x1E, 0x35, 0x36, 0x37,
/*ROW9*/0x1D, 0x1C, 0x1B, 0x32, 0x33, 0x34,
/*ROWA*/0x1A, 0x19, 0x18, 0x2D, 0x30, 0x31,
/*ROWB*/0x17, 0x16, 0x15, 0x2A, 0x2B, 0x2C,
/*ROWC*/0x14, 0x13, 0x12, 0x27, 0x28, 0x29,
/*NC  */0x2E, 0x2F, };

static const dStruct_Iw7027Param_t Iw7027_DefaultParam =
{ .u16IwOutputFreq = 120, .u8IwCurrent = 0x42, .eIwDelayTableSelect = d2D, .u16IwInputFreq = 120, .u16IwOutputDelay = 1,
		.fIwRunErrorCheck = 1, .fIwIsError = 0 };

/***2.4 External Variables ***************************************************/

/***2.5 Internal Functions ***************************************************/

/***2.6 External Functions ***************************************************/

void Iw7027_writeMultiByte(uint8 chipsel, uint8 regaddress, uint8 length, uint8 *txdata)
{
	//Selest chip
	SpiMaster_setCsPin(chipsel);
	DELAY_US(IW_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte(0x01);
	SpiMaster_sendSingleByte(length);
	SpiMaster_sendSingleByte(regaddress);

	//Send multi data
	SpiMaster_sendMultiByte(txdata, length);

	//Unselest all chip
	DELAY_US(IW_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin(0x00);

}

void Iw7027_writeSingleByte(uint8 chipsel, uint8 regaddress, uint8 txdata)
{
	//Selest chip
	SpiMaster_setCsPin(chipsel);
	DELAY_US(IW_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte(0xC0);
	SpiMaster_sendSingleByte(regaddress);

	//Send Data
	SpiMaster_sendSingleByte(txdata);

	//Unselest all chip
	DELAY_US(IW_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin(0x00);
}

uint8 Iw7027_readSingleByte(uint8 chipsel, uint8 regaddress)
{
	uint8 readbyte;

	//Selest chip
	SpiMaster_setCsPin(chipsel);
	DELAY_US(IW_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte(0x41);
	SpiMaster_sendSingleByte(regaddress | 0x80);

	//2 dummy clocks to read byte
	SpiMaster_sendSingleByte(0x00);
	readbyte = SpiMaster_sendSingleByte(0x00);

	//Unselest all chip
	DELAY_US(IW_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin(0x00);

	return readbyte;
}

uint8 Iw7027_readMultiByte(uint8 chipsel, uint8 regaddress, uint8 length, uint8 *rxdata)
{
	uint8 i;

	//Selest chip
	SpiMaster_setCsPin(chipsel);
	DELAY_US(IW_SPI_MASTER_TRANS_START_DELAY);

	//Send Head & address
	SpiMaster_sendSingleByte(0x01);
	SpiMaster_sendSingleByte(length);
	SpiMaster_sendSingleByte(regaddress | 0x80);

	//set out dummy clocks to read byte
	SpiMaster_sendSingleByte(0x00);
	while (length--)
	{
		rxdata[i] = SpiMaster_sendSingleByte(0x00);
	}

	//Unselest all chip
	DELAY_US(IW_SPI_MASTER_TRANS_STOP_DELAY);
	SpiMaster_setCsPin(0x00);

	return rxdata[length - 1];
}

uint8 Iw7027_checkReadWithTimeout(uint8 chipsel, uint8 regaddress, uint8 checkvalue, uint8 bitmask)
{
	uint8 retrytime;
	uint8 val;
	uint8 chipmask;
	uint8 status;

	//Sequence check from IW_0 -> IW_N
	for (chipmask = IW_0; chipmask < IW_ALL; chipmask = chipmask << 1)
	{
		//Every selected chip check 10 times.
		//If any of the chip check timeout ,return fail.
		if (chipsel & chipmask)
		{
			retrytime = 10;
			status = 1;
			while (--retrytime && status)
			{
				val = Iw7027_readSingleByte((chipsel & chipmask), regaddress);

				if ((val & bitmask) == checkvalue)
				{
					//Correct , set status = 0 to stop loop
					status = 0;
				}
				else
				{
					//Wrong , delay 100us to retry.
					DELAY_US(100);
				}
			}
			if (retrytime == 0)
			{
				//if time out , return fail
				return FLAG_FAIL;
			}
		}
	}

	//Check loop finish ,return success .
	return FLAG_SUCCESS;

}

uint8 Iw7027_init(void)
{
	uint8 status = 0;
	uint8 i = 0;

#if UART_DEBUG_ON
	PrintTime(&System_Time);
	PrintString("[IW7027 INTIAL] Start...\r\n");
	PrintTime(&System_Time);
	PrintString("[IW7027 INTIAL] -1 : Power On & Delay.\r\n");
#endif
	//Step 1: Power ON .
	//YZF 2016/4/30 : Force power off for 500ms to ensure IW7027 is powerd off AC DIP conditon.
	HW_SET_IW7027_POWER_OFF;
	DELAY_MS(500);
	HW_SET_IW7027_POWER_ON;
	DELAY_MS(200);

	//Step 2: check chip ID to ensure IW7027 is working .
	do
	{
#if UART_DEBUG_ON
		PrintTime(&System_Time);
		PrintString("[IW7027 INTIAL] -2 : Read Chip ID\r\n");
#endif
		status = Iw7027_checkReadWithTimeout(IW_ALL, 0x6B, 0x24, 0xFF);
	} while (status == 0);

	//Step 3 : Write Initial setting in sequence  from chip IW0 to IW_N
	for (i = 0; i < IW_DEVICE_AMOUNT; i++)
	{
		Iw7027_writeMultiByte(IW_0 << i, 0x00, 0x60, (uint8 *) &Iw7027_DefaultRegMap[0x60 * i]);
#if UART_DEBUG_ON
		PrintTime(&System_Time);
		PrintString("[IW7027 INTIAL] -3 : Write Initial Reg Map\r\n");
#endif
	}

	//Step 4 :wait STB
	do
	{
#if UART_DEBUG_ON
		PrintTime(&System_Time);
		PrintString("[IW7027 INTIAL] -4 : Wait STB ...\r\n");
#endif
		DELAY_MS(200);
	} while ( HW_GET_STB_IN == 0);

	//Step 5 : Set IW7027 status using default param.
	SysParam_Iw7027 = Iw7027_DefaultParam;
	Iw7027_updateWorkParams(&SysParam_Iw7027);
	DELAY_MS(200);		//wait 200ms for pwm stable.
#if UART_DEBUG_ON
			PrintTime(&System_Time);
			PrintString("[IW7027 INTIAL] -5 : Get STB , Set work status ...\r\n");
			PrintString("Initial IW7027 work status:\r\n");
			PrintArray((uint8 *)&SysParam_Iw7027,sizeof(SysParam_Iw7027));
			PrintEnter();
#endif

	//Step 6 : Initialize Done ,turn on BL
	Iw7027_writeSingleByte(IW_ALL, 0x00, 0x07);
#if UART_DEBUG_ON
	PrintTime(&System_Time);
	PrintString("[IW7027 INTIAL] -6 : Initialize finish , turn on BL..\r\n");
	PrintString("\e[32mBL on ,enter main loop.\r\n\e[30m");
#endif

	//Return status
	return status;
}

uint8 Iw7027_updateDuty(uint16 *dutymatrix)
{
	uint8 i;

	static uint8 Iw7027_SortBuff[IW_DEVICE_AMOUNT * 32];

	//Sort duty matrix by LED_sort_map
	for (i = 0; i < IW_LED_CHANNEL; i++)
	{
		//convert 1 16bit data -> 2 8bit data . with resorted in LED hardware order
		Iw7027_SortBuff[2 * Iw7027_LedSortMap[i]] = dutymatrix[i] >> 8;
		Iw7027_SortBuff[2 * Iw7027_LedSortMap[i] + 1] = dutymatrix[i] & 0xFF;
	}

	//Sequence write chip IW0 ~ IW_N
	for (i = 0; i < IW_DEVICE_AMOUNT; i++)
	{
		Iw7027_writeMultiByte( IW_0 << i, 0x40, 32, Iw7027_SortBuff + 32 * i);
	}

	return FLAG_SUCCESS;
}

uint8 Iw7027_updateCurrent(uint8 current)
{
	uint8 status;

	//Write data to IW7027
	//1 . Disable Protect , Set [FAUL_LOCK] (0x62  BIT0)to 1
	Iw7027_writeSingleByte( IW_ALL, 0x62, 0x01);

	//2 . Write current to 0x27
	Iw7027_writeSingleByte( IW_ALL, 0x27, current);

	//3 . Check status. Low 4 bit of 0xB3 = 0x05
	status = Iw7027_checkReadWithTimeout( IW_ALL, 0xB3, 0x05, 0x0F);

	//4 . Enable Protect , Set [FAULT LOCK] (0x62  BIT0)to 0 ,IDAC_REMAP + FAUL_LOCK
	Iw7027_writeSingleByte( IW_ALL, 0x62, 0x00);

	return status;
}

uint8 Iw7027_updateFrequency(uint8 freq)
{
	uint8 pllval;

	//Determine current register value accroding to Iw7027 datasheet
	switch (freq)
	{
	case 50:
		pllval = 48;
		break;
	case 60:
		pllval = 40;
		break;
	case 100:
		pllval = 24;
		break;
	case 120:
		pllval = 20;
		break;
	default:
		return FLAG_FAIL;
	}

	//1 .	Set [PLL_OUTDIV] (0x31 BIT0~BIT4) , Set [PLL_EN] (0x31 BIT7) = 1
	Iw7027_writeSingleByte( IW_ALL, 0x31, pllval + BIT7);

	//2.	Set [PWM_PER_SEL] (0x2F BIT7) =1,Load PWM period from register 0x21[4:0] and 0x22[7:0]
	Iw7027_writeSingleByte( IW_ALL, 0x2F, BIT7);

	//3.	Set [PWM_PER] (0x21 BIT0~BIT4 , 0x22 BIT0~BIT7 ) to 4095 (0x0fff)
	Iw7027_writeSingleByte( IW_ALL, 0x21, 0x0F);
	Iw7027_writeSingleByte( IW_ALL, 0x22, 0xFF);

	//5.	Check [RO_PLL_LOCK]	(0x84 BIT4)	to ensuer PLL work well
	if (Iw7027_checkReadWithTimeout( IW_ALL, 0x84, BIT4, BIT4))
	{
		//return pll value when success
		return FLAG_SUCCESS;
	}
	else
	{
		return FLAG_FAIL;
	}
}

uint8 Iw7027_updateDelayTable(enum Iw7027_Delay_e delay)
{
	return FLAG_SUCCESS;
}

uint8 Iw7027_updateWorkParams(dStruct_Iw7027Param_t *param_in)
{
	static dStruct_Iw7027Param_t param_now;

	//Update IW7027_PLL & Vsync_Out when changed.
	if (param_now.u16IwOutputFreq != param_in->u16IwOutputFreq)
	{
		param_now.u16IwOutputFreq = param_in->u16IwOutputFreq;
		Iw7027_updateFrequency(param_now.u16IwOutputFreq);
	}

	if ((param_now.u16IwInputFreq != param_in->u16IwInputFreq) || (param_now.u16IwOutputDelay != param_in->u16IwOutputDelay))
	{
		param_now.u16IwInputFreq = param_in->u16IwInputFreq;
		param_now.u16IwOutputDelay = param_in->u16IwOutputDelay;
		PwmOut_init(param_now.u16IwInputFreq, param_now.u16IwOutputDelay);
	}

	//Update current when changed.
	//YZF 20150526: set current change step
	if (param_now.u8IwCurrent != param_in->u8IwCurrent)
	{
		while (param_now.u8IwCurrent != param_in->u8IwCurrent)
		{
			param_now.u8IwCurrent = min((uint16) param_in->u8IwCurrent, (uint16) param_now.u8IwCurrent + IW_CURRENT_CHANGE_STEP);
			Iw7027_updateCurrent(param_now.u8IwCurrent);
		}

	}

	//Update delay table when changed.
	if (param_now.eIwDelayTableSelect != param_in->eIwDelayTableSelect)
	{
		param_now.eIwDelayTableSelect = param_in->eIwDelayTableSelect;
		Iw7027_updateDelayTable(param_now.eIwDelayTableSelect);
	}

	if (param_in->fIwRunErrorCheck)
	{
		Iw7027_checkOpenShorStatus(param_in);
		param_in->fIwRunErrorCheck = 0;
	}

	return FLAG_SUCCESS;

}

uint8 Iw7027_checkOpenShorStatus(dStruct_Iw7027Param_t *iwparam)
{
	//Reset Is error
	iwparam->fIwIsError = 0;
	uint8 i;

	//Enable Error Read
	Iw7027_writeSingleByte( IW_ALL, 0x78, 0x80);

	//Read Open/Short/DSShort status from 0x85~0x8A
	for (i = 0; i < IW_DEVICE_AMOUNT; i++)
	{
		Iw7027_readMultiByte(IW_0 << i, 0x85, 6, iwparam->u8IwOpenShortStatus + i * 6);
	}

	//Disable Error¡¡Read
	Iw7027_writeSingleByte( IW_ALL, 0x78, 0x00);

	for (i = 0; i < IW_DEVICE_AMOUNT * 6; i++)
	{
		if (iwparam->u8IwOpenShortStatus[i])
		{
			iwparam->fIwIsError |= 1;
		}
	}

	//Rrturn error status.
	return iwparam->fIwIsError;
}
