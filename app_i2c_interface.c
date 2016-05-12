#include "app.h"
#include "driver.h"



uint8_t I2cSlave_handleMap(uint8_t *i2cmap)
{
	/* 0x00~0x7F 	: iw7027 direct access map buffer
	 * 0x80~0x83 	: Iw7027 Direct Accesss
	 * 				0x80	[W/R MODE]
	 * 				0x81	[Chip Sel]
	 * 				0x82	[Start Address]
	 * 				0x83	[Length]
	 *
	 * 0x90			: System Scheduler Control
	 * 				0x90	[Scheduler control enable]
	 * 				0x91	[System_On]
	 * 				0x92	[LocalDimming_On]
	 * 				0x93	[CpuTickPeriod]
	 * 				0x94	[GpioCheckPeriod]
	 * 				0x95	[ErrorHandlePeriod]
	 * 				0x96	[CPU_Load]
	 *
	 * 0xA0			: Backlight Control
	 * 				0xA0	[BL control Enable]
	 * 				0xA1 	[Freq]
	 * 				0xA2	[Current]
	 * 				0xA3	[DelayTableSel]
	 * 				0xA4	[Vsync_F]
	 * 				0xA5	[Vsync_Delay]
	 *
	 * 0xB0			: BoardInfo
	 * 0xC0			: Manual Pattern Control
	 * 				0xC0	[SEL MODE]
	 * 				0xC1	[DutyHighByte]
	 * 				0xC2	[DutyHighLow]
	 * 				0xC3	[LED No.]
	 * 				0xC4	[Amout to write]
	 *
	 * 0xD0			: DPL control
	 * 				0xD0	[ENABLE + Table select]
	 * 				0xD1	[DplON]
	 * 				0xD2	[DplChannelAmount]
	 * 				0xD3	[DplSampleAmount]
	 * 				0xD4	[DplLimitStepUp_H]
	 * 				0xD5	[DplLimitStepUp_L]
	 * 				0xD6 	[DplLimitStepDown_H]
	 * 				0xD7 	[DplLimitStepDown_L]
	 * 				0xD8	[DplGdDutyMax_H]
	 * 				0xD9	[DplGdDutyMax_L]
	 * 				0xDA	[DplLdDutyMax_H]
	 * 				0xDB	[DplLdDutyMax_L]
	 * 				0xDC	[DplHighTempLimit_H]
	 * 				0xDD	[DplHighTempLimit_L]
	 * 				0xDE	[DplLowTempLimit_H]
	 * 				0xDF	[DplLowTempLimit_L]
	 *
	 * 0xE0			: .
	 * 0xF0			: Hardware Version
	 */


	 /* 0x80~0x83 	: Iw7027 Direct Accesss
	 * 				0x80	[W/R MODE] =
	 * 						0x00:	No operation
	 * 						0x80:	Read single
	 * 						0x81:	Write single
	 * 						0x82:	Read Multi
	 * 						0x83:	Write multi
	 * 				0x81	[Chip Sel]
	 * 				0x82	[Start Address]
	 * 				0x83	[Length]
	 */

	if( i2cmap[0x80] & BIT7 ) //active
	{
		if( i2cmap[0x80] & BIT0 ) //Write
		{
			if( i2cmap[0x80] & BIT1 ) //Multiple write
			{
				Iw7027_writeMultiByte( i2cmap[0x81] , i2cmap[0x82] , i2cmap[0x83] , &i2cmap[i2cmap[0x82]]);
			}
			else	//Single write
			{
				Iw7027_writeSingleByte( i2cmap[0x81] , i2cmap[0x82] , i2cmap[i2cmap[0x82]]);
			}
		}
		else	//Read
		{
			if( i2cmap[0x80] & BIT1 ) //Multiple Read
			{
				uint8_t i;
				for(i = 0 ; i< i2cmap[0x83] ; i++)
				{
					i2cmap[ i2cmap[0x82] + i ] = Iw7027_readSingleByte( i2cmap[0x81] , i2cmap[0x82] + i);
				}
			}
			else//Single Read
			{
				i2cmap[i2cmap[0x82]] = Iw7027_readSingleByte( i2cmap[0x81] , i2cmap[0x82] );
			}
		}

		//Reset mark, run only once.
		i2cmap[0x80] = 0x00 ;
	}

	 /* 0x90		: System Scheduler Control
	 * 				0x90	[ENABLE]
	 * 				0x91	[System_On]
	 * 				0x92	[LocalDimming_On]
	 * 				0x93	[CpuTickPeriod_H]
	 * 				0x94	[CpuTickPeriod_L]
	 * 				0x95	[GpioCheckPeriod_H]
	 * 				0x96	[GpioCheckPeriod_L]
	 * 				0x97	[ManualModePeriod_H]
	 * 				0x98	[ManualModePeriod_L]
	 * 				0x99	[CPU_Load]
	 */
	if( i2cmap[0x90] & BIT7 ) //active
	{

		System_Schedule.schSystemOn = i2cmap[0x91];
		System_Schedule.schLocalDimmingOn 	= i2cmap[0x92];
		System_Schedule.schCpuTickPeriod = 0x0100 * i2cmap[0x93] + i2cmap[0x94];
		System_Schedule.schBoardCheckPeriod = 0x0100 * i2cmap[0x95] + i2cmap[0x96];
		System_Schedule.schManualModePeriod = 0x0100 * i2cmap[0x97] + i2cmap[0x98];
		i2cmap[0x99] = System_Schedule.cpuLoad;

		//Reset mark, run only once.
		i2cmap[0x90] = 0x00 ;
	}

	 /* 0xA0: Backlight Control
	 * 				0xA0	[ENABLE]
	 * 				0xA1 	[IW_Freq]
	 * 				0xA2	[IW_Current]
	 * 				0xA3	[IW_DelayTableSel]
	 * 				0xA4	[Vsync_F]
	 * 				0xA5	[Vsync_Delay]
	 */
	if( i2cmap[0xA0] & BIT7 ) //active
	{
		System_Iw7027Param.iwFrequency = (enum Iw7027_Frequency)i2cmap[0xA1];
		System_Iw7027Param.iwCurrent = (enum Iw7027_Current)i2cmap[0xA2];
		System_Iw7027Param.iwDelayTableSelet = (enum Iw7027_Delay)i2cmap[0xA3];
		System_Iw7027Param.iwVsyncFrequency = i2cmap[0xA4];
		System_Iw7027Param.iwVsyncDelay = i2cmap[0xA5];

		Iw7027_updateWorkParams(&System_Iw7027Param);
		//Reset mark, run only once.
		i2cmap[0xA0] = 0x00 ;
	}


	/* Board Info
	 *
	 *
	 */
	if( i2cmap[0xB0] & BIT7 ) //active
	{
		i2cmap[0xB1] = System_BoardInfo.boardD60V;
		i2cmap[0xB2] = System_BoardInfo.boardD13V;
		i2cmap[0xB3] = System_BoardInfo.boardTemprature;
		i2cmap[0xB4] = System_BoardInfo.boardStb;
		i2cmap[0xB5] = System_BoardInfo.boardIw7027Falut;
		i2cmap[0xB6] = System_BoardInfo.boardSpiRxFreq;
		i2cmap[0xB7] = System_BoardInfo.boardSpiRxValid;

		//Reset mark, run only once.
		i2cmap[0xB0] = 0x00 ;
	}

	 /* 0xC0	: Manual Pattern
	 * 				0xC0	[SEL MODE]
	 * 				0xC1	[DutyHighByte]
	 * 				0xC2	[DutyHighLow]
	 * 				0xC3	[DutyStartAddress]
	 * 				0xC4	[DutyAmount]
	 */
	if( i2cmap[0xC0] & BIT7 ) //active
	{
		uint16_t duty;
		uint8_t	i;
		duty = 0x0100 * i2cmap[0xC1] + i2cmap[0xC2] ;

		for( i = 0; i < i2cmap[ 0xC4 ]; i++ )
		{
			System_ManualDutyBuff[ i2cmap[ 0xC3 ] + i ] = duty ;
		}

		//Reset mark, run only once.
		i2cmap[0xC0] = 0x00 ;
	}

	 /* 0xD0	: DPL control
	 * 				0xD0	[ENABLE + Table select]
	 * 				0xD1	[DplON]
	 * 				0xD2	[DplChannelAmount]
	 * 				0xD3	[DplSampleAmount]
	 * 				0xD4	[DplLimitStepUp_H]
	 * 				0xD5	[DplLimitStepUp_L]
	 * 				0xD6 	[DplLimitStepDown_H]
	 * 				0xD7 	[DplLimitStepDown_L]
	 * 				0xD8	[DplGdDutyMax_H]
	 * 				0xD9	[DplGdDutyMax_L]
	 * 				0xDA	[DplLdDutyMax_H]
	 * 				0xDB	[DplLdDutyMax_L]
	 * 				0xDC	[DplHighTempLimit_H]
	 * 				0xDD	[DplHighTempLimit_L]
	 * 				0xDE	[DplLowTempLimit_H]
	 * 				0xDF	[DplLowTempLimit_L]
	 */
	if( i2cmap[0xD0] & BIT7 ) //active
	{
		switch( i2cmap[0xD0]&0x0F )
		{
		case 0x0://Table 1
			System_DplParam = DPL_Param_70XU30A_80CH_320ma_12bit;
			break;
		case 0x1://Table 2
			System_DplParam = DPL_Param_70XU30A_80CH_250ma_12bit;
			break;
		case 0xF://Manual table

			System_DplParam.dplOn 				= i2cmap[0xD1] ;
			System_DplParam.dplChannelAmount	= i2cmap[0xD2] ;
			System_DplParam.dplSampleAmoutToRunParamUpdate 	= i2cmap[0xD3] ;
			System_DplParam.dplLimitUpStep 		= 0x0100 * i2cmap[0xD4] + 0x0100 * i2cmap[0xD5] ;
			System_DplParam.dplLimitDownStep	= 0x0100 * i2cmap[0xD6] + 0x0100 * i2cmap[0xD7] ;
			System_DplParam.dplGdDutyMax 		= 0x0100 * i2cmap[0xD8] + 0x0100 * i2cmap[0xD9] ;
			System_DplParam.dplLdDutyMax 		= 0x0100 * i2cmap[0xDA] + 0x0100 * i2cmap[0xDB] ;

			uint16_t temp;
			uint8_t	i;
			temp = 0x0100 * i2cmap[0xDC] + 0x0100 * i2cmap[0xDD] ;
			for(i = 0; i< System_DplParam.dplChannelAmount ; i++ )
			{
				System_DplParam.dplLdDutySumLimitHighTempTable[i] 	= temp ;
			}

			temp = 0x0100 * i2cmap[0xDE] + 0x0100 * i2cmap[0xDF] ;
			for(i = 0; i< System_DplParam.dplChannelAmount ; i++ )
			{
				System_DplParam.dplLdDutySumLimitHighTempTable[i] 	= temp ;
			}


		default:
			break;
		}
	}

	return STATUS_SUCCESS;

}

uint8_t I2cSlave_initMap(uint8_t *i2cmap)
{
	 /* 0x90		: System Scheduler Control
	 * 				0x90	[ENABLE]
	 * 				0x91	[System_On]
	 * 				0x92	[LocalDimming_On]
	 * 				0x93	[CpuTickPeriod_H]
	 * 				0x94	[CpuTickPeriod_L]
	 * 				0x95	[GpioCheckPeriod_H]
	 * 				0x96	[GpioCheckPeriod_L]
	 * 				0x97	[ManualModePeriod_H]
	 * 				0x98	[ManualModePeriod_L]
	 * 				0x99	[CPU_Load]
	 */
	i2cmap[0x90] = 0x80;
	i2cmap[0x91] = 1;
	i2cmap[0x92] = 1;
	i2cmap[0x93] = (33)>>8;		//1000Hz for Cpu wake up
	i2cmap[0x94] = (33)&0xFF;
	i2cmap[0x95] = (328)>>8;	//100Hz for Gpio check
	i2cmap[0x96] = (328)&0xFF;
	i2cmap[0x97] = (273)>>8;	//60Hz for Manual mode
	i2cmap[0x98] = (273)&0xFF;

	 /* 0xA0: Backlight Control
	 * 				0xA0	[ENABLE]
	 * 				0xA1 	[Freq]
	 * 				0xA2	[Current]
	 * 				0xA3	[DelayTableSel]
	 * 				0xA4	[Vsync_F]
	 * 				0xA5	[Vsync_Delay]
	 */
	i2cmap[0xA0] = 0x80;
	i2cmap[0xA1] = 3;	//120Hz
	i2cmap[0xA2] = 1;	//200mA
	i2cmap[0xA3] = 0;
	i2cmap[0xA4] = 120;
	i2cmap[0xA5] = 1;

	 /* 0xC0	: Manual Pattern
	 * 				0xC0	[SEL MODE]
	 * 				0xC1	[DutyHighByte]
	 * 				0xC2	[DutyHighLow]
	 * 				0xC3	[DutyStartAddress]
	 * 				0xC4	[DutyAmount]
	 */
	i2cmap[0xC0] = 0x80; //default = black screen
	i2cmap[0xC1] = 0;
	i2cmap[0xC2] = 0;
	i2cmap[0xC3] = 0;
	i2cmap[0xC4] = 80;

	 /* 0xD0	: DPL control
	 * 				0xD0	[ENABLE + Table select]
	 * 				0xD1	[DplON]
	 * 				0xD2	[DplChannelAmount]
	 * 				0xD3	[DplSampleAmount]
	 * 				0xD4	[DplLimitStepUp_H]
	 * 				0xD5	[DplLimitStepUp_L]
	 * 				0xD6 	[DplLimitStepDown_H]
	 * 				0xD7 	[DplLimitStepDown_L]
	 * 				0xD8	[DplGdDutyMax_H]
	 * 				0xD9	[DplGdDutyMax_L]
	 * 				0xDA	[DplLdDutyMax_H]
	 * 				0xDB	[DplLdDutyMax_L]
	 * 				0xDC	[DplHighTempLimit_H]
	 * 				0xDD	[DplHighTempLimit_L]
	 * 				0xDE	[DplLowTempLimit_H]
	 * 				0xDF	[DplLowTempLimit_L]
	 */
	i2cmap[0xD0] = 0x80;


	i2cmap[0xF0] = (uint8_t)(SOFTWARE_VERSION>>24);
	i2cmap[0xF1] = (uint8_t)(SOFTWARE_VERSION>>16);
	i2cmap[0xF2] = (uint8_t)(SOFTWARE_VERSION>>8);
	i2cmap[0xF3] = (uint8_t)SOFTWARE_VERSION;
	i2cmap[0xF4] = (uint8_t)(HARDWARE_VERSION>>24);
	i2cmap[0xF5] = (uint8_t)(HARDWARE_VERSION>>16);
	i2cmap[0xF6] = (uint8_t)(HARDWARE_VERSION>>8);
	i2cmap[0xF7] = (uint8_t)HARDWARE_VERSION;


	I2cSlave_handleMap(i2cmap);
	return STATUS_SUCCESS;
}
