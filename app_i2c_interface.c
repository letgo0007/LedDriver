#include "app.h"
#include "driver.h"

uint8_t I2cSlave_handleMap(uint8_t *i2cmap)
{
	/*I2C Function 1 [SPI access function]
	#define I2C_SPIACCESS_WRMODE		(0x80)
	#define I2C_SPIACCESS_CHIPSEL		(0x81)
	#define I2C_SPIACCESS_REGADD		(0x82)
	#define I2C_SPIACCESS_LENGTH		(0x83)
	*/

	switch( i2cmap[I2C_SPIACCESS_WRMODE] )
	{
		case 0x80://Single Read
			i2cmap[ i2cmap[I2C_SPIACCESS_REGADD] ] = Iw7027_readSingleByte( i2cmap[I2C_SPIACCESS_CHIPSEL] , i2cmap[I2C_SPIACCESS_REGADD] );
			break;
		case 0x81://Single Write
			Iw7027_writeSingleByte(
					i2cmap[I2C_SPIACCESS_CHIPSEL] ,
					i2cmap[I2C_SPIACCESS_REGADD] ,
					i2cmap[ i2cmap[I2C_SPIACCESS_REGADD] ]
							);
			break;
		case 0x82://Multiple Read
			Iw7027_readMultiByte(
					i2cmap[I2C_SPIACCESS_CHIPSEL] ,
					i2cmap[I2C_SPIACCESS_REGADD] ,
					i2cmap[I2C_SPIACCESS_LENGTH] ,
					&i2cmap[i2cmap[I2C_SPIACCESS_REGADD]]
							 );
			break;
		case 0x83://Multiple Write
			Iw7027_writeMultiByte(
					i2cmap[I2C_SPIACCESS_CHIPSEL] ,
					i2cmap[I2C_SPIACCESS_REGADD] ,
					i2cmap[I2C_SPIACCESS_LENGTH] ,
					&i2cmap[i2cmap[I2C_SPIACCESS_REGADD]]
							 );
			break;
		default://No operation
			break;
	}

	i2cmap[I2C_SPIACCESS_WRMODE] = 0;


	//I2C Function 2 [Scheduler control]
	/* Value @ I2C_SPIACCESS_WRMODE
	 * [00] Read param from System scheduler (default)
	 * [80] Write param to System scheduler
	 *
	#define I2C_SCH_WRMODE				(0x90)
	#define I2C_SCH_RESET				(0x91)
	#define I2C_SCH_LDENABLE			(0x92)
	#define I2C_SCH_PEROID_CPUTICK_H	(0x93)
	#define I2C_SCH_PEROID_CPUTICK_L	(0x94)
	#define I2C_SCH_PEROID_GPIOCHECK_H	(0x95)
	#define I2C_SCH_PEROID_GPIOCHECK_L	(0x96)
	#define I2C_SCH_PEROID_MANUALMODE_H	(0x97)
	#define I2C_SCH_PEROID_MANUALMODE_L	(0x98)
	#define I2C_SCH_CPULOAD				(0x99)
*/

	switch( i2cmap[I2C_SCH_WRMODE] )
	{
		case 0x80://Write mode
			System_Schedule.schSystemOn 		= i2cmap[I2C_SCH_RESET];
			System_Schedule.schLocalDimmingOn 	= i2cmap[I2C_SCH_LDENABLE];
			System_Schedule.schCpuTickPeriod 	= 0x0100 * i2cmap[I2C_SCH_PEROID_CPUTICK_H] + i2cmap[I2C_SCH_PEROID_CPUTICK_L];
			System_Schedule.schBoardCheckPeriod = 0x0100 * i2cmap[I2C_SCH_PEROID_GPIOCHECK_H] + i2cmap[I2C_SCH_PEROID_GPIOCHECK_L];
			System_Schedule.schManualModePeriod = 0x0100 * i2cmap[I2C_SCH_PEROID_MANUALMODE_H] + i2cmap[I2C_SCH_PEROID_MANUALMODE_L];
			break;
		default://Read mode
			i2cmap[I2C_SCH_RESET] 				= System_Schedule.schSystemOn;
			i2cmap[I2C_SCH_LDENABLE] 			= System_Schedule.schLocalDimmingOn;
			i2cmap[I2C_SCH_PEROID_CPUTICK_H] 	= System_Schedule.schCpuTickPeriod >> 8 ;
			i2cmap[I2C_SCH_PEROID_CPUTICK_L] 	= System_Schedule.schCpuTickPeriod & 0xFF;
			i2cmap[I2C_SCH_PEROID_GPIOCHECK_H] 	= System_Schedule.schBoardCheckPeriod >> 8 ;
			i2cmap[I2C_SCH_PEROID_GPIOCHECK_L] 	= System_Schedule.schBoardCheckPeriod & 0xFF;
			i2cmap[I2C_SCH_PEROID_MANUALMODE_H] = System_Schedule.schManualModePeriod >> 8 ;
			i2cmap[I2C_SCH_PEROID_MANUALMODE_L] = System_Schedule.schManualModePeriod & 0xFF;
			i2cmap[I2C_SCH_CPULOAD] 			= System_Schedule.cpuLoad;
			break;
	}
	i2cmap[I2C_SCH_WRMODE] = 0;


	//I2C Function 3 [Backlight control]
	/* Value @ I2C_BL_WRMODE
	 * [00] Read param from System_Iw7027Param (default)
	 * [80] Write param to System_Iw7027Param
	 *
	#define I2C_BL_WRMODE				(0xA0)
	#define I2C_BL_IWFREQ				(0xA1)
	#define I2C_BL_IWCURRENT			(0xA2)
	#define I2C_BL_IWDELAYSEL			(0xA3)
	#define I2C_BL_VSYNCFREQ			(0xA4)
	#define I2C_BL_VSYNCDELAY			(0xA5)
	*/

	switch( i2cmap[I2C_BL_WRMODE] )
	{
		case 0x80://Write param
			System_Iw7027Param.iwFrequency 		= (enum Iw7027_Frequency)i2cmap[I2C_BL_IWFREQ];
			System_Iw7027Param.iwCurrent 		= (enum Iw7027_Current)i2cmap[I2C_BL_IWCURRENT];
			System_Iw7027Param.iwDelayTableSelet= (enum Iw7027_Delay)i2cmap[I2C_BL_IWDELAYSEL];
			System_Iw7027Param.iwVsyncFrequency = i2cmap[I2C_BL_VSYNCFREQ];
			System_Iw7027Param.iwVsyncDelay 	= i2cmap[I2C_BL_VSYNCDELAY];
			//Apply param
			Iw7027_updateWorkParams(&System_Iw7027Param);
			break;
		default ://Read param
			i2cmap[I2C_BL_IWFREQ] 				= System_Iw7027Param.iwFrequency ;
			i2cmap[I2C_BL_IWCURRENT] 			= System_Iw7027Param.iwCurrent;
			i2cmap[I2C_BL_IWDELAYSEL] 			= System_Iw7027Param.iwDelayTableSelet;
			i2cmap[I2C_BL_VSYNCFREQ] 			= System_Iw7027Param.iwVsyncFrequency;
			i2cmap[I2C_BL_VSYNCDELAY] 			= System_Iw7027Param.iwVsyncDelay;
			break;
	}
	i2cmap[I2C_BL_WRMODE] = 0;


	//I2C Function 4 [Error control]
	/* Value @ I2C_BL_WRMODE
	 * [00] Read param from System_ErrorParam (default)
	 * [80] Write param to System_ErrorParam
	 *
	#define I2C_ERROR_WRMODE			(0xB0)
	#define I2C_ERROR_ERRORTYPE			(0xB1)
	#define I2C_ERROR_ERRORCOUNT		(0xB2)
	#define I2C_ERROR_DC60VMAX			(0xB3)
	#define I2C_ERROR_DC60VMIN			(0xB4)
	#define I2C_ERROR_DC13VMAX			(0xB5)
	#define I2C_ERROR_DC13VMIN			(0xB6)
	#define I2C_ERROR_SPIRXFREQMIN		(0xB7)
	#define I2C_ERROR_SPIDATACHECKEN	(0xB8)
	#define I2C_ERROR_IW7027FAULTIN		(0xB9)
	#define I2C_ERROR_IW7027FAULTTYPE	(0xBA)
	#define I2C_ERROR_ERRORSAVEEN		(0xBB)
	*/
	switch( i2cmap[I2C_ERROR_WRMODE] )
	{
		case 0x80://Write param
			System_ErrorParam.eDc60vMax 		= i2cmap[I2C_ERROR_DC60VMAX];
			System_ErrorParam.eDc60vMin 		= i2cmap[I2C_ERROR_DC60VMIN];
			System_ErrorParam.eDc13vMax 		= i2cmap[I2C_ERROR_DC13VMAX];
			System_ErrorParam.eDc13vMin 		= i2cmap[I2C_ERROR_DC13VMIN];
			System_ErrorParam.eSpiRxFreqMin 	= i2cmap[I2C_ERROR_SPIRXFREQMIN];
			System_ErrorParam.eSpiDataErrorIgnore 	= i2cmap[I2C_ERROR_SPIDATACHECKEN];
			System_ErrorParam.eSpiRxFreqMin 	= i2cmap[I2C_ERROR_SPIRXFREQMIN];
			System_ErrorParam.eErrorSaveEn		= i2cmap[I2C_ERROR_ERRORSAVEEN];
			break;
		default ://Read param
			i2cmap[I2C_ERROR_ERRORTYPE] 		= System_ErrorParam.eErrorType ;
			i2cmap[I2C_ERROR_ERRORCOUNT] 		= System_ErrorParam.eCount;
			i2cmap[I2C_ERROR_DC60VMAX] 			= System_ErrorParam.eDc60vMax;
			i2cmap[I2C_ERROR_DC60VMIN] 			= System_ErrorParam.eDc60vMin ;
			i2cmap[I2C_ERROR_DC13VMAX] 			= System_ErrorParam.eDc13vMax ;
			i2cmap[I2C_ERROR_DC13VMIN] 			= System_ErrorParam.eDc13vMin ;
			i2cmap[I2C_ERROR_SPIRXFREQMIN] 		= System_ErrorParam.eSpiRxFreqMin ;
			i2cmap[I2C_ERROR_SPIDATACHECKEN] 	= System_ErrorParam.eSpiDataErrorIgnore ;
			i2cmap[I2C_ERROR_IW7027FAULTIGNORE] = System_ErrorParam.eIw7027FaultIgnore ;
			i2cmap[I2C_ERROR_IW7027FAULTTYPE] 	= System_ErrorParam.eIw7027ErrorType ;
			i2cmap[I2C_ERROR_ERRORSAVEEN] 		= System_ErrorParam.eErrorSaveEn ;
			break;
	}
	i2cmap[I2C_ERROR_WRMODE] = 0;


	//I2C Function 5 [Manual pattern]
	/* Value @ I2C_MANUAL_WRMODE
	 * [00] Read param from System_ManualDutyBuff (default)
	 * [80] Write param to System_ManualDutyBuff
	 *
	#define I2C_MANUAL_WRMODE			(0xC0)
	#define I2C_MANUAL_DUTY_H			(0xC1)
	#define I2C_MANUAL_DUTY_L			(0xC2)
	#define I2C_MANUAL_CHSTART			(0xC3)
	#define I2C_MANUAL_CHAMOUNT			(0xC4)
	*/
	switch( i2cmap[I2C_MANUAL_WRMODE] )
	{
		case 0x80://Write buffer
		{
			uint16_t i = 0;
			uint16_t duty = 0x0100 * i2cmap[I2C_MANUAL_DUTY_H] + i2cmap[I2C_MANUAL_DUTY_L] ;

			for( i = 0; i < i2cmap[ I2C_MANUAL_CHAMOUNT ]; i++ )
			{
				System_ManualDutyBuff[ i2cmap[ I2C_MANUAL_CHSTART ] + i ] = duty ;
			}

			break;
		}
		default ://Read buffer
			i2cmap[ I2C_MANUAL_DUTY_H ] = System_ManualDutyBuff[ i2cmap[ I2C_MANUAL_CHSTART ] ] >> 8;
			i2cmap[ I2C_MANUAL_DUTY_L ] = System_ManualDutyBuff[ i2cmap[ I2C_MANUAL_CHSTART ] ] & 0xFF;
			break;
	}
	i2cmap[I2C_MANUAL_WRMODE] = 0x00 ;


	//I2C Function 6 [DPL control]
	/* Value @ I2C_MANUAL_WRMODE
	 * [00] Read param from System_DplParam (default)
	 * [80] Write param to System_DplParam
	 * [81] Load Default Param Table 1 to System_DplParam;
	 * [82] Load Default Param Table 2 to System_DplParam;
	 * ...
	 * Const table amount refer to model introduction
	#define I2C_DPL_WRMODE				(0xD0)
	#define I2C_DPL_ENABLE				(0xD1)
	#define I2C_DPL_CH_AMOUNT			(0xD2)
	#define I2C_DPL_GAMMA_SEL			(0xD3)
	#define I2C_DPL_SAMPLE_FRAME		(0xD4)
	#define I2C_DPL_UPDATE_FRAME		(0xD5)
	#define I2C_DPL_LIMIT_UPSTEP_H		(0xD6)
	#define I2C_DPL_LIMIT_UPSTEP_L		(0xD7)
	#define I2C_DPL_LIMIT_DOWNSTEP_H	(0xD8)
	#define I2C_DPL_LIMIT_DOWNSTEP_L	(0xD9)
	#define I2C_DPL_GDMAX_H				(0xDA)
	#define I2C_DPL_GDMAX_L				(0xDB)
	#define I2C_DPL_LDMAX_H				(0xDC)
	#define I2C_DPL_LDMAX_L				(0xDD)
	#define I2C_DPL_HIGHTEMP_H			(0xDE)
	#define I2C_DPL_HIGHTEMP_L			(0xDF)
	#define I2C_DPL_LOWTEMP_H			(0xE0)
	#define I2C_DPL_LOWTEMP_L			(0xE1)
	*/

	switch (I2C_DPL_WRMODE)
	{
		case 0x80://Write param
			System_DplParam.dplOn 				= i2cmap[I2C_DPL_ENABLE] ;
			System_DplParam.dplChannelAmount	= i2cmap[I2C_DPL_CH_AMOUNT] ;
			System_DplParam.dplInputGammaSelect = i2cmap[I2C_DPL_GAMMA_SEL] ;
			System_DplParam.dplSampleFrames 	= i2cmap[I2C_DPL_SAMPLE_FRAME] ;
			System_DplParam.dplUpdateFrames 	= 0x0100 * i2cmap[I2C_DPL_UPDATE_FRAME_H] + i2cmap[I2C_DPL_UPDATE_FRAME_L] ;
			System_DplParam.dplLimitUpStep 		= 0x0100 * i2cmap[I2C_DPL_LIMIT_UPSTEP_H] + i2cmap[I2C_DPL_LIMIT_UPSTEP_L] ;
			System_DplParam.dplLimitDownStep	= 0x0100 * i2cmap[I2C_DPL_LIMIT_DOWNSTEP_H] + i2cmap[I2C_DPL_LIMIT_DOWNSTEP_L] ;
			System_DplParam.dplGdDutyMax 		= 0x0100 * i2cmap[I2C_DPL_GDMAX_H] + i2cmap[I2C_DPL_GDMAX_L] ;
			System_DplParam.dplLdDutyMax 		= 0x0100 * i2cmap[I2C_DPL_LDMAX_H] + i2cmap[I2C_DPL_LDMAX_L] ;
			break;
		case 0x81://Load default table 1
			System_DplParam = DPL_Param_70XU30A_80CH_320ma_12bit;
			break;
		case 0x82://Load default table 2
			System_DplParam = DPL_Param_70XU30A_80CH_250ma_12bit;
			break;
		default://Read param
			i2cmap[I2C_DPL_ENABLE] 				= System_DplParam.dplOn;
			i2cmap[I2C_DPL_CH_AMOUNT] 			= System_DplParam.dplChannelAmount;
			i2cmap[I2C_DPL_GAMMA_SEL] 			= System_DplParam.dplInputGammaSelect;
			i2cmap[I2C_DPL_SAMPLE_FRAME] 		= System_DplParam.dplSampleFrames;
			i2cmap[I2C_DPL_UPDATE_FRAME_H] 		= System_DplParam.dplUpdateFrames >> 8;
			i2cmap[I2C_DPL_UPDATE_FRAME_L] 		= System_DplParam.dplUpdateFrames & 0xFF;
			i2cmap[I2C_DPL_LIMIT_UPSTEP_H] 		= System_DplParam.dplLimitUpStep >> 8;
			i2cmap[I2C_DPL_LIMIT_UPSTEP_L] 		= System_DplParam.dplLimitUpStep & 0xFF;
			i2cmap[I2C_DPL_LIMIT_DOWNSTEP_H] 	= System_DplParam.dplLimitDownStep >> 8;
			i2cmap[I2C_DPL_LIMIT_DOWNSTEP_L] 	= System_DplParam.dplLimitDownStep & 0xFF;
			i2cmap[I2C_DPL_GDMAX_H] 			= System_DplParam.dplGdDutyMax >> 8;
			i2cmap[I2C_DPL_GDMAX_L] 			= System_DplParam.dplGdDutyMax & 0xFF;
			i2cmap[I2C_DPL_LDMAX_H] 			= System_DplParam.dplLdDutyMax >> 8;
			i2cmap[I2C_DPL_LDMAX_L] 			= System_DplParam.dplLdDutyMax & 0xFF;
			break;
	}
	i2cmap[I2C_DPL_WRMODE] = 0;

	return STATUS_SUCCESS;

}

uint8_t I2cSlave_initMap(uint8_t *i2cmap)
{
	I2cSlave_handleMap(i2cmap);
	return STATUS_SUCCESS;
}
