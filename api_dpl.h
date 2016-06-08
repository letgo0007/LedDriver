/******************************************************************************
 * @file 	[app_dpl.h]
 *
 * [Dynamic Power Limit] function for local dimming led driver.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/

#ifndef API_DPL_H_
#define API_DPL_H_
/***1 Includes ***************************************************************/

#include "std.h"

/***2.1 External Macros ******************************************************/

//Software Version
#define DPL_VERSION					(0x01)
//Maximum Size for LED data buffer size.
#define DPL_LED_CH_MAX				(128)

/***2.2 External Structures **************************************************/

//Parameters Struct
typedef struct Dpl_Prama_t
{
	//On/Off control of DPL
	flag dplOn;
	//Duty channel amount
	uint8 dplChannelAmount;
	//Enable input gamma correction
	uint8 dplInputGammaEnable;
	//Frame amount to run dpl sample.
	uint8 dplSampleFrames;
	//Frame amount to run dpl param update.
	uint16 dplUpdateFrames;
	//Limit adjustment rise up step
	uint16 dplLimitUpStep;
	//Limit adjustment fall down step
	uint16 dplLimitDownStep;
	//Global average duty limit
	uint16 dplGdDutyMax;
	//Local duty limit
	uint16 dplLdDutyMax;
	//Limit Temperature calibration value;
	int16 dplTemperatureCalibration;
	//Sum duty limit for 1min
	uint16 dplLdDutySumLimitHighTemp;
	//Temprature Safe duty for every local
	uint16 dplLdDutySumLimitLowTemp;
	//Input Gamma point @ 0x0000
	uint16 dplInputGammaGp0x000;
	//Input Gamma point @ 0x03F0
	uint16 dplInputGammaGp0x3F0;
	//Input Gamma point @ 0x07F0
	uint16 dplInputGammaGp0x7F0;
	//Input Gamma point @ 0x0BF0
	uint16 dplInputGammaGp0xBF0;
	//Input Gamma point @ 0x0FF0
	uint16 dplInputGammaGp0xFF0;
	//RESERVED...
	uint8 reserved[0x11];
} Dpl_Prama_t;

/***2.3 External Variables ***************************************************/

//Temp buffers (External access for Debug use only).
extern uint16 u16Dpl_tempSampleCount;
extern uint16 u16Dpl_tempDutyMatrix[DPL_LED_CH_MAX];
extern uint32 u32Dpl_tempSumDutyMatrix[DPL_LED_CH_MAX];
extern uint16 u16Dpl_tempDutyLimitTable[DPL_LED_CH_MAX];
extern uint16 u16Dpl_InputGamma[0x100];
//Dpl param
extern Dpl_Prama_t tDpl_Param;

/***2.4 External Functions ***************************************************/
//DMA accelerated memory copy.
extern void Hal_Mem_copy();
//DMA accelerated memory set.
extern void Hal_Mem_set16();
//DPL main struct
extern uint8 DPL_Function(uint16 *inputduty, uint16 *outputduty, Dpl_Prama_t *dplparam);
extern void DPL_caliberateTemp(int8 temp, Dpl_Prama_t *dplparam);

#endif /* API_DPL_H_ */

