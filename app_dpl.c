/******************************************************************************
 * @file 	[app_dpl.c]
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

/***1 Includes ***************************************************************/

#include "app_dpl.h"

/***2.1 Internal Marcos ******************************************************/

#define DPL_DefaultParam			(DPL_DefaultParam_70XU30A)
#define DPL_TempCalibratineTable	(DPL_TempCalibratineTable_70XU30A)
#define DPL_ZoneOffsetTable			(DPL_ZoneOffsetTable_70XU30A)

/***2.2 Internal Struct ******************************************************/

/***2.3 Internal Variables ***************************************************/
//Buffers
uint16 DPL_tempSampleCount =
{ 0 };
uint16 DPL_tempDutyMatrix[DPL_LED_CH_MAX] =
{ 0 };
uint32 DPL_tempSumDutyMatrix[DPL_LED_CH_MAX] =
{ 0 };
uint16 DPL_tempDutyLimitTable[DPL_LED_CH_MAX] =
{ 0 };
uint16 DPL_InputGamma[0x100] =
{ 0 };

//Default params.
static const DPL_Prama DPL_DefaultParam_70XU30A =
{ .dplOn = 1, .dplChannelAmount = 78, .dplInputGammaEnable = 1, .dplSampleFrames = 120, .dplUpdateFrames = 1200,
		.dplLimitUpStep = 0x0080, .dplLimitDownStep = 0x0080, .dplGdDutyMax = 0x0800, .dplLdDutyMax = 0x1500,
		.dplTemperatureCalibration = 0x0000, .dplLdDutySumLimitHighTemp = 0x0800, .dplLdDutySumLimitLowTemp = 0x0700,
		.dplInputGammaGp0x000 = 0x0000, .dplInputGammaGp0x3F0 = 0x03F0, .dplInputGammaGp0x7F0 = 0x07F0, .dplInputGammaGp0xBF0 =
				0x0BF0, .dplInputGammaGp0xFF0 = 0x0FF0, };

//To balance different enviroment temperature.
static const int16 DPL_TempCalibratineTable_70XU30A[60] =
{ 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, };

//To balance LED environment temperature on different area of a LED Panel.
static const uint16 DPL_ZoneOffsetTable_70XU30A[DPL_LED_CH_MAX] =
{
/*ROW0*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW1*/0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000,
/*ROW2*/0x0000, 0x0080, 0x0080, 0x0080, 0x0000, 0x0000,
/*ROW3*/0x0000, 0x0080, 0x0000, 0x0040, 0x0080, 0x0000,
/*ROW4*/0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000,
/*ROW5*/0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000,
/*ROW6*/0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000,
/*ROW7*/0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000,
/*ROW8*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROW9*/0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000,
/*ROWA*/0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000,
/*ROWB*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*ROWC*/0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/*NC  */0x0000, 0x0000 };

/***2.4 External Variables ***************************************************/

/***2.5 Internal Functions ***************************************************/

void DPL_limitLocalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	static uint16 ldmax_now;
	uint16 i = 0;

	//If LD duty  modified , reset all duty limit to new max value.
	if (ldmax_now != dplparam->dplLdDutyMax)
	{
		ldmax_now = dplparam->dplLdDutyMax;
		Mem_set16((uint32) &DPL_tempDutyLimitTable, dplparam->dplLdDutyMax, sizeof(DPL_tempDutyLimitTable) / 2);
	}

	//Limit every local duty <= dplLdLimitTable
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		outputduty[i] = min(inputduty[i], DPL_tempDutyLimitTable[i]);
	}
}

void DPL_limitGlobalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	uint16 i = 0;
	uint32 sum = 0;
	uint16 avg = 0;
	uint32 gain = 0;

	//Calculate average duty
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		sum = sum + inputduty[i];
	}
	avg = sum / dplparam->dplChannelAmount;

	//If Over Limit, reduce duty by gain.
	if (avg > dplparam->dplGdDutyMax)
	{
		//gain = 1 ~ 255
		gain = (uint32) (dplparam->dplGdDutyMax) * 256 / avg;

		//output = input * gian / 256
		for (i = 0; i < dplparam->dplChannelAmount; i++)
		{
			outputduty[i] = (gain * inputduty[i]) >> 8;
		}
	}
	//If Within Limit , Bypass input -> output .
	else
	{
		//Size = channel * 2 bytes (16bit)
		Mem_copy((uint32) &*outputduty, (uint32) &*inputduty, (dplparam->dplChannelAmount) * 2);
	}

}

void DPL_sumLocalDuty(uint16 *inputduty, uint32 *outputdutysum, DPL_Prama *dplparam)
{
	uint16 i = 0;
	//Sum up duty of every local
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		outputdutysum[i] += inputduty[i];
	}

}

void DPL_updateParam(uint32 *inputdutysum, DPL_Prama *dplparam)
{
	/*Update local duty limit table arroding to Local Duty Sum in the last 1min
	 * [A] DutySum > HighTemp 	, Limit = - step, no lower than High temp limit
	 * [C] DutySum < LowTemp	, Limit = + step, no higher than local limit max
	 */
	uint16 i;
	uint16 avgduty;
	uint16 highlimit;
	uint16 lowlimit;

	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		avgduty = (inputdutysum[i] / (dplparam->dplUpdateFrames / dplparam->dplSampleFrames)) / 3 * 2;

		highlimit = dplparam->dplLdDutySumLimitHighTemp - DPL_ZoneOffsetTable[i] - dplparam->dplTemperatureCalibration;
		lowlimit = dplparam->dplLdDutySumLimitLowTemp - DPL_ZoneOffsetTable[i] - dplparam->dplTemperatureCalibration;

		//Duty sum high , Limit step down ,but no lower than highlimit.
		//YZF 2016/5/20: + 0x0020 for noise reduction , avoid limit jump around high limit.
		if (avgduty > highlimit + 0x0020)
		{
			DPL_tempDutyLimitTable[i] = max(DPL_tempDutyLimitTable[i] - dplparam->dplLimitDownStep, highlimit);
		}
		//Duty sum high , Limit step down ,but no higher than dplLdDutyMax.
		else if (avgduty < lowlimit - 0x0020)
		{
			DPL_tempDutyLimitTable[i] = min(DPL_tempDutyLimitTable[i] + dplparam->dplLimitUpStep, dplparam->dplLdDutyMax);
		}
	}

	//Sum = Sum / 2
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		inputdutysum[i] = inputdutysum[i] >> 1;
	}

}

void DPL_correctGamma(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	static uint16 gp_now[5];

	//If Gamma point changed. Caiculate 256 point Gamma table accroding to 5 point input.
	if ((gp_now[0] != dplparam->dplInputGammaGp0x000) || (gp_now[1] != dplparam->dplInputGammaGp0x3F0)
			|| (gp_now[2] != dplparam->dplInputGammaGp0x7F0) || (gp_now[3] != dplparam->dplInputGammaGp0xBF0)
			|| (gp_now[4] != dplparam->dplInputGammaGp0xFF0))
	{

		gp_now[0] = dplparam->dplInputGammaGp0x000;
		gp_now[1] = dplparam->dplInputGammaGp0x3F0;
		gp_now[2] = dplparam->dplInputGammaGp0x7F0;
		gp_now[3] = dplparam->dplInputGammaGp0xBF0;
		gp_now[4] = dplparam->dplInputGammaGp0xFF0;

		uint8 i;
		uint16 step1, step2, step3, step4;

		step1 = (dplparam->dplInputGammaGp0x3F0 - dplparam->dplInputGammaGp0x000) / 63;
		step2 = (dplparam->dplInputGammaGp0x7F0 - dplparam->dplInputGammaGp0x3F0) / 63;
		step3 = (dplparam->dplInputGammaGp0xBF0 - dplparam->dplInputGammaGp0x7F0) / 63;
		step4 = (dplparam->dplInputGammaGp0xFF0 - dplparam->dplInputGammaGp0xBF0) / 63;

		for (i = 0; i < 63; i++)
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x000 + step1 * (i);
		}
		for (i = 63; i < 127; i++)
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x3F0 + step2 * (i - 63);
		}
		for (i = 127; i < 191; i++)
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x7F0 + step3 * (i - 127);
		}
		for (i = 191; i < 255; i++)
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0xBF0 + step4 * (i - 191);
		}
		DPL_InputGamma[255] = dplparam->dplInputGammaGp0xFF0;
	}

	//Gamma correction
	if (dplparam->dplInputGammaEnable)
	{
		uint16 i;
		for (i = 0; i < dplparam->dplChannelAmount; i++)
		{
			outputduty[i] = DPL_InputGamma[inputduty[i] >> 4];
		}
	}

}

/***2.6 External Functions ***************************************************/

/* DPL_Function
 * @Brief
 * 		Dynamic Power Limit function , limit local / global duty to protect LED bars.
 * 		This function has 4 sub functions:
 * 		DPL_limitLocalDuty		: Single CH LED duty limit.
 * 		DPL_limitGlobalDuty		: All CH LED limit.
 * 		DPL_sumLocalDuty		: Sample & Sum up every single CH duty every certain time(typ 1sec)
 * 		DPL_updateParam			: Update the limit tabel for DPL_limitLocalDuty accroding to certain amount samples (typ 60 samples)
 * @variables
 * 		*inputduty				: Duty matrix input
 * 		*outputduty				: Duty Matrix output (power limited)
 * 		dplparam				: DPL function parameters , DPL_Prama type .
 * @return
 * 		FLAG_SUCCESS			: DPL function finish
 *
 */
uint8 DPL_Function(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	static uint8 firstrun;

	//Auto initialize param.
	if (firstrun == 0)
	{
		//Load default param.
		Mem_copy((uint32) *&dplparam, (uint32) &DPL_DefaultParam, sizeof(DPL_DefaultParam));
		firstrun = 1;
	}

	//DPL main struct.
	if (dplparam->dplOn)
	{
		//STEP1: Gamma correction.
		DPL_correctGamma(inputduty, inputduty, dplparam);
		//STEP2: Input -> LD limit -> GD limit -> Output
		DPL_limitLocalDuty(inputduty, DPL_tempDutyMatrix, dplparam);
		DPL_limitGlobalDuty(DPL_tempDutyMatrix, outputduty, dplparam);
		//STEP3: Sample
		if (DPL_tempSampleCount % dplparam->dplSampleFrames == 1)
		{
			DPL_sumLocalDuty(outputduty, DPL_tempSumDutyMatrix, dplparam);
		}
		//STEP4: Update Param
		if (DPL_tempSampleCount == dplparam->dplUpdateFrames)
		{
			DPL_updateParam(DPL_tempSumDutyMatrix, dplparam);
			DPL_tempSampleCount = 0;
		}
		DPL_tempSampleCount++;
	}
	else    //when DPL is off , bypass input to output.
	{
		Mem_copy((uint32) &*outputduty, (uint32) &*inputduty, dplparam->dplChannelAmount * 2);
		DPL_tempSampleCount = 0;
	}
	return FLAG_SUCCESS;
}

void DPL_caliberateTemp(int8 temp, DPL_Prama *dplparam)
{
	//Set high/low limt
	temp = max(temp, 0);
	temp = min(temp, 59);

	//Set temp caliberation
	dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable[temp];

}

