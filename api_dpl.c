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

#include "api_dpl.h"

/***2.1 Internal Marcos ******************************************************/
//Default param & tables when start up.
#define Dpl_DefaultParam			Dpl_DefaultParam_70XU30A
#define Dpl_TempCalibratineTable	Dpl_TempCalibratineTable_70XU30A
#define Dpl_ZoneOffsetTable			Dpl_ZoneOffsetTable_70XU30A
//Dc power share area define .
#define Dpl_Dc0ShareChAmount		78
#define Dpl_Dc1ShareChAmount		0
#define Dpl_Dc0ShareIndex			Dpl_Dc0ShareIndex_70XU30A
#define Dpl_Dc1ShareIndex			Dpl_Dc1ShareIndex_70XU30A
/***2.2 Internal Struct ******************************************************/

/***2.3 Internal Variables ***************************************************/
//Buffers
uint16 u16Dpl_tempSampleCount =
{ 0 };
uint16 u16Dpl_tempDutyMatrix[DPL_LED_CH_MAX] =
{ 0 };
uint32 u32Dpl_tempSumDutyMatrix[DPL_LED_CH_MAX] =
{ 0 };
uint16 u16Dpl_tempDutyLimitTable[DPL_LED_CH_MAX] =
{ 0 };
uint16 u16Dpl_InputGamma[0x100] =
{ 0 };

//Default params.
static const Dpl_Prama_t Dpl_DefaultParam_70XU30A =
{ .dplOn = 1, .dplChannelAmount = 78, .dplInputGammaEnable = 1, .dplSampleFrames = 120, .dplUpdateFrames = 1200,
		.dplLimitUpStep = 0x0080, .dplLimitDownStep = 0x0080, .dplGdDutyMax = 0x0800, .dplLdDutyMax = 0x1500,
		.dplTemperatureCalibration = 0x0000, .dplLdDutySumLimitHighTemp = 0x0800, .dplLdDutySumLimitLowTemp = 0x0700,
		.dplInputGammaGp0x000 = 0x0000, .dplInputGammaGp0x3F0 = 0x03F0, .dplInputGammaGp0x7F0 = 0x07F0, .dplInputGammaGp0xBF0 =
				0x0BF0, .dplInputGammaGp0xFF0 = 0x0FF0, };

//To balance different enviroment temperature.
static const int16 Dpl_TempCalibratineTable_70XU30A[60] =
{ 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, };

//To balance LED environment temperature on different area of a LED Panel.
static const uint16 Dpl_ZoneOffsetTable_70XU30A[DPL_LED_CH_MAX] =
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

#if Dpl_Dc0ShareChAmount
//DC power supply 0 share area number , 70XU30A use only 1 DC power for all area.
static const uint8 Dpl_Dc0ShareIndex_70XU30A[Dpl_Dc0ShareChAmount] =
{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
		0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
		0x29, 0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B, 0x3C,
		0x3D, 0x3E, 0x3F, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, };
#endif

#if Dpl_Dc1ShareChAmount
//DC power supply 1 share area number
static const uint8 Dpl_Dc1ShareIndex_70XU30A[Dpl_Dc1ShareChAmount] =
{	0};
#endif

/***2.4 External Variables ***************************************************/

/***2.5 Internal Functions ***************************************************/

void Dpl_limitLocalDuty(uint16 *inputduty, uint16 *outputduty, Dpl_Prama_t *dplparam)
{
	static uint16 ldmax_now;
	uint16 i = 0;

	//If LD duty  modified , reset all duty limit to new max value.
	if (ldmax_now != dplparam->dplLdDutyMax)
	{
		ldmax_now = dplparam->dplLdDutyMax;
		Hal_Mem_set16((uint32) &u16Dpl_tempDutyLimitTable, dplparam->dplLdDutyMax, sizeof(u16Dpl_tempDutyLimitTable) / 2);
	}

	//Limit every local duty <= dplLdLimitTable
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		outputduty[i] = min(inputduty[i], u16Dpl_tempDutyLimitTable[i]);
	}
}

void Dpl_limitGlobalDuty(uint16 *inputduty, uint16 *outputduty, Dpl_Prama_t *dplparam)
{
	uint16 i = 0;
	uint32 sum[2] =
	{ 0 };
	uint16 avgz[2] =
	{ 0 };
	uint16 avg = 0;
	uint32 gain = 0;

	//Calculate average duty for 2 Dc share area. Totoal average = max area.
#if Dpl_Dc0ShareChAmount
	for (i = 0; i < Dpl_Dc0ShareChAmount; i++)
	{
		sum[0] = sum[0] + inputduty[Dpl_Dc0ShareIndex[i]];
	}
	avgz[0] = sum[0] / Dpl_Dc0ShareChAmount;
#endif

#if Dpl_Dc1ShareChAmount
	for (i = 0; i < Dpl_Dc1ShareChAmount; i++)
	{
		sum[1] = sum[1] + inputduty[Dpl_Dc1ShareIndex[i]];
	}
	avgz[1] = sum[1] / Dpl_Dc1ShareChAmount;
#endif

	avg = max(avgz[0], avgz[1]);

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
		Hal_Mem_copy((uint32) &*outputduty, (uint32) &*inputduty, (dplparam->dplChannelAmount) * 2);
	}

}

void Dpl_sumLocalDuty(uint16 *inputduty, uint32 *outputdutysum, Dpl_Prama_t *dplparam)
{
	uint16 i = 0;
	//Sum up duty of every local
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		outputdutysum[i] += inputduty[i];
	}

}

void Dpl_updateParam(uint32 *inputdutysum, Dpl_Prama_t *dplparam)
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

		highlimit = dplparam->dplLdDutySumLimitHighTemp - Dpl_ZoneOffsetTable[i] - dplparam->dplTemperatureCalibration;
		lowlimit = dplparam->dplLdDutySumLimitLowTemp - Dpl_ZoneOffsetTable[i] - dplparam->dplTemperatureCalibration;

		//Duty sum high , Limit step down ,but no lower than highlimit.
		//YZF 2016/5/20: + 0x0020 for noise reduction , avoid limit jump around high limit.
		if (avgduty > highlimit + 0x0020)
		{
			u16Dpl_tempDutyLimitTable[i] = max(u16Dpl_tempDutyLimitTable[i] - dplparam->dplLimitDownStep, highlimit);
		}
		//Duty sum high , Limit step down ,but no higher than dplLdDutyMax.
		else if (avgduty < lowlimit - 0x0020)
		{
			u16Dpl_tempDutyLimitTable[i] = min(u16Dpl_tempDutyLimitTable[i] + dplparam->dplLimitUpStep, dplparam->dplLdDutyMax);
		}
	}

	//Sum = Sum / 2
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		inputdutysum[i] = inputdutysum[i] >> 1;
	}

}

void Dpl_correctGamma(uint16 *inputduty, uint16 *outputduty, Dpl_Prama_t *dplparam)
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
			u16Dpl_InputGamma[i] = dplparam->dplInputGammaGp0x000 + step1 * (i);
		}
		for (i = 63; i < 127; i++)
		{
			u16Dpl_InputGamma[i] = dplparam->dplInputGammaGp0x3F0 + step2 * (i - 63);
		}
		for (i = 127; i < 191; i++)
		{
			u16Dpl_InputGamma[i] = dplparam->dplInputGammaGp0x7F0 + step3 * (i - 127);
		}
		for (i = 191; i < 255; i++)
		{
			u16Dpl_InputGamma[i] = dplparam->dplInputGammaGp0xBF0 + step4 * (i - 191);
		}
		u16Dpl_InputGamma[255] = dplparam->dplInputGammaGp0xFF0;
	}

	//Gamma correction
	if (dplparam->dplInputGammaEnable)
	{
		uint16 i;
		for (i = 0; i < dplparam->dplChannelAmount; i++)
		{
			outputduty[i] = u16Dpl_InputGamma[inputduty[i] >> 4];
		}
	}

}

/***2.6 External Functions ***************************************************/

uint8 Dpl_process(uint16 *inputduty, uint16 *outputduty, Dpl_Prama_t *dplparam)
{
	static uint8 firstrun;

	//Auto initialize param.
	if (firstrun == 0)
	{
		//Load default param.
		Hal_Mem_copy((uint32) *&dplparam, (uint32) &Dpl_DefaultParam, sizeof(Dpl_DefaultParam));
		firstrun = 1;
	}

	//Dpl main struct.
	if (dplparam->dplOn)
	{
		//STEP1: Gamma correction.
		Dpl_correctGamma(inputduty, inputduty, dplparam);
		//STEP2: Input -> LD limit -> GD limit -> Output
		Dpl_limitLocalDuty(inputduty, u16Dpl_tempDutyMatrix, dplparam);
		Dpl_limitGlobalDuty(u16Dpl_tempDutyMatrix, outputduty, dplparam);
		//STEP3: Sample
		if (u16Dpl_tempSampleCount % dplparam->dplSampleFrames == 1)
		{
			Dpl_sumLocalDuty(outputduty, u32Dpl_tempSumDutyMatrix, dplparam);
		}
		//STEP4: Update Param
		if (u16Dpl_tempSampleCount == dplparam->dplUpdateFrames)
		{
			Dpl_updateParam(u32Dpl_tempSumDutyMatrix, dplparam);
			u16Dpl_tempSampleCount = 0;
		}
		u16Dpl_tempSampleCount++;
	}
	else    //when Dpl is off , bypass input to output.
	{
		Hal_Mem_copy((uint32) &*outputduty, (uint32) &*inputduty, dplparam->dplChannelAmount * 2);
		u16Dpl_tempSampleCount = 0;
	}
	return FLAG_SUCCESS;
}

void Dpl_caliberateTemp(int8 temp, Dpl_Prama_t *dplparam)
{
	//Set high/low limt
	temp = max(temp, 0);
	temp = min(temp, 59);

	//Set temp caliberation
	dplparam->dplTemperatureCalibration = Dpl_TempCalibratineTable[temp];

}

