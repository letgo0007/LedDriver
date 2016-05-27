#include "app_dpl.h"
#include "math.h"
extern Mem_copy(uint32 target_add, uint32 source_add, uint16 size);
extern Mem_set16();
/* DPL_Function
 * @Brief
 * 		Dynamic Power Limit function , limit local / global duty to protect LED bars.
 * 		This function has 4 sub functions:
 * 		DPL_limitLocalDuty		: Single CH LED duty limit.
 * 		DPL_limitGlobalDuty		: All CH LED limit.
 * 		DPL_sumLocalDuty	: Sample & Sum up every single CH duty every certain time(typ 1sec)
 * 		DPL_updateParam	: Update the limit tabel for DPL_limitLocalDuty accroding to certain amount samples (typ 60 samples)
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

	//Auto Initialize
	if (firstrun == 0)
	{
		Mem_copy((uint32) *&dplparam, (uint32) &DPL_DefaultParam, sizeof(DPL_DefaultParam));
		Mem_set16((uint32) &DPL_tempDutyLimitTable, dplparam->dplLdDutyMax, sizeof(DPL_tempDutyLimitTable) / 2);
		firstrun = 1;
	}

	/*------------------------------------------------------------------
	 *
	 *
	 *
	 -------------------------------------------------------------------*/
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

void DPL_limitLocalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	uint16 i = 0;

	//Limit every local duty <= dplLdLimitTable
	for (i = 0; i < dplparam->dplChannelAmount; i++)
	{
		if (inputduty[i] > (DPL_tempDutyLimitTable[i]))
		{
			outputduty[i] = DPL_tempDutyLimitTable[i];
		}
		else
		{
			outputduty[i] = inputduty[i];
		}
	}
}

void DPL_limitGlobalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam)
{
	uint16 i = 0;
	uint32 sum = 0;
	uint16 avg = 0;
	uint32 gain = 0;

	//calculate average duty
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
			DPL_tempDutyLimitTable[i] = fmax(DPL_tempDutyLimitTable[i] - dplparam->dplLimitDownStep, highlimit);
		}
		//Duty sum high , Limit step down ,but no higher than dplLdDutyMax.
		else if (avgduty < lowlimit - 0x0020)
		{
			DPL_tempDutyLimitTable[i] = fmin(DPL_tempDutyLimitTable[i] + dplparam->dplLimitUpStep, dplparam->dplLdDutyMax);
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

	if ((gp_now[0] != dplparam->dplInputGammaGp0x000) || (gp_now[1] != dplparam->dplInputGammaGp0x3F0) || (gp_now[2] != dplparam->dplInputGammaGp0x7F0)
			|| (gp_now[3] != dplparam->dplInputGammaGp0xBF0) || (gp_now[4] != dplparam->dplInputGammaGp0xFF0))
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

void DPL_caliberateTemp(int8 temp, DPL_Prama *dplparam)
{
	//Set high/low limt
	temp = fmax(temp, 0);
	temp = fmin(temp, 59);

	//Set temp caliberation
	dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable[temp];

}

