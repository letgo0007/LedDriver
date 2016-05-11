#include "app_dpl.h"

#include "string.h"

/* DPL_Function
 * @Brief
 * 		Dynamic Power Limit function , limit local / global duty to protect LED bars.
 * 		This function has 4 sub functions:
 * 		DPL_LocalDutyLimit		: Single CH LED duty limit.
 * 		DPL_GlobalDutyLimit		: All CH LED limit.
 * 		DPL_LocalDutyStatistic	: Sample & Sum up every single CH duty every certain time(typ 1sec)
 * 		DPL_ParametersUpdate	: Update the limit tabel for DPL_LocalDutyLimit accroding to certain amount samples (typ 60 samples)
 * @variables
 * 		*inputduty				: Duty matrix input
 * 		*outputduty				: Duty Matrix output (power limited)
 * 		dplparam				: DPL function parameters , DPL_Prama type .
 * @return
 * 		STATUS_SUCCESS			: DPL function finish
 *
 */
uint8_t DPL_Function(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam)
{
	if(dplparam->dplOn)
	{
		//Gamma correction.
		DPL_GammaCorrection( inputduty ,inputduty , DPL_InputGamma ,dplparam);
		//Input -> LD limit -> GD limit -> Output
		DPL_LocalDutyLimit( inputduty , DPL_tempDutyMatrix , dplparam );
		DPL_GlobalDutyLimit( DPL_tempDutyMatrix , outputduty , dplparam );

		//Sample 1frame data to Sum buffer
		if(dplparam->dplStartSample)
		{
			DPL_LocalDutyStatistic( outputduty , DPL_tempSumDutyMatrix , dplparam);
			dplparam->dplStartSample = 0;	//run statistic once a trigger.
			DPL_tempSampleCount ++;
		}
		//Update param when get enough samples
		if(DPL_tempSampleCount == dplparam->dplSampleAmoutToRunParamUpdate)
		{
			DPL_ParametersUpdate(DPL_tempSumDutyMatrix , dplparam);
			DPL_tempSampleCount = 0;
		}
	}
	else//when DPL is off , bypass input to output.
	{
		memcpy( outputduty , inputduty , (dplparam->dplChannelAmount) * 2 );
	}
	return STATUS_SUCCESS;
}

void DPL_LocalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam)
{
	uint16_t i = 0;

	//Limit every local duty <= dplLdLimitTable
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		if( inputduty[i] > (dplparam->dplLdDutyLimitTable[i]) )
		{
			outputduty[i] = dplparam->dplLdDutyLimitTable[i];
		}
		else
		{
			outputduty[i] = inputduty[i];
		}
	}
}

void DPL_GlobalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam)
{
	uint16_t i = 0;
	uint16_t sum = 0;
	uint16_t avg = 0;
	uint16_t gain = 0x100;

	//calculate average duty
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		sum = sum + inputduty[i];
	}
	avg = sum / dplparam->dplChannelAmount ;

	//If Over Limit, reduce duty by gain.
	if( avg > dplparam->dplGdDutyMax )
	{
		//gain = 1 ~ 255
		gain = 0x000100 * (dplparam->dplGdDutyMax) / avg ;

		//output = input * gian / 256
		for(i=0;i<dplparam->dplChannelAmount;i++)
		{
			outputduty[i] = (gain * inputduty[i]) >>8 ;
		}
	}
	//If Within Limit , Bypass input -> output .
	else
	{
		memcpy( outputduty , inputduty , (dplparam->dplChannelAmount) * 2 );
	}

}

void DPL_LocalDutyStatistic(uint16_t *inputduty , uint32_t *outputdutysum , DPL_Prama *dplparam)
{
	uint16_t i = 0;
	//Sum up duty of every local
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		outputdutysum[i] += inputduty[i];
	}

}

void DPL_ParametersUpdate(uint32_t *inputdutysum , DPL_Prama *dplparam)
{
	/*Update local duty limit table arroding to Local Duty Sum in the last 1min
	 * [A] DutySum > HighTemp 			 , Limit = - 0x100,
	 * [B] LowTemp <= DutySum <= HighTemp, Limit = HighTempLimit
	 * [C] DutySum < LowTemp			 , Limit = + 0x100, until reach limit
	 */
	uint16_t i;
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		//[A]
		if( inputdutysum[i] /DPL_tempSampleCount  > (dplparam->dplLdDutySumLimitHighTempTable[i]) )
		{
			dplparam->dplLdDutyLimitTable[i] -= dplparam->dplLimitDownStep;
		}
		//[C]
		else if( inputdutysum[i] /DPL_tempSampleCount  < (dplparam->dplLdDutySumLimitLowTempTable[i]) )
		{

			if(dplparam->dplLdDutyLimitTable[i] > dplparam->dplLdDutyMax - dplparam->dplLimitUpStep)
			{
				dplparam->dplLdDutyLimitTable[i] = dplparam->dplLdDutyMax;
			}
			else
			{
				dplparam->dplLdDutyLimitTable[i] += dplparam->dplLimitUpStep;
			}
		}
		//[B]
		else
		{
			dplparam->dplLdDutyLimitTable[i] = dplparam->dplLdDutySumLimitHighTempTable[i];
		}
	}

	//Clear Sum
	memset( inputdutysum , 0x00 , (dplparam->dplChannelAmount) * 4);

}

void DPL_GammaCorrection(uint16_t *inputduty , uint16_t *outputduty , const uint16_t *gammatable , DPL_Prama *dplparam)
{
	uint16_t i;
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		outputduty[i] =  gammatable [ inputduty[i]>>4 ] ;
	}
}
