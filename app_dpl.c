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
		DPL_GammaCorrection( inputduty ,inputduty ,dplparam);
		//Input -> LD limit -> GD limit -> Output
		DPL_LocalDutyLimit( inputduty , DPL_tempDutyMatrix , dplparam );
		DPL_GlobalDutyLimit( DPL_tempDutyMatrix , outputduty , dplparam );

		//Sample
		if( DPL_tempSampleCount % dplparam->dplFrameAmountToSample == 1)
		{
			DPL_LocalDutyStatistic( outputduty , DPL_tempSumDutyMatrix , dplparam);
		}
		//Update Param
		if(DPL_tempSampleCount == dplparam->dplFrameAmountToUpdateParam )
		{
			DPL_ParametersUpdate(DPL_tempSumDutyMatrix , dplparam);
			//Reset count
			DPL_tempSampleCount = 0;
		}
		DPL_tempSampleCount ++;
	}
	else//when DPL is off , bypass input to output.
	{
		memcpy( outputduty , inputduty , (dplparam->dplChannelAmount) * 2 );
		DPL_tempSampleCount = 0 ;
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
	uint32_t gain = 0;

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
		gain = (uint32_t)(dplparam->dplGdDutyMax) * 256 / avg ;

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
	 * [A] DutySum > HighTemp 			 , Limit = - step,
	 * [B] LowTemp <= DutySum <= HighTemp, Limit = HighTempLimit
	 * [C] DutySum < LowTemp			 , Limit = + step, until reach limit
	 */
	uint16_t i;
	uint16_t avgduty;
	uint16_t highlimit;
	uint16_t lowlimit;

	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		avgduty = inputdutysum[i] / (dplparam->dplFrameAmountToUpdateParam / dplparam->dplFrameAmountToSample ) ;
		highlimit = dplparam->dplLdDutySumLimitHighTempTable[i] - dplparam->dplTemperatureCalibration ;
		lowlimit = dplparam->dplLdDutySumLimitLowTempTable[i] - dplparam->dplTemperatureCalibration ;

		//[A]
		if( avgduty > highlimit )
		{
			dplparam->dplLdDutyLimitTable[i] -= dplparam->dplLimitDownStep;
		}
		//[C]
		else if( avgduty < lowlimit	)
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
			dplparam->dplLdDutyLimitTable[i] = highlimit ;
		}
	}

	//Clear Sum
	memset( inputdutysum , 0x00 , (dplparam->dplChannelAmount) * 4);

}

void DPL_GammaCorrection(uint16_t *inputduty , uint16_t *outputduty , DPL_Prama *dplparam)
{
	uint16_t i;
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		outputduty[i] =  DPL_InputGamma[dplparam->dplInputGammaSelect] [ inputduty[i]>>4 ] ;
	}
}

void DPL_TemperatureCalibration(int8_t temp, DPL_Prama *dplparam)
{
	if( temp < 0 )
	{
		// temp < 0C , use 0C param
		dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable [0] ;
	}
	else if( temp >59 )
	{
		// temp > 59C , use 59C param
		dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable [59] ;
	}
	else
	{
		// 0 < temp < 59
		dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable [temp];
	}
}
