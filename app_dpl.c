#include "app_dpl.h"
#include "string.h"
#include "math.h"

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
	static uint8_t firstrun;

	//Auto initial when 1st run
	if(firstrun == 0)
	{
		memcpy(dplparam , &DPL_DefaultParam , sizeof(DPL_DefaultParam) );
		uint8_t i;
		for(i = 0 ; i < DPL_LED_CH_MAX ; i++)
		{
			DPL_tempDutyLimitTable[i] = dplparam->dplLdDutyMax ;
		}
		firstrun = 1;
	}

	if(dplparam->dplOn)
	{
		//STEP1: Gamma correction.
		DPL_GammaCorrection( inputduty ,inputduty ,dplparam);
		//STEP2: Input -> LD limit -> GD limit -> Output
		DPL_LocalDutyLimit( inputduty , DPL_tempDutyMatrix , dplparam );
		DPL_GlobalDutyLimit( DPL_tempDutyMatrix , outputduty , dplparam );
		//STEP3: Sample
		if( DPL_tempSampleCount % dplparam->dplSampleFrames == 1)
		{
			DPL_LocalDutyStatistic( outputduty , DPL_tempSumDutyMatrix , dplparam);
		}
		//STEP4: Update Param
		if(DPL_tempSampleCount == dplparam->dplUpdateFrames )
		{
			DPL_ParametersUpdate(DPL_tempSumDutyMatrix , dplparam);
			DPL_tempSampleCount = 0;
		}
		DPL_tempSampleCount ++;
	}
	else//when DPL is off , bypass input to output.
	{
		//Size = channel * 2 bytes (16bit)
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
		if( inputduty[i] > (DPL_tempDutyLimitTable[i]) )
		{
			outputduty[i] = DPL_tempDutyLimitTable[i];
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
	uint32_t sum = 0;
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
		//Size = channel * 2 bytes (16bit)
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
	 * [A] DutySum > HighTemp 	, Limit = - step, no lower than High temp limit
	 * [C] DutySum < LowTemp	, Limit = + step, no higher than local limit max
	 */
	uint16_t i;
	uint16_t avgduty;
	uint16_t highlimit;
	uint16_t lowlimit;

	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		avgduty 	= (inputdutysum[i] / (dplparam->dplUpdateFrames / dplparam->dplSampleFrames )) / 3 * 2 ;

		highlimit 	= dplparam->dplLdDutySumLimitHighTemp - DPL_EnvOffsetTable[i] - dplparam->dplTemperatureCalibration ;
		lowlimit 	= dplparam->dplLdDutySumLimitLowTemp - DPL_EnvOffsetTable[i] - dplparam->dplTemperatureCalibration ;

		//Duty sum high , Limit step down ,but no lower than highlimit.
		//YZF 2016/5/20: + 0x0020 for noise reduction , avoid limit jump around high limit.
		if( avgduty > highlimit +  0x0020 )
		{
			DPL_tempDutyLimitTable[i] = fmax( DPL_tempDutyLimitTable[i] - dplparam->dplLimitDownStep , highlimit );
		}
		//Duty sum high , Limit step down ,but no higher than dplLdDutyMax.
		else if( avgduty < lowlimit -  0x0020)
		{
			DPL_tempDutyLimitTable[i] = fmin( DPL_tempDutyLimitTable[i] + dplparam->dplLimitUpStep , dplparam->dplLdDutyMax);
		}
	}

	//Sum = Sum / 2
	for(i=0;i<dplparam->dplChannelAmount;i++)
	{
		inputdutysum[i] = inputdutysum[i] >> 1;
	}

}

void DPL_GammaCorrection(uint16_t *inputduty , uint16_t *outputduty , DPL_Prama *dplparam)
{
	static uint16_t gp_now[5];

	if( 	(gp_now[0] != dplparam->dplInputGammaGp0x000 ) ||
			(gp_now[1] != dplparam->dplInputGammaGp0x3F0 ) ||
			(gp_now[2] != dplparam->dplInputGammaGp0x7F0 ) ||
			(gp_now[3] != dplparam->dplInputGammaGp0xBF0 ) ||
			(gp_now[4] != dplparam->dplInputGammaGp0xFF0 ) 		)
	{//If Gamma Curve param modified , caculate new gamma curve.

		gp_now[0] = dplparam->dplInputGammaGp0x000;
		gp_now[1] = dplparam->dplInputGammaGp0x3F0;
		gp_now[2] = dplparam->dplInputGammaGp0x7F0;
		gp_now[3] = dplparam->dplInputGammaGp0xBF0;
		gp_now[4] = dplparam->dplInputGammaGp0xFF0;

		uint8_t i;
		uint16_t step1,step2,step3,step4;

		step1 = (dplparam->dplInputGammaGp0x3F0   - dplparam->dplInputGammaGp0x000 ) /63;
		step2 = (dplparam->dplInputGammaGp0x7F0   - dplparam->dplInputGammaGp0x3F0  ) /63;
		step3 = (dplparam->dplInputGammaGp0xBF0   - dplparam->dplInputGammaGp0x7F0  ) /63;
		step4 = (dplparam->dplInputGammaGp0xFF0   - dplparam->dplInputGammaGp0xBF0  ) /63;

		for(i = 0 ; i < 63 ; i++ )
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x000  + step1 * (i);
		}
		for(i = 63 ; i < 127 ; i++ )
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x3F0   + step2 * (i-63) ;
		}
		for(i = 127 ; i < 191 ; i++ )
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0x7F0   + step3 * (i-127) ;
		}
		for(i = 191 ; i < 255 ; i++ )
		{
			DPL_InputGamma[i] = dplparam->dplInputGammaGp0xBF0   + step4 * (i-191) ;
		}
		DPL_InputGamma[255] = dplparam->dplInputGammaGp0xFF0   ;
	}

	//Gamma correction
	if(dplparam->dplInputGammaEnable)
	{
		uint16_t i;
		for(i=0;i<dplparam->dplChannelAmount;i++)
		{
			outputduty[i] =  DPL_InputGamma[ inputduty[i]>>4 ] ;
		}
	}

}

void DPL_TemperatureCalibration(int8_t temp, DPL_Prama *dplparam)
{
	//Set high/low limt
	temp = fmax(temp,0);
	temp = fmin(temp,59);

	//Set temp caliberation
	dplparam->dplTemperatureCalibration = DPL_TempCalibratineTable [temp] ;

}

