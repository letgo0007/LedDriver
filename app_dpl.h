#ifndef APP_DPL_H_
#define APP_DPL_H_

#include "std.h"

//Define const
#define DPL_LED_CH_MAX				(128)
#define DPL_VERSION					(0x01)

//Parameters Struct
typedef struct DPL_Prama
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
} DPL_Prama;

//To balance different enviroment temperature.
static const int16 DPL_TempCalibratineTable[60] =
{ 0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64,
		66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118, };

//To balance LED environment temperatuer on different area of a LED Panel.
static const uint16 DPL_ZoneOffsetTable[DPL_LED_CH_MAX] =
{ 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000, 0x0000, 0x0080, 0x0080,
		0x0080, 0x0000, 0x0000, 0x0000, 0x0080, 0x0000, 0x0040, 0x0080, 0x0000, 0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000, 0x0000, 0x0000, 0x0000, 0x0040, 0x0080, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0040, 0x0080, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0080, 0x0080, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

static const DPL_Prama DPL_DefaultParam =
{
	.dplOn = 1,
	.dplChannelAmount = 78,
	.dplInputGammaEnable = 1,
	.dplSampleFrames = 120,
	.dplUpdateFrames = 1200,
	.dplLimitUpStep = 0x0080,
	.dplLimitDownStep = 0x0080,
	.dplGdDutyMax = 0x0800,
	.dplLdDutyMax = 0x0F00,
	.dplTemperatureCalibration = 0x0000,
	.dplLdDutySumLimitHighTemp = 0x0800 ,
	.dplLdDutySumLimitLowTemp = 0x0700 ,
	.dplInputGammaGp0x000 = 0x0000,
	.dplInputGammaGp0x3F0 = 0x03F0,
	.dplInputGammaGp0x7F0 = 0x07F0,
	.dplInputGammaGp0xBF0 = 0x0BF0,
	.dplInputGammaGp0xFF0 = 0x0FF0,
};

#endif /* APP_DPL_H_ */

//Interface Variables - Parameters .
extern DPL_Prama System_DplParam;

//Read only Variables - Buffer.
uint16 DPL_tempSampleCount;
uint16 DPL_tempDutyMatrix[DPL_LED_CH_MAX];
uint32 DPL_tempSumDutyMatrix[DPL_LED_CH_MAX];
uint16 DPL_tempDutyLimitTable[DPL_LED_CH_MAX];
uint16 DPL_InputGamma[0x100];

//Function calls
uint8 DPL_Function(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam);
void DPL_limitLocalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam);
void DPL_limitGlobalDuty(uint16 *inputduty, uint16 *outputduty, DPL_Prama *dplparam);
void DPL_sumLocalDuty(uint16 *inputduty, uint32 *outputdutysum, DPL_Prama *dplparam);
void DPL_updateParam(uint32 *inputdutysum, DPL_Prama *dplparam);
void DPL_correctGamma(uint16 *inputduty, uint16 *outputduty, DPL_Prama *);
void DPL_caliberateTemp(int8 temp, DPL_Prama *dplparam);
void DPL_GammaUpdate(uint16 gp0, uint16 gp63, uint16 gp127, uint16 gp191, uint16 gp255);
