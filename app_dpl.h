#ifndef APP_DPL_H_
#define APP_DPL_H_

#include "driverlib.h"

//Define const
#define DPL_LED_CH_MAX					(128)

//Parameters Struct
typedef struct DPL_Prama
{
	//On/Off control of DPL
	uint8_t		dplOn;
	//Duty channel amount
	uint8_t		dplChannelAmount;
	//Enable input gamma correction
	uint8_t		dplInputGammaEnable;
	//Frame amount to run dpl sample.
	uint8_t		dplSampleFrames;
	//Frame amount to run dpl param update.
	uint16_t	dplUpdateFrames;
	//Limit adjustment rise up step
	uint16_t	dplLimitUpStep;
	//Limit adjustment fall down step
	uint16_t	dplLimitDownStep;
	//Global average duty limit
	uint16_t 	dplGdDutyMax;
	//Local duty limit
	uint16_t 	dplLdDutyMax;
	//Limit Temperature calibration value;
	int16_t		dplTemperatureCalibration;
	//Sum duty limit for 1min
	uint16_t	dplLdDutySumLimitHighTemp;
	//Temprature Safe duty for every local
	uint16_t	dplLdDutySumLimitLowTemp;
	//Input Gamma point @ 0x0000
	uint16_t	dplInputGammaGp0x000;
	//Input Gamma point @ 0x03F0
	uint16_t	dplInputGammaGp0x3F0;
	//Input Gamma point @ 0x07F0
	uint16_t	dplInputGammaGp0x7F0;
	//Input Gamma point @ 0x0BF0
	uint16_t	dplInputGammaGp0xBF0;
	//Input Gamma point @ 0x0FF0
	uint16_t	dplInputGammaGp0xFF0;
	uint8_t  	reserved[0x11];
}DPL_Prama;


//To balance different enviroment temperature.
static const int16_t DPL_TempCalibratineTable[60] = {
		//0¡æ	1¡æ		2¡æ		3¡æ		4¡æ		5¡æ		6¡æ		7¡æ		8¡æ		9¡æ		10¡æ		11¡æ		12¡æ		13¡æ		14¡æ		15¡æ
		0,		2,		4,		6,		8,		10,		12,		14,		16,		18,		20,		22,		24,		26,		28,		30,
		//16¡æ	17¡æ		18¡æ		19¡æ		20¡æ		21¡æ		22¡æ		23¡æ		24¡æ		25¡æ		26¡æ		27¡æ		28¡æ		29¡æ		30¡æ		31¡æ
		32,		34,		36,		38,		40,		42,		44,		46,		48,		50,		52,		54,		56,		58,		60,		62,
		//32¡æ	33¡æ		34¡æ		35¡æ		36¡æ		37¡æ		38¡æ		39¡æ		40¡æ		41¡æ		42¡æ		43¡æ		44¡æ		45¡æ		46¡æ		47¡æ
		64,		66,		68,		70,		72,		74,		76,		78,		80,		82,		84,		86,		88,		90,		92,		94,
		//48¡æ	49¡æ		50¡æ		51¡æ		52¡æ		53¡æ		54¡æ		55¡æ		56¡æ		57¡æ		58¡æ		59¡æ
		96,		98,		100,	102,	104,	106,	108,	110,	112,	114,	116,	118,
};

//To balance LED environment temperatuer on different area of a LED Panel.
static const uint16_t DPL_EnvOffsetTable[DPL_LED_CH_MAX] = {
		//13 x 6 led matrix TV front view
		0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 ,
		0x0000 , 0x0000 , 0x0080 , 0x0080 , 0x0000 , 0x0000 ,
		0x0000 , 0x0080 , 0x0080 , 0x0080 , 0x0000 , 0x0000 ,
		0x0000 , 0x0080 , 0x0000 , 0x0040 , 0x0080 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0040 , 0x0080 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0040 , 0x0080 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0040 , 0x0080 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0040 , 0x0080 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 ,
		0x0000 , 0x0000 , 0x0080 , 0x0080 , 0x0000 , 0x0000 ,
		0x0000 , 0x0000 , 0x0080 , 0x0080 , 0x0000 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 ,
		0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 , 0x0000 ,
		//Unused
		0x0000 , 0x0000
};

static const DPL_Prama DPL_DefaultParam = {
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

//Buffers
extern DPL_Prama System_DplParam ;
uint16_t DPL_tempDutyMatrix[DPL_LED_CH_MAX];
uint32_t DPL_tempSumDutyMatrix[DPL_LED_CH_MAX];
uint16_t DPL_tempSampleCount;
uint16_t DPL_tempDutyLimitTable[DPL_LED_CH_MAX];
uint16_t DPL_InputGamma[256] ;

//Function calls
uint8_t DPL_Function(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_LocalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_GlobalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_LocalDutyStatistic(uint16_t *inputduty,uint32_t *outputdutysum,DPL_Prama *dplparam);
void DPL_ParametersUpdate(uint32_t *inputdutysum,DPL_Prama *dplparam);
void DPL_GammaCorrection(uint16_t *inputduty , uint16_t *outputduty , DPL_Prama *);
void DPL_TemperatureCalibration(int8_t temp, DPL_Prama *dplparam);


