#ifndef APP_DPL_H_
#define APP_DPL_H_

#include "driverlib.h"
//Define const
#define DPL_LED_CH_MAX					(80)

//Parameters Struct
typedef struct DPL_Prama
{
	//On/Off control of DPL
	uint8_t		dplOn;
	//Duty channel amount
	uint8_t		dplChannelAmount;
	//Select input GAMMA curve
	uint8_t		dplInputGammaSelect;
	//Frame amount to run dpl sample , if=60 ,it means sample duty matrix evert 60 frames(1 sec)
	uint8_t		dplFrameAmountToSample;
	//Frame amount to run dpl param update, if=600 ,it means update duty limit every 600 frames(10 sec)
	uint16_t	dplFrameAmountToUpdateParam;
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
	//Dynamic Local limit tabel
	uint16_t 	dplLdDutyLimitTable[DPL_LED_CH_MAX];
	//Sum duty limit for 1min
	uint16_t	dplLdDutySumLimitHighTempTable[DPL_LED_CH_MAX];
	//Temprature Safe duty for every local
	uint16_t	dplLdDutySumLimitLowTempTable[DPL_LED_CH_MAX];
}DPL_Prama;

//Buffers
DPL_Prama System_DplParam ;
uint16_t DPL_tempDutyMatrix[DPL_LED_CH_MAX];
uint32_t DPL_tempSumDutyMatrix[DPL_LED_CH_MAX];
uint16_t DPL_tempSampleCount;

//Function calls
uint8_t DPL_Function(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_LocalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_GlobalDutyLimit(uint16_t *inputduty,uint16_t *outputduty,DPL_Prama *dplparam);
void DPL_LocalDutyStatistic(uint16_t *inputduty,uint32_t *outputdutysum,DPL_Prama *dplparam);
void DPL_ParametersUpdate(uint32_t *inputdutysum,DPL_Prama *dplparam);
void DPL_GammaCorrection(uint16_t *inputduty , uint16_t *outputduty , DPL_Prama *dplparam);
void DPL_TemperatureCalibration(int8_t temp, DPL_Prama *dplparam);

//Input Gamma Const Table
static const uint16_t DPL_InputGamma[4][256] = {
		//DPL_InputGamma[0] = bypass gamma
		0,		16,		32,		48,		64,		80,		96,		112,	128,	144,	160,	176,	192,	208,	224,	240,
		256,	272,	288,	304,	320,	336,	352,	368,	384,	400,	416,	432,	448,	464,	480,	496,
		512,	528,	544,	560,	576,	592,	608,	624,	640,	656,	672,	688,	704,	720,	736,	752,
		768,	784,	800,	816,	832,	848,	864,	880,	896,	912,	928,	944,	960,	976,	992,	1008,
		1024,	1040,	1056,	1072,	1088,	1104,	1120,	1136,	1152,	1168,	1184,	1200,	1216,	1232,	1248,	1264,
		1280,	1296,	1312,	1328,	1344,	1360,	1376,	1392,	1408,	1424,	1440,	1456,	1472,	1488,	1504,	1520,
		1536,	1552,	1568,	1584,	1600,	1616,	1632,	1648,	1664,	1680,	1696,	1712,	1728,	1744,	1760,	1776,
		1792,	1808,	1824,	1840,	1856,	1872,	1888,	1904,	1920,	1936,	1952,	1968,	1984,	2000,	2016,	2032,
		2048,	2064,	2080,	2096,	2112,	2128,	2144,	2160,	2176,	2192,	2208,	2224,	2240,	2256,	2272,	2288,
		2304,	2320,	2336,	2352,	2368,	2384,	2400,	2416,	2432,	2448,	2464,	2480,	2496,	2512,	2528,	2544,
		2560,	2576,	2592,	2608,	2624,	2640,	2656,	2672,	2688,	2704,	2720,	2736,	2752,	2768,	2784,	2800,
		2816,	2832,	2848,	2864,	2880,	2896,	2912,	2928,	2944,	2960,	2976,	2992,	3008,	3024,	3040,	3056,
		3072,	3088,	3104,	3120,	3136,	3152,	3168,	3184,	3200,	3216,	3232,	3248,	3264,	3280,	3296,	3312,
		3328,	3344,	3360,	3376,	3392,	3408,	3424,	3440,	3456,	3472,	3488,	3504,	3520,	3536,	3552,	3568,
		3584,	3600,	3616,	3632,	3648,	3664,	3680,	3696,	3712,	3728,	3744,	3760,	3776,	3792,	3808,	3824,
		3840,	3856,	3872,	3888,	3904,	3920,	3936,	3952,	3968,	3984,	4000,	4016,	4032,	4048,	4064,	4080,
		//Gamma Table 1
		0,		16,		32,		48,		64,		80,		96,		112,	128,	144,	160,	176,	192,	208,	224,	240,
		256,	272,	288,	304,	320,	336,	352,	368,	384,	400,	416,	432,	448,	464,	480,	496,
		512,	528,	544,	560,	576,	592,	608,	624,	640,	656,	672,	688,	704,	720,	736,	752,
		768,	784,	800,	816,	832,	848,	864,	880,	896,	912,	928,	944,	960,	976,	992,	1008,
		1024,	1040,	1056,	1072,	1088,	1104,	1120,	1136,	1152,	1168,	1184,	1200,	1216,	1232,	1248,	1264,
		1280,	1296,	1312,	1328,	1344,	1360,	1376,	1392,	1408,	1424,	1440,	1456,	1472,	1488,	1504,	1520,
		1536,	1552,	1568,	1584,	1600,	1616,	1632,	1648,	1664,	1680,	1696,	1712,	1728,	1744,	1760,	1776,
		1792,	1808,	1824,	1840,	1856,	1872,	1888,	1904,	1920,	1936,	1952,	1968,	1984,	2000,	2016,	2032,
		2048,	2064,	2080,	2096,	2112,	2128,	2144,	2160,	2176,	2192,	2208,	2224,	2240,	2256,	2272,	2288,
		2304,	2320,	2336,	2352,	2368,	2384,	2400,	2416,	2432,	2448,	2464,	2480,	2496,	2512,	2528,	2544,
		2560,	2576,	2592,	2608,	2624,	2640,	2656,	2672,	2688,	2704,	2720,	2736,	2752,	2768,	2784,	2800,
		2816,	2832,	2848,	2864,	2880,	2896,	2912,	2928,	2944,	2960,	2976,	2992,	3008,	3024,	3040,	3056,
		3072,	3088,	3104,	3120,	3136,	3152,	3168,	3184,	3200,	3216,	3232,	3248,	3264,	3280,	3296,	3312,
		3328,	3344,	3360,	3376,	3392,	3408,	3424,	3440,	3456,	3472,	3488,	3504,	3520,	3536,	3552,	3568,
		3584,	3600,	3616,	3632,	3648,	3664,	3680,	3696,	3712,	3728,	3744,	3760,	3776,	3792,	3808,	3824,
		3840,	3856,	3872,	3888,	3904,	3920,	3936,	3952,	3968,	3984,	4000,	4016,	4032,	4048,	4064,	4080,
		//Gamma Table 2
		0,		16,		32,		48,		64,		80,		96,		112,	128,	144,	160,	176,	192,	208,	224,	240,
		256,	272,	288,	304,	320,	336,	352,	368,	384,	400,	416,	432,	448,	464,	480,	496,
		512,	528,	544,	560,	576,	592,	608,	624,	640,	656,	672,	688,	704,	720,	736,	752,
		768,	784,	800,	816,	832,	848,	864,	880,	896,	912,	928,	944,	960,	976,	992,	1008,
		1024,	1040,	1056,	1072,	1088,	1104,	1120,	1136,	1152,	1168,	1184,	1200,	1216,	1232,	1248,	1264,
		1280,	1296,	1312,	1328,	1344,	1360,	1376,	1392,	1408,	1424,	1440,	1456,	1472,	1488,	1504,	1520,
		1536,	1552,	1568,	1584,	1600,	1616,	1632,	1648,	1664,	1680,	1696,	1712,	1728,	1744,	1760,	1776,
		1792,	1808,	1824,	1840,	1856,	1872,	1888,	1904,	1920,	1936,	1952,	1968,	1984,	2000,	2016,	2032,
		2048,	2064,	2080,	2096,	2112,	2128,	2144,	2160,	2176,	2192,	2208,	2224,	2240,	2256,	2272,	2288,
		2304,	2320,	2336,	2352,	2368,	2384,	2400,	2416,	2432,	2448,	2464,	2480,	2496,	2512,	2528,	2544,
		2560,	2576,	2592,	2608,	2624,	2640,	2656,	2672,	2688,	2704,	2720,	2736,	2752,	2768,	2784,	2800,
		2816,	2832,	2848,	2864,	2880,	2896,	2912,	2928,	2944,	2960,	2976,	2992,	3008,	3024,	3040,	3056,
		3072,	3088,	3104,	3120,	3136,	3152,	3168,	3184,	3200,	3216,	3232,	3248,	3264,	3280,	3296,	3312,
		3328,	3344,	3360,	3376,	3392,	3408,	3424,	3440,	3456,	3472,	3488,	3504,	3520,	3536,	3552,	3568,
		3584,	3600,	3616,	3632,	3648,	3664,	3680,	3696,	3712,	3728,	3744,	3760,	3776,	3792,	3808,	3824,
		3840,	3856,	3872,	3888,	3904,	3920,	3936,	3952,	3968,	3984,	4000,	4016,	4032,	4048,	4064,	4080,
		//Gamma Table 3
		0,		16,		32,		48,		64,		80,		96,		112,	128,	144,	160,	176,	192,	208,	224,	240,
		256,	272,	288,	304,	320,	336,	352,	368,	384,	400,	416,	432,	448,	464,	480,	496,
		512,	528,	544,	560,	576,	592,	608,	624,	640,	656,	672,	688,	704,	720,	736,	752,
		768,	784,	800,	816,	832,	848,	864,	880,	896,	912,	928,	944,	960,	976,	992,	1008,
		1024,	1040,	1056,	1072,	1088,	1104,	1120,	1136,	1152,	1168,	1184,	1200,	1216,	1232,	1248,	1264,
		1280,	1296,	1312,	1328,	1344,	1360,	1376,	1392,	1408,	1424,	1440,	1456,	1472,	1488,	1504,	1520,
		1536,	1552,	1568,	1584,	1600,	1616,	1632,	1648,	1664,	1680,	1696,	1712,	1728,	1744,	1760,	1776,
		1792,	1808,	1824,	1840,	1856,	1872,	1888,	1904,	1920,	1936,	1952,	1968,	1984,	2000,	2016,	2032,
		2048,	2064,	2080,	2096,	2112,	2128,	2144,	2160,	2176,	2192,	2208,	2224,	2240,	2256,	2272,	2288,
		2304,	2320,	2336,	2352,	2368,	2384,	2400,	2416,	2432,	2448,	2464,	2480,	2496,	2512,	2528,	2544,
		2560,	2576,	2592,	2608,	2624,	2640,	2656,	2672,	2688,	2704,	2720,	2736,	2752,	2768,	2784,	2800,
		2816,	2832,	2848,	2864,	2880,	2896,	2912,	2928,	2944,	2960,	2976,	2992,	3008,	3024,	3040,	3056,
		3072,	3088,	3104,	3120,	3136,	3152,	3168,	3184,	3200,	3216,	3232,	3248,	3264,	3280,	3296,	3312,
		3328,	3344,	3360,	3376,	3392,	3408,	3424,	3440,	3456,	3472,	3488,	3504,	3520,	3536,	3552,	3568,
		3584,	3600,	3616,	3632,	3648,	3664,	3680,	3696,	3712,	3728,	3744,	3760,	3776,	3792,	3808,	3824,
		3840,	3856,	3872,	3888,	3904,	3920,	3936,	3952,	3968,	3984,	4000,	4016,	4032,	4048,	4064,	4080,
};

static const int16_t DPL_TempCalibratineTable[60] = {
	//	0	��	1	��	2	��	3	��	4	��	5	��	6	��	7	��	8	��	9	��
		-50	,	-48	,	-46	,	-44	,	-42	,	-40	,	-38	,	-36	,	-34	,	-32	,
	//	10	��	11	��	12	��	13	��	14	��	15	��	16	��	17	��	18	��	19	��
		-30	,	-28	,	-26	,	-24	,	-22	,	-20	,	-18	,	-16	,	-14	,	-12	,
	//	20	��	21	��	22	��	23	��	24	��	25	��	26	��	27	��	28	��	29	��
		-10	,	-8	,	-6	,	-4	,	-2	,	0	,	2	,	4	,	6	,	8	,
	//	30	��	31	��	32	��	33	��	34	��	35	��	36	��	37	��	38	��	39	��
		10	,	12	,	14	,	16	,	18	,	20	,	22	,	24	,	26	,	28	,
	//	40	��	41	��	42	��	43	��	44	��	45	��	46	��	47	��	48	��	49	��
		30	,	32	,	34	,	36	,	38	,	40	,	42	,	44	,	46	,	48	,
	//	50	��	51	��	52	��	53	��	54	��	55	��	56	��	57	��	58	��	59	��
		50	,	52	,	54	,	56	,	58	,	60	,	62	,	64	,	66	,	68	,
};

static const DPL_Prama DPL_Param_70XU30A_80CH_320ma_12bit = {
		1,		//DPL on
		80,		//LED channel
		0,		//Input Gamma curve 0
		120,	//Sample every 1s (120frame)
		1200,	//Update param every 10s (1200param)
		0x0100,	//Limit up step
		0x0100,	//Limit down step
		0x0800,	//DPL GD duty max
		0x0FFF,	//DPL LD duty max
		0x0000,	//Temperature Calibration
		{		//Default Local duty limit 0~80
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
		},
		{		//High temprature limit 0~80
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
				0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 , 0x0800 ,
		},
		{		//Low temprature limit
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
				0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 , 0x0700 ,
		},
};

static const DPL_Prama DPL_Param_70XU30A_80CH_250ma_12bit = {
		1,		//DPL on
		80,		//LED channel
		0,		//Input Gamma curve 0
		120,	//Sample every 1s (120frame)
		600,	//Update param every 10s (1200param)
		0x0080,	//Limit up step
		0x0080,	//Limit down step
		0x0200,	//DPL GD duty max
		0x0FFF,	//DPL LD duty max
		0x0000,	//Temperature Calibration
		{		//Default Local duty limit
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
				0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 , 0x0FF0 ,
		},
		{		//High temprature limit
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
				0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 , 0x0B00 ,
		},
		{		//Low temprature limit
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
				0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 , 0x0900 ,
		},
};

#endif /* APP_DPL_H_ */
