/******************************************************************************
 * @file 	[driver_scheduler.h]
 *
 * Initial & setup initial CPU shceduler for NON-OS system .
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 ******************************************************************************/
#ifndef DRIVER_SCHEDULER_H_
#define DRIVER_SCHEDULER_H_

/***1 Includes ***************/

#include "driverlib.h"
#include "std.h"

/***2.1 External Macros ******/

/***2.2 External Structures **/

typedef struct Struct_CpuScheduler_t
{
	//[1] Normal Working [0] Reboot
	flag fSystemResetN;
	//[1] Local Dimming Mode [0] Manual Mode
	flag fLocalDimmingOn;
	//[0x0001~0xFFFF] Period for CPU wake up , unit in 30.5us
	uint16 u16CpuTickPeriod;
	//[0x0001~0xFFFF] Period for board status check , unit in 30.5us
	uint16 u16GpioCheckPeriod;
	//[0x0001~0xFFFF] Peroid for Manual mode duty update , unit in 30.5us
	uint16 u16TestModePeriod;
	//[1]Flag for Spi Rxed task ,triggered by Spi CS rising edge.
	flag fTaskFlagSpiRx;
	//[1]Flag for Spi Tx task , triggerd by Spi Rx data check OK.
	flag fTaskFlagSpiTx;
	//[1]Flag for I2c slave event ,triggerd by I2C STOP .
	flag fTaskFlagI2c;
	//[1]Flag for board check task , controled by u16GpioCheckPeriod.
	flag fTaskFlagGpioCheck;
	//[1]Flag for Manual Mode duty update task , controled by u16TestModePeriod.
	flag fTaskFlagTestMode;
	//[1]Test flag of 1s
	flag fTestFlag1Hz;
	//[1]Test flag of 60Hz
	flag fTestFlag60Hz;
	//[0~100]Cpu working load, unit in % .
	uint8 u8CpuLoad;
	//[0x0001~0xFFFF] Cpu wake up tick count
	uint16 u16CpuTickCount;
	//RESERVED
	uint8 RESERVED[0x0D];
} dStruct_CpuScheduler_t;

/***2.3 External Variables ***/

//System scheduler ,manage task & cpu schedule
extern dStruct_CpuScheduler_t SysParam_Schedule;

//System time ,indicates system work time (Read Time Clock)since start up
extern Calendar SysParam_Time;

/***2.4 External Functions ***/

void Sch_init(void);

void Sch_CpuOff(void);

#endif /* DRIVER_SCHEDULER_H_ */
