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

typedef struct Scheduler
{
	//[1] Normal Working [0] Reboot
	flag schSystemOn;
	//[1] Local Dimming Mode [0] Manual Mode
	flag schLocalDimmingOn;
	//[0x0001~0xFFFF] Period for CPU wake up , unit in 30.5us
	uint16 schCpuTickPeriod;
	//[0x0001~0xFFFF] Period for board status check , unit in 30.5us
	uint16 schBoardCheckPeriod;
	//[0x0001~0xFFFF] Peroid for Manual mode duty update , unit in 30.5us
	uint16 schManualModePeriod;
	//[1]Flag for Spi Rxed task ,triggered by Spi CS rising edge.
	flag taskFlagSpiRx;
	//[1]Flag for Spi Tx task , triggerd by Spi Rx data check OK.
	flag taskFlagSpiTx;
	//[1]Flag for I2c slave event ,triggerd by I2C STOP .
	flag taskFlagI2c;
	//[1]Flag for board check task , controled by schBoardCheckPeriod.
	flag taskFlagBoardCheck;
	//[1]Flag for Manual Mode duty update task , controled by schManualModePeriod.
	flag taskFlagManualMode;
	//[1]Test flag of 1s
	flag testFlag1Hz;
	//[1]Test flag of 60Hz
	flag testFlag60Hz;
	//[0x0001~0xFFFF] Cpu wake up tick count
	uint16 cpuTickCount;
	//[0~100]Cpu working load, unit in % .
	uint8 cpuLoad;
	//RESERVED
	uint8 reserved[0x0C];
} Scheduler;

/***2.3 External Variables ***/

//System scheduler ,manage task & cpu schedule
extern Scheduler SysParam_Schedule;

//System time ,indicates system work time (Read Time Clock)since start up
extern Calendar System_Time;

/***2.4 External Functions ***/

void Sch_init(void);

void Sch_CpuOff(void);

#endif /* DRIVER_SCHEDULER_H_ */
