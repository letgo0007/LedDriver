#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "driverlib.h"

//Struct
typedef struct Scheduler
{
	uint8_t  schSystemOn;
	uint8_t  schLocalDimmingOn;
	uint16_t schCpuTickPeriod;
	uint16_t schGpioCheckTPeriod;
	uint16_t schErrorHandlePeriod;
	uint8_t	 taskFlagSpiRx;
	uint8_t	 taskFlagSpiTx;
	uint8_t	 taskFlagI2c;
	uint8_t  taskFlagGpioCheck;
	uint8_t  taskFlagErrorHandle;
	uint16_t cpuTickCount;
	uint8_t  cpuLoad;
}Scheduler;

//Variables
Scheduler System_Schedule;
Calendar System_Time ;
uint16_t Sch_CpuOnMark;
uint16_t Sch_CpuWorkTime;

//Function Calls
uint8_t Sch_init(void);
void Sch_CpuOff(void);


#endif /* SCHEDULER_H_ */
