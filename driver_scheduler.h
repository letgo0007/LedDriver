#ifndef DRIVER_SCHEDULER_H_
#define DRIVER_SCHEDULER_H_

#include "driverlib.h"

//Struct
typedef struct Scheduler
{
	//[1] Normal Working [0] Reboot
	uint8_t  schSystemOn;
	//[1] Local Dimming Mode [0] Manual Mode
	uint8_t  schLocalDimmingOn;
	//Cpu wake up period ,unit in 30.5us
	uint16_t schCpuTickPeriod;
	//Board check period ,unit in 30.5us
	uint16_t schBoardCheckPeriod;
	//Manual mode working peroid , unit in 30.5us
	uint16_t schManualModePeriod;
	//Flag for Spi Rxed task ,triggered by Spi slave frame ISR.
	uint8_t	 taskFlagSpiRx;
	//Flag for Spi Tx task ,triggerd by Spi RX task success.
	uint8_t	 taskFlagSpiTx;
	//Flag for I2c salve rxed data ,triggerd by I2C stop ISR .
	uint8_t	 taskFlagI2c;
	//Flag for board check task , controled by schBoardCheckPeriod.
	uint8_t  taskFlagBoardCheck;
	//Flag for Manual Mode task , controled by schManualModePeriod.
	uint8_t  taskFlagManualMode;
	//Test flag of 1s
	uint8_t  testFlag1Hz;
	//Test flag of 60Hz
	uint8_t  testFlag60Hz;
	//Test cpu tick count
	uint16_t cpuTickCount;
	//Cpu working load, unit in % .
	uint8_t  cpuLoad;
	uint8_t  reserved[0x0c];
}Scheduler;

//Variables
extern Scheduler System_Schedule;
Calendar System_Time ;
uint16_t Sch_CpuOnMark;
uint32_t Sch_CpuWorkTime;

//Function Calls
/**********************************************************
 * @Brief Sch_init
 * 		Initialize TIMERB0 & RTC as scheduler.
 * 		Set default scheduler peroid.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Sch_init(void);
/**********************************************************
 * @Brief Sch_init
 * 		Turn off CPU with CPU load mark function.
 * 		Use this at the end of a loop.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Sch_CpuOff(void);


#endif /* DRIVER_SCHEDULER_H_ */
