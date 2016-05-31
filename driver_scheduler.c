/******************************************************************************
 * @file 	[driver_scheduler.c]
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
/***1 Includes ***************/

#include "driver_scheduler.h"

#include <intrinsics.h>
#include <msp430f5249.h>
#include <msp430f5xx_6xxgeneric.h>
#include <stdbool.h>
#include <timer_b.h>


/***2.1 Internal Marcos ******/

/***2.2 Internal Struct ******/

/***2.3 Internal Variables ***/
uint16 Sch_CpuOnMark = 0;
uint32 Sch_CpuWorkTime = 0;

/***2.4 External Variables ***/
Calendar System_Time =
{ 0 };

/***2.5 Internal Functions ***/
#pragma LOCATION(Isr_Scheduler_TimerB0,0x4E00)
#pragma LOCATION(Isr_Scheduler_Rtc,0x4F00)

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void Isr_Scheduler_TimerB0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(TB0IV, 14))
	{
	case 0: //No interrupt
		break;
	case 2: //CC1, set CPU wake up period.
		TB0CCR1 += SysParam_Schedule.schCpuTickPeriod;
		SysParam_Schedule.cpuTickCount++;
		if (Sch_CpuOnMark == 0)
		{
			Sch_CpuOnMark = TB0R;
		}
		//Turn on CPU
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	case 4: //CC2, set board check period.
		TB0CCR2 += SysParam_Schedule.schBoardCheckPeriod;
		SysParam_Schedule.taskFlagBoardCheck = 1;
		break;
	case 6: //CC3, set Manual Mode period.
		TB0CCR3 += SysParam_Schedule.schManualModePeriod;
		SysParam_Schedule.taskFlagManualMode = 1;
		break;
	case 8: //CC4
		break;
	case 10: //CC5
		break;
	case 12: //CC6
		break;
	case 14: //Overflow , calculate CPU load, uint in %.
		SysParam_Schedule.cpuLoad = Sch_CpuWorkTime * 100 / 0x10000;
		Sch_CpuWorkTime = 0;
		break;
	default:
		break;
	}

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void Isr_Scheduler_Rtc(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) Isr_Scheduler_Rtc (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(RTCIV, 16))
	{
	case 0: //No interrupts
		break;
	case 2: //1 sec , update system time & set test 1Hz flag.
		System_Time = RTC_A_getCalendarTime( RTC_A_BASE);
		SysParam_Schedule.testFlag1Hz = 1;
		break;
	case 4: //1 min
		break;
	case 6: //RTCAIFG
		break;
	case 8: //RT0PSIFG
		break;
	default:
		break;
	}
}

/***2.6 External Functions ***/
/**********************************************************
 * @Brief Sch_init
 * 		Initialize TIMERB0 & RTC as scheduler.
 * 		Set default scheduler peroid.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Sch_init(void)
{
	/* For Non-OS system, use a internal TIMER for task schedule management.
	 * The scheduler control the main() loop by setting different task flags in different peroid.
	 * main()	 :  wait CPU wake -> main(task1,task2,task3...) -> CPU sleep , loop forever.
	 * TIMERB0	 :  CC0 = 1000Hz-> wake up CPU
	 * 				CC1 = 100Hz-> set task1 flag
	 * 				CC2 = 1Hz-> set task2 flag
	 * 				...
	 * TimerB0 peroid setting range is 30.5us(32786Hz) ~ 2s(0.5Hz) .
	 * For long peroid setting (e.g. 1min or 1hour) ,please use RTC module.
	 */

	//Start TimerB0 clock
	Timer_B_initContinuousModeParam b0param =
	{ 0 };
	b0param.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
	b0param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
	b0param.startTimer = true;
	b0param.timerClear = TIMER_B_DO_CLEAR;
	b0param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_ENABLE;
	Timer_B_initContinuousMode(TIMER_B0_BASE, &b0param);

	//Set Compare mode for CC1~CC6
	Timer_B_initCompareModeParam ccparam =
	{ 0 };
	ccparam.compareInterruptEnable = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
	ccparam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
	ccparam.compareValue = 0;

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_2;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_3;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_4;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_5;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_6;
	Timer_B_initCompareMode(TIMER_B0_BASE, &ccparam);

	//Init RTC time 2016/1/1 Fri 00:00:00
	Calendar initTime =
	{ 0, 0, 0, 4, 1, 0, 0x2016 };
	RTC_A_initCalendar(RTC_A_BASE, &initTime, RTC_A_FORMAT_BCD);

	//Start RTC Clock
	RTC_A_startClock(RTC_A_BASE);

	//Enable Sec / Min Interrupt
	RTC_A_clearInterrupt(RTC_A_BASE, RTCRDYIFG + RTCTEVIFG);
	RTC_A_enableInterrupt(RTC_A_BASE, RTCRDYIE + RTCTEVIE);

	//Set default scheduler timer
	SysParam_Schedule.schCpuTickPeriod = 32;	//1ms	1kHz
	SysParam_Schedule.schBoardCheckPeriod = 327;	//10ms	100Hz
	SysParam_Schedule.schManualModePeriod = 547;	//16.7ms 60Hz
	SysParam_Schedule.schSystemOn = 1;	//System On
	SysParam_Schedule.schLocalDimmingOn = 1;	//Local Dimming On

}
/**********************************************************
 * @Brief Sch_init
 * 		Turn off CPU with CPU load mark function.
 * 		Use this at the end of a loop.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Sch_CpuOff(void)
{
	//Sum up CPU on time .
	if (TB0R > Sch_CpuOnMark)
	{
		Sch_CpuWorkTime += TB0R - Sch_CpuOnMark;
	}
	Sch_CpuOnMark = 0;
	//Turn off CPU .
	__bis_SR_register(LPM0_bits);
}

