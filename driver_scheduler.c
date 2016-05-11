#include "driver_scheduler.h"
#include "app_dpl.h"

#ifndef ACLK_F
#define ACLK_F 		(32768)
#endif

extern DPL_Prama System_DplParam;

uint8_t Sch_init(void)
{
	/* For Non-OS system, use TimerB0 & RTC module as CPU & task schduler .
	 * By using Timer ISR , we set task flags & run mian.
	 * When all task is done , CPU is turned off, wait until next Timer ISR.
	 * RTC & TimerB0 cover different task peroid,
	 * For peroid <2s task : use Timer B0
	 * For peroid >1s task : use RTC
	 */

	//Start TimerB0 clock
	Timer_B_initContinuousModeParam b0param = {0};
	b0param.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
	b0param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
	b0param.startTimer = true;
	b0param.timerClear = TIMER_B_DO_CLEAR;
	b0param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_ENABLE;
	Timer_B_initContinuousMode(TIMER_B0_BASE,&b0param);

	//Set Compare mode for CC1~CC6
	Timer_B_initCompareModeParam ccparam ={0};
	ccparam.compareInterruptEnable = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
	ccparam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
	ccparam.compareValue = 0;

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_1;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_2;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_3;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_4;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_5;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	ccparam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_6;
	Timer_B_initCompareMode(TIMER_B0_BASE , &ccparam);

	//Init RTC time 2016/1/1 Fri 00:00:00
	Calendar initTime = {0,0,0,4,1,0,0x2016};
    RTC_A_initCalendar(RTC_A_BASE,&initTime,RTC_A_FORMAT_BCD);

    //Enable Sec / Min Interrupt
    RTC_A_clearInterrupt(RTC_A_BASE,RTCRDYIFG + RTCTEVIFG);
    RTC_A_enableInterrupt(RTC_A_BASE,RTCRDYIE + RTCTEVIE);

    //Start RTC Clock
    RTC_A_startClock(RTC_A_BASE);

	return STATUS_SUCCESS;
}

void Sch_CpuOff(void)
{
	//Calculate CPU on time
	if(TB0R > Sch_CpuOnMark)
	{
		Sch_CpuWorkTime += TB0R - Sch_CpuOnMark ;
	}
	Sch_CpuOnMark = 0;
	//Turn off CPU
	__bis_SR_register(LPM0_bits);
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void Sch_ISR_TimerB0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_B1_VECTOR))) TIMER0_B1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(TB0IV,14))
    {
    	case 0://No interrupt
    		break;
    	case 2://CC1
    		//CPU wake up tick = 1000Hz / 1ms
    		TB0CCR1 += System_Schedule.schCpuTickPeriod ;
    		System_Schedule.cpuTickCount ++;

    		if(Sch_CpuOnMark == 0)
    		{
    			Sch_CpuOnMark = TB0R;
    		}
    		//Turn on CPU
    		__bic_SR_register_on_exit(LPM0_bits);

    		break;
    	case 4://CC2
    		//GPIO Check tick = 100Hz / 10ms
    		TB0CCR2 += System_Schedule.schGpioCheckTPeriod ;
    		System_Schedule.taskFlagGpioCheck = 1;
    		break;
    	case 6://CC3
    		//Error Handle tick = 10Hz / 100ms
    		TB0CCR3 += System_Schedule.schErrorHandlePeriod ;
    		System_Schedule.taskFlagErrorHandle = 1;
    		break;
    	case 8://CC4
    		break;
    	case 10://CC5
    		break;
    	case 12://CC6
    		break;
    	case 14://Overflow
    		System_Schedule.cpuLoad = (Sch_CpuWorkTime>>8 / 100)>>8 ;
    		Sch_CpuWorkTime = 0;
    		break;
    	default:
    		break;
    }

}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void Sch_ISR_RTC(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTC_A_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV,16))
    {
        case 0: //No interrupts
        	break;
        case 2: //1 sec
        	System_Time = RTC_A_getCalendarTime( RTC_A_BASE );
        	System_DplParam.dplStartSample = 1;
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
