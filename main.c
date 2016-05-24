#include "driver.h"
#include "app.h"
#include "string.h"

#pragma location=0xF000
const uint32_t ISP_PW = 0x20140217 ;

#pragma location=0xF200
int _system_pre_init(void)
{
	/**************************************************************
	 * Insert low-level initializations here.
	 * System will run _system_pre_init before RAM is initialized.
	 * Return: 0 to omit initialization
	 * Return: 1 to do RAM initialization.
	 * Then jump to main()
	 * Disable Watchdog timer to prevent reset during long variable
	 * initialization sequences.
	 **************************************************************/

	WDTCTL = WDTPW + WDTHOLD;// Stop WDT
	__disable_interrupt();

	//If pass word not correct , init I2C slave in ISP mode.
	if(ISP_PW != 0x20140217)
	{
		//P4.6 = ERROR_OUT , P4.7 = LED_G , P3.0 = SDA ,P3.1 = SCL
		P4OUT |= BIT7 + BIT6 ;
		P4DIR |= BIT7 + BIT6 ;
		P3SEL |= BIT0 + BIT1 ;

		//Initialize I2C slave in ISP mode address
		UCB0CTL1 |= UCSWRST;
		UCB0CTL0 = UCMODE_3 + UCSYNC;
		UCB0I2COA = BOARD_I2C_ADDRESS_ISP/2;
		UCB0CTL1 &= ~UCSWRST;
		UCB0IE |= UCRXIE + UCTXIE + UCSTTIE + UCSTPIE;

		//Enable Interrupt & Turn Off CPU.
		__enable_interrupt();
		__bis_SR_register(LPM0_bits);
	}

	/*==================================*/
	/* Choose if segment initialization */
	/* should be done or not. */
	/*==================================*/
 	 return 1;
}

int main(void) {
	//Initialize Driver
    Mcu_init();
    Sch_init();
    Iw7027_init(Iw7027_DefaultRegMap_70XU30A_78CH);
	//Initialize application
	I2cSlave_initMap(I2cSlave_Map);

    //Main Loop
    while(System_Schedule.schSystemOn)
    {
    	WDT_A_resetTimer(WDT_A_BASE);

    	//Task1 : Board Check
    	if(System_Schedule.taskFlagBoardCheck)
    	{
    		Mcu_checkBoardStatus(&System_BoardInfo , &System_ErrorParam);
    		System_Schedule.taskFlagBoardCheck = 0;
    	}

    	//Task2 : Handle I2C control cmd.
    	if(System_Schedule.taskFlagI2c)
    	{
    		I2cSlave_handleMap(I2cSlave_Map);
    		System_Schedule.taskFlagI2c = 0;
    	}

    	//Task3 	: Set backlight duty ,either in [Local Dimming Mode] or [Manual Mode] ,selected by schLocalDimmingOn.
    	//	Task3-1 : [Local Dimming Mode] ,update duty matrix received from spi slave.
    	//  Task3-0 : [Manual Mode] ,update duty matrix from System_ManualDutyBuff .
    	//			  By default the buffer is set to 0x00 , use as "BL Mute function" .
    	//			  Also , the buffer can be manually set by i2c interface , use as "Global Dimming Mode" or "Test Pattern Mode"

    	//Task3-1 : [Local Dimming Mode]
    	if(System_Schedule.schLocalDimmingOn)
    	{

        	if(System_Schedule.taskFlagSpiRx)
        	{
        		//Check Spi Rx data validation
        		System_BoardInfo.bSpiRxValid = SpiSlave_handle ( SpiSlave_RxBuff , System_InputDutyBuff , CITRUS_12BIT_78CH );

        		//Do Spi Rx data process when format correct.
        		if ( System_BoardInfo.bSpiRxValid )
        		{
        			//Dynamic Power limit function
        			DPL_TemperatureCalibration(System_BoardInfo.bTemprature , &System_DplParam );
        			DPL_Function( System_InputDutyBuff , System_OutputDutyBuff , &System_DplParam );
        			//Enable Tx task .
        			System_Schedule.taskFlagSpiTx = 1;
        		}

        		System_Schedule.taskFlagSpiRx = 0;
        	}

        	//3.2 Tx data handle
        	if(System_Schedule.taskFlagSpiTx)
        	{
        		// Update duty info to Iw7027 device.
        		Iw7027_updateDuty ( System_OutputDutyBuff , Iw7027_LedSortMap_70XU30A_78CH);
        		System_Schedule.taskFlagSpiTx = 0 ;
        	}
    	}
    	//Task3-0 [Manual Mode]
    	else
    	{
    		if(System_Schedule.taskFlagManualMode)
    		{
                Iw7027_updateDuty( (uint16_t*)System_ManualDutyBuff , Iw7027_LedSortMap_70XU30A_78CH);
                System_Schedule.taskFlagManualMode = 0 ;
    		}

    	}

    	//Task4 Test & report
    	if(System_Schedule.testFlag1Hz)
    	{
    		PrintEnter();
    		PrintTime(&System_Time);
    		System_Schedule.schLocalDimmingOn = 1;

#if 0
    		//Board Info Log
    		PrintString("boardinfo: ");
    		PrintArray((uint8_t *)&System_BoardInfo,sizeof(System_BoardInfo));
    		PrintEnter();
    		PrintString("CPU Locd = ");
    		PrintCharBCD(System_Schedule.cpuLoad);
    		PrintString(" % \r\n");
#endif
#if 0
    		//DPL log
    		PrintString("Input: ");
    		PrintInt(System_InputDutyBuff[0]);
    		PrintString(" Onput: ");
    		PrintInt(System_OutputDutyBuff[0]);
    		PrintString(" Sum: ");
    		PrintInt(DPL_tempSumDutyMatrix[0]);
    		PrintString(" Limit: ");
    		PrintInt(System_DplParam.dplLdDutyLimitTable[0]);

    		PrintEnter();
#endif
    		TOGGLE_LED_G;
    		System_Schedule.testFlag1Hz = 0;
    	}

    	//Finish all task , turn off cpu.
    	Sch_CpuOff();
    } //End of while(System_Schedule.schSystemOn)


    //Reboot
    Mcu_reset();


}





