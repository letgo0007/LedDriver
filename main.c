#include "driver.h"
#include "app.h"
#include "string.h"

uint8_t test[60];

int _system_pre_init(void)
{
 /* Insert your low-level initializations here */

/* Disable Watchdog timer to prevent reset during */
 /* long variable initialization sequences. */
 WDTCTL = WDTPW | WDTHOLD;

 /*==================================*/
 /* Choose if segment initialization */
 /* should be done or not. */
 /* Return: 0 to omit initialization */
 /* 1 to run initialization */
 /*==================================*/
 return 1;
}

int main(void) {
	//MCU & Scheduler initial
    Mcu_init();
    Sch_init();
    //Init IW7027
    Iw7027_init(Iw7027_DefaultRegMap_70XU30A_78CH);
    //Init I2C map
	I2cSlave_initMap(I2cSlave_Map);
	__enable_interrupt();

    //Enter Main Loop
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
        		System_BoardInfo.boardSpiRxValid = SpiSlave_handle ( SpiSlave_RxBuff , System_InputDutyBuff , CITRUS_12BIT_78CH );

        		//Do Spi Rx data process when format correct.
        		if ( System_BoardInfo.boardSpiRxValid )
        		{
        			//Power limit function
        			DPL_TemperatureCalibration(System_BoardInfo.boardTemprature , &System_DplParam );
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

    		PrintTime(&System_Time);
    		System_Schedule.schLocalDimmingOn = 1;

#if 1
    		//Board Info Log
    		PrintString("boardinfo: ");
    		PrintArray((uint8_t *)&System_BoardInfo,sizeof(System_BoardInfo));
    		PrintEnter();
    		PrintString("CPU Locd = ");
    		PrintCharBCD(System_Schedule.cpuLoad);
    		PrintString(" % \r\n");
#endif
#if 1
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


    //MainLoop Stop ,reboot MCU .
    Mcu_reset();

}





