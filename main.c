#include "driver.h"
#include "app.h"

int main(void) {
	//MCU & Scheduler initial
    Mcu_init();
    Sch_init();
    //Init IW7027
    Iw7027_init(Iw7027_DefaultRegMap_70XU30A_78CH);
    //Init Parameters
	I2cSlave_initMap(I2cSlave_Map);

    //Enable interrupt;
    _EINT();

    while(System_Schedule.schSystemOn)
    {
    	WDT_A_resetTimer(WDT_A_BASE);

    	//1 Check GPIO & ADC status
    	if(System_Schedule.taskFlagGpioCheck)
    	{
    		Board_getBoardInfo(&System_BoardInfo);
    		System_Schedule.taskFlagGpioCheck = 0;
    	}

    	//2 Handle I2C event
    	if(System_Schedule.taskFlagI2c)
    	{
    		I2cSlave_handleMap(I2cSlave_Map);
    		System_Schedule.taskFlagI2c = 0;
    	}

    	//3 Local Dimming Mode
    	if(System_Schedule.schLocalDimmingOn)
    	{
    		//3.1 Rx data handle
        	if(System_Schedule.taskFlagSpiRx)
        	{
        		//Check Spi Rx data validation
        		System_BoardInfo.boardSpiRxValid = SpiSlave_handle ( SpiSlave_RxBuff , System_InputDutyBuff , CITRUS_12BIT_78CH );

        		//Do Spi Rx data process when format correct.
        		if ( System_BoardInfo.boardSpiRxValid )
        		{
        			//Power limit function
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
        		System_Schedule.taskFlagSpiTx = 0;
        	}
    	}
    	//Manual Mode , update Manual duty buff content to IW7027
    	else
    	{
    		//Lower cpu tick to 60Hz;
//    		System_Schedule.schCpuTickPeriod = UCS_getACLK()/60;
            Iw7027_updateDuty( (uint16_t*)System_ManualDutyBuff , Iw7027_LedSortMap_70XU30A_78CH);
    	}

    	//4 Error handle & report .
    	if(System_Schedule.taskFlagErrorHandle)
    	{

    		PrintTime(&System_Time);

    		PrintString("boardinfo: ");
    		PrintArray((uint8_t *)&System_BoardInfo,sizeof(System_BoardInfo));
    		PrintEnter();

    		//System_Schedule.schLocalDimmingOn = !(System_Schedule.schLocalDimmingOn);

    		TOGGLE_LED_G;
    		PrintString("CPU Locd = ");
    		PrintCharBCD(System_Schedule.cpuLoad);
    		PrintString(" % \r\n");

    		System_Schedule.taskFlagErrorHandle = 0;
    	}

    	//Finish all task , turn off cpu.
    	Sch_CpuOff();
    }


    //System main loop can only be stoped by i2c
    Mcu_reset();

}



