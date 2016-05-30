/******************************************************************************
 * @file 	[main.c]
 *
 * Main loop of the project with C standard basic function :
 * _system_pre_init() 	: Low level initial before RAM & STACK is initialized.
 * main() 				: main loop for C standard .
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/

#include "app_dpl.h"
#include "app_i2c_interface.h"
#include "app_spi_interface.h"
#include "driver_iw7027.h"
#include "driver_mcu.h"
#include "driver_scheduler.h"
#include "driver_uartdebug.h"
#include "std.h"

int main(void)
{
	//Initialize Driver
	Mcu_init();
	Sch_init();
	Iw7027_init();

	//Main Loop
	while (System_Schedule.schSystemOn)
	{
		/**********************************************************************
		 * Reset Watchdog timer.
		 *********************************************************************/
		WATCHDOG_RESET;

		/**********************************************************************
		 * Task 1:
		 * 		Check board status & set.
		 *********************************************************************/
		if (System_Schedule.taskFlagBoardCheck)
		{
			Mcu_checkBoardStatus(&System_BoardInfo, &System_ErrorParam);
			System_Schedule.taskFlagBoardCheck = 0;
		}

		/**********************************************************************
		 * Task 2:
		 * 		Handle I2C Slave special function.
		 *********************************************************************/
		if (System_Schedule.taskFlagI2c)
		{
			I2cSlave_handleSpecialFunction(I2cSlave_SpecialFuncBuff);
			System_Schedule.taskFlagI2c = 0;
		}

		/**********************************************************************
		 * Task 3:
		 * 		Set duty value for backlight.
		 * Task 3-1:[Local Dimming Mode]
		 * 		SpiSlave_RxBuff -> System_InputDutyBuff -> IW7027.
		 * 		BL should work in this mode for most conditions.
		 * Task 3-2:[Manual Pattern Mode]
		 *  	System_ManualDutyBuff -> IW7027 .
		 *  	By default this buffer is set to 0 for "Mute Function".
		 *  	Also it can be modified by I2C slave to display "Test Pattern".
		 *********************************************************************/
		//Task3-1 : [Local Dimming Mode]
		if (System_Schedule.schLocalDimmingOn)
		{
			Iw7027_updateWorkParams(&System_Iw7027Param);
			if (System_Schedule.taskFlagSpiRx)
			{
				//Check Spi Rx data validation
				System_BoardInfo.bSpiRxValid = SpiSlave_handle(SpiSlave_RxBuff, System_InputDutyBuff, CITRUS_12BIT_78CH);

				//Do Spi Rx data process when format correct.
				if (System_BoardInfo.bSpiRxValid)
				{
					//Dynamic Power limit function
					DPL_caliberateTemp(System_BoardInfo.bTemprature, &System_DplParam);
					DPL_Function(System_InputDutyBuff, System_OutputDutyBuff, &System_DplParam);
					//Enable Tx task .
					System_Schedule.taskFlagSpiTx = 1;
				}

				System_Schedule.taskFlagSpiRx = 0;
			}

			//3.2 Tx data handle
			if (System_Schedule.taskFlagSpiTx)
			{
				// Update duty info to Iw7027 device.
				Iw7027_updateDuty(System_OutputDutyBuff);
				System_Schedule.taskFlagSpiTx = 0;
			}
		}
		//Task3-0 [Manual Pattern Mode]
		else
		{
			if (System_Schedule.taskFlagManualMode)
			{
				Iw7027_updateDuty(System_ManualDutyBuff);
				System_Schedule.taskFlagManualMode = 0;
			}

		}

		//Task4 Test & report
		if (System_Schedule.testFlag1Hz)
		{
			PrintEnter();
			PrintTime(&System_Time);

#if 1
			//Board Info Log
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
			PrintInt(DPL_tempSumDutyMatrix[0] >> 16);
			PrintInt(DPL_tempSumDutyMatrix[0] & 0xFFFF);
			PrintString(" Limit: ");
			PrintInt(DPL_tempDutyLimitTable[0]);

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

