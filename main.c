/******************************************************************************
 * @file 	[main.c]
 *
 * Main loop of the project with C standard basic function :
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

extern void Mcu_Boot(void);

int main(void)
{
	//Initialize Driver
	Mcu_init();
	Sch_init();
	Iw7027_init();

	//Main Loop
	while (SysParam_Schedule.fSystemResetN)
	{
		/**********************************************************************
		 * Reset Watchdog timer.
		 *********************************************************************/
		HW_WATCHDOG_RESET;

		/**********************************************************************
		 * Task 1:
		 * 		Check board status & set.
		 *********************************************************************/
		if (SysParam_Schedule.fTaskFlagGpioCheck)
		{
			Mcu_checkBoardStatus(&SysParam_BoardInfo, &SysParam_Error);
			SysParam_Schedule.fTaskFlagGpioCheck = 0;
		}

		/**********************************************************************
		 * Task 2:
		 * 		Handle I2C Slave special function.
		 *********************************************************************/
		if (SysParam_Schedule.fTaskFlagI2c)
		{
			I2cSlave_handleSpecialFunction(HwBuf_I2cSlave);
			SysParam_Schedule.fTaskFlagI2c = 0;
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
		if (SysParam_Schedule.fLocalDimmingOn)
		{
			Iw7027_updateWorkParams(&SysParam_Iw7027);
			if (SysParam_Schedule.fTaskFlagSpiRx)
			{
				//Check Spi Rx data validation
				SysParam_BoardInfo.fSpiDataValid = SpiSlave_handle(HwBuf_SpiSlaveRx, HwBuf_InputDuty, CITRUS_12BIT_78CH);

				//Do Spi Rx data process when format correct.
				if (SysParam_BoardInfo.fSpiDataValid)
				{
					//Dynamic Power limit function
					DPL_caliberateTemp(SysParam_BoardInfo.su8McuTemperature, &SysParam_Dpl);
					DPL_Function(HwBuf_InputDuty, HwBuf_OutputDuty, &SysParam_Dpl);
					//Enable Tx task .
					SysParam_Schedule.fTaskFlagSpiTx = 1;
				}

				SysParam_Schedule.fTaskFlagSpiRx = 0;
			}

			//3.2 Tx data handle
			if (SysParam_Schedule.fTaskFlagSpiTx)
			{
				// Update duty info to Iw7027 device.
				Iw7027_updateDuty(HwBuf_OutputDuty);
				SysParam_Schedule.fTaskFlagSpiTx = 0;
			}
		}
		//Task3-0 [Test Pattern Mode]
		else
		{
			if (SysParam_Schedule.fTaskFlagTestMode)
			{
				Iw7027_updateDuty(HwBuf_TestDuty);
				SysParam_Schedule.fTaskFlagTestMode = 0;
			}

		}

		//Task4 Test & report
		if (SysParam_Schedule.fTestFlag1Hz)
		{
			PrintEnter();
			PrintTime(&SysParam_Time);

#if 1
			//Board Info Log
			PrintEnter();
			PrintString("CPU Locd = ");
			PrintCharBCD(SysParam_Schedule.u8CpuLoad);
			PrintString(" % \r\n");
#endif
#if 1
			//DPL log
			PrintString("Input: ");
			PrintInt(HwBuf_InputDuty[0]);
			PrintString(" Onput: ");
			PrintInt(HwBuf_OutputDuty[0]);
			PrintString(" Sum: ");
			PrintInt(DPL_tempSumDutyMatrix[0] >> 16);
			PrintInt(DPL_tempSumDutyMatrix[0] & 0xFFFF);
			PrintString(" Limit: ");
			PrintInt(DPL_tempDutyLimitTable[0]);

			PrintEnter();
#endif
			HW_TOGGLE_LED_G;
			SysParam_Schedule.fTestFlag1Hz = 0;
		}

		//Finish all task , turn off cpu.
		Sch_CpuOff();
	} //End of while(SysParam_Schedule.schSystemOn)

	//Reboot
	Mcu_reset();

}

