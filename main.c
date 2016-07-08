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

#include "api_dpl.h"
#include "drv_i2c_slave.h"
#include "drv_iw7027.h"
#include "drv_spi_slave.h"
#include "drv_uart.h"
#include "hal.h"

#pragma LOCATION(main,ISP_MAIN_ADDRESS)
int main(void)
{
	//Initialize Driver
	Hal_Mcu_init();
	Hal_Sch_init();
	Iw7027_init();

	//Main Loop
	while (tHal_CpuScheduler.fSystemResetN)
	{
		/**********************************************************************
		 * Reset Watchdog timer.
		 *********************************************************************/
		HAL_WATCHDOG_RESET;

		/**********************************************************************
		 * Task 1:
		 * 		Check board status & set.
		 *********************************************************************/
		if (tHal_CpuScheduler.fTaskFlagGpioCheck)
		{
			Hal_Mcu_checkBoardStatus(&tHal_BoardInfo, &tHal_BoardErrorParam);
			tHal_CpuScheduler.fTaskFlagGpioCheck = 0;
		}

		/**********************************************************************
		 * Task 2:
		 * 		Handle I2C Slave special function.
		 *********************************************************************/
		if (tHal_CpuScheduler.fTaskFlagI2c)
		{
			I2cSlave_handleSpecialFunction(u8Hal_Buf_I2cSlave);
			tHal_CpuScheduler.fTaskFlagI2c = 0;
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
		if (tHal_CpuScheduler.fLocalDimmingOn)
		{
			Iw7027_updateWorkParams(&tDrv_Iw7027Param);
			if (tHal_CpuScheduler.fTaskFlagSpiRx)
			{
				//Check Spi Rx data validation
				tHal_BoardInfo.fSpiDataValid = SpiSlave_getDuty(u8Hal_Buf_SpiSlaveRx, u16Hal_Buf_InputDuty, MFC11_12BIT_120CH);

				//Do Spi Rx data process when format correct.
				if (tHal_BoardInfo.fSpiDataValid)
				{
					//Dynamic Power limit function
					Dpl_caliberateTemp(tHal_BoardInfo.su8McuTemperature, &tDpl_Param);
					Dpl_process(u16Hal_Buf_InputDuty, u16Hal_Buf_OutputDuty, &tDpl_Param);
					//Enable Tx task .
					tHal_CpuScheduler.fTaskFlagSpiTx = 1;
				}

				tHal_CpuScheduler.fTaskFlagSpiRx = 0;
			}

			//3.2 Tx data handle
			if (tHal_CpuScheduler.fTaskFlagSpiTx)
			{
				// Update duty info to Iw7027 device.
				Iw7027_updateDuty(u16Hal_Buf_OutputDuty);
				tHal_CpuScheduler.fTaskFlagSpiTx = 0;
			}
		}
		//Task3-0 [Test Pattern Mode]
		else
		{
			if (tHal_CpuScheduler.fTaskFlagTestMode)
			{
				Iw7027_updateDuty(u16Hal_Buf_TestDuty);
				tHal_CpuScheduler.fTaskFlagTestMode = 0;
			}

		}

		//Task4 Test command & report
		if (tHal_CpuScheduler.fTestFlag1Hz)
		{
			//Toggle LED as Mcu Heart Beat.
			Gpio_toggle(GPIO_LED_G);

			tHal_CpuScheduler.fLocalDimmingOn = 0;
			static uint8 i;

			u16Hal_Buf_TestDuty[i % 128] = 0xA0;
			i++;

#if 1
			PrintTime(&tHal_Time);
			PrintString("CPU Locd = ");
			PrintCharBCD(tHal_CpuScheduler.u8CpuLoad);
			PrintString(" % \r\n");
#endif

#if 1
			//Dpl log
			PrintString("Input: ");
			PrintInt(u16Hal_Buf_InputDuty[0]);
			PrintString(" Onput: ");
			PrintInt(u16Hal_Buf_OutputDuty[0]);
			PrintString(" Sum: ");
			PrintInt(u32Dpl_tempSumDutyMatrix[0] >> 16);
			PrintInt(u32Dpl_tempSumDutyMatrix[0] & 0xFFFF);
			PrintString(" Limit: ");
			PrintInt(u16Dpl_tempDutyLimitTable[0]);

			PrintEnter();
#endif

			tHal_CpuScheduler.fTestFlag1Hz = 0;
		}

		//Finish all task , turn off cpu.
		Hal_Sch_CpuOff();
	} //End of while(tHal_CpuScheduler.schSystemOn)

//Reboot
	Hal_Mcu_reset();

}

