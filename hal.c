/******************************************************************************
 * @file 	[hal.c]
 *
 * Hardware Abstract Layer for LED Driver Board.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] 	[TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL			Init	Initial Version
 * 2		20150707 Yang Zhifang	65SX970A	New		Add new branch
 *
 *****************************************************************************/

/***1 Includes ***************************************************************/

#include "hal.h"

#include "api_dpl.h"
#include "drv_iw7027.h"
#include "drv_uart.h"

/***2.1 Internal Marcos ******************************************************/
//XT1 frequency, if no external XT1,set to 0
#define HAL_XT1_F						(0)
//XT2 frequency, if no external XT2,set to 0
#define HAL_XT2_F						(0)
//High speed Sub_System_Clock
#define HAL_SMCLK_F						(HAL_CPU_F)
//Low speed Assist_Clock
#define HAL_ACLK_F						(32768)
//I2C Slave addess for [NORMAL] mode ,7bit mode
#define HAL_I2C_SLAVE_ADD_NORMAL		(0x14)
//I2C buffer start address for [NORMAL] mode .
#define HAL_I2C_SLAVE_BUFF_OFFSET		(0x4000)
//SPI master speed (unit in Hz)
#define HAL_SPI_MASTER_SPEED			(4000000)
//Bound Rate of UART
#define HAL_UART_BAUDRATE				(115200)
//Temperature Sensor Calibration value for 30C
#define HAL_ADCCAL_15V_30C  			*((uint16 *)0x1A1A)
//Temperature Sensor Calibration value for 85C
#define HAL_ADCCAL_15V_85C  			*((uint16 *)0x1A1C)

/***2.2 Internal Struct ********************************************************/

/***2.3 Internal Variables *****************************************************/

//Cpu work time buffer to calculate cpu load.
uint16 u16Hal_SchCpuOnMark = 0;
uint32 u32Hal_SchCpuWorkTime = 0;

/***2.4 External Variables ***/

//Local Dimming duty information buffer
uint16 u16Hal_Buf_InputDuty[128] =
{ 0 };
uint16 u16Hal_Buf_OutputDuty[128] =
{ 0 };
uint16 u16Hal_Buf_TestDuty[128] =
{ 0 };
//Spi Slave hardware buffer
uint8 u8Hal_Buf_SpiSlaveRx[256] =
{ 0 };
//Uart rx hardware buffer.
uint8 u8Hal_Buf_UartRx[256] =
{ 0 };

/******************************************************************************
 * Set I2C Slave Map Struct
 * Put Variables those need I2C access to the RAM address of
 * HAL_I2C_SLAVE_BUFF_OFFSET ~ HAL_I2C_SLAVE_BUFF_OFFSET + 0xFF .
 * Using the #pragma location = ADDRESS compiler command to appoint address.
 * Refer to TI compiler #pragma cmd list file :
 * http://www.ti.com/lit/ug/slau132l/slau132l.pdf
 *
 * Name            			origin    	length
 * ----------------------  	--------  	---------
 * tHal_CpuScheduler		0x00		0x20
 * tHal_BoardInfo			0x20		0x08
 * tHal_Time				0x28		0x08
 * tHal_BoardErrorParam		0x30		0x10
 * tDrv_Iw7027Param			0x40		0x30
 * tDpl_Param				0x70		0x30
 * u8Hal_Buf_I2cSlave		0xA0		0x50
 * u32Hal_Buf_SoftVersion	0xF0		0x08
 * u32Hal_Buf_IspPw			0xF8		0x08
 * ----------------------  	--------  	---------
 * 							total		0x100
 ******************************************************************************/
#pragma LOCATION(tHal_CpuScheduler , HAL_I2C_SLAVE_BUFF_OFFSET + 0x00)
#pragma LOCATION(tHal_BoardInfo , HAL_I2C_SLAVE_BUFF_OFFSET + 0x20)
#pragma LOCATION(tHal_Time , HAL_I2C_SLAVE_BUFF_OFFSET + 0x28)
#pragma LOCATION(tHal_BoardErrorParam , HAL_I2C_SLAVE_BUFF_OFFSET + 0x30)
#pragma LOCATION(tDrv_Iw7027Param , HAL_I2C_SLAVE_BUFF_OFFSET + 0x40)
#pragma LOCATION(tDpl_Param , HAL_I2C_SLAVE_BUFF_OFFSET + 0x70)
#pragma LOCATION(u8Hal_Buf_I2cSlave , HAL_I2C_SLAVE_BUFF_OFFSET + 0xA0)
#pragma LOCATION(u32Hal_Buf_SoftVersion , HAL_I2C_SLAVE_BUFF_OFFSET + 0xF0)
#pragma LOCATION(u32Hal_Buf_IspPw , HAL_I2C_SLAVE_BUFF_OFFSET + 0xF8)

Hal_CpuScheduler_t tHal_CpuScheduler =
{ 0 };

Hal_BoardInfo_t tHal_BoardInfo =
{ 0 };

Hal_Time tHal_Time =
{ 0 };

Hal_BoardErrorParam_t tHal_BoardErrorParam =
{ 0 };

Drv_Iw7027Param_t tDrv_Iw7027Param =
{ 0 };

Dpl_Prama_t tDpl_Param =
{ 0 };

uint8 u8Hal_Buf_I2cSlave[0x50] =
{ 0 };

volatile uint64 u32Hal_Buf_SoftVersion =
{ HAL_VERSION };

volatile uint64 u32Hal_Buf_IspPw =
{ 0 };

/***2.5 Internal Functions ***/

/******************************************************************************
 * Set Flash Structure
 * The system with ISP(in system programming) function define the flash into
 * [Boot] 		: Reset interrupt service & ISP program. This section should
 * 				NOT be modified in ISP progress.
 * [App-ISR] 	: Interrupt Service Routine , must place at address <0xFFFF
 * 				Interrupt Vector of MSP430 only support 16bit address.
 * [App-VECTOR] : Interrupt Vector defines jump address for each hardware
 * 				interrupt . Note that the "Reset Vector" @ 0xFFFE~0xFFFF
 * 				must always set to 0x4400(_c_int00_noargs_noexit) to ensure
 * 				system can reset normally.
 * [App-FUNC] 	: Main program area.
 * [App-CONST] 	: Const table area.
 *
 * The Flash address of each section is set as below :
 *
 * [Boot]	     			origin    	length		Remark
 * ----------------------  	--------  	---------	----------
 * _c_int00_noargs_noexit	0x4400		0x001a		@C/C++ complier lib
 * __TI_ISR_TRAP			0x441a		0x0008		@C/C++ complier lib
 * _system_pre_init			0x4480
 * ----------------------  	--------  	---------	----------
 * 							total		0x400
 *
 * [App-ISR]		    	origin    	length		Remark
 * ----------------------  	--------  	---------	----------
 * Hal_Isr_Gpio_P1			0x4800
 * Hal_Isr_SpiSlave_Cs		0x4900
 * Hal_Isr_I2cSlave			0x4A00
 * Hal_Isr_Uart				0x4D00
 * Hal_Isr_SchTimerB0		0x4E00
 * Hal_Isr_SchRtc			0x4F00
 * ----------------------  	--------  	---------	----------
 * 							total		0x800
 *
 * [App-VECTOR]			  	origin    	length		Remark
 * ----------------------  	--------  	---------	----------
 * 							0xFE00		0x200
 * ----------------------  	--------  	---------	----------
 * 							total		T.B.D.
 *
 * [App-FUNC]  			  	origin    	length		Remark
 * ----------------------  	--------  	---------	----------
 * main()					0x10000
 * ----------------------  	--------  	---------	----------
 * 							total		TBD
 *
 * [App-CONST]     			origin    	length		Remark
 * ----------------------  	--------  	---------	----------
 * 							0x20000
 * ----------------------  	--------  	---------	----------
 * 							total		TBD
 *
 ******************************************************************************/
#pragma LOCATION(Hal_Isr_Gpio_P1,0x5000)
#pragma LOCATION(Hal_Isr_SpiSlave_Cs,0x5200)
#pragma LOCATION(Hal_Isr_I2cSlave,0x5400)
#pragma LOCATION(Hal_Isr_Uart,0x5600)
#pragma LOCATION(Hal_Isr_SchTimerB0,0x5800)
#pragma LOCATION(Hal_Isr_SchRtc,0x5A00)

/**********************************************************
 * @Brief Hal_Isr_SchTimerB0
 * 		TimerB0 interrupt service.
 * 		Use as Cpu scheduler.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt void Hal_Isr_SchTimerB0(void)
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
		TB0CCR1 += tHal_CpuScheduler.u16CpuTickPeriod;
		tHal_CpuScheduler.u16CpuTickCount++;
		if (u16Hal_SchCpuOnMark == 0)
		{
			u16Hal_SchCpuOnMark = TB0R;
		}
		//Turn on CPU
		__bic_SR_register_on_exit(LPM0_bits);
		break;
	case 4: //CC2, set board check period.
		TB0CCR2 += tHal_CpuScheduler.u16GpioCheckPeriod;
		tHal_CpuScheduler.fTaskFlagGpioCheck = 1;
		break;
	case 6: //CC3, set Manual Mode period.
		TB0CCR3 += tHal_CpuScheduler.u16TestModePeriod;
		tHal_CpuScheduler.fTaskFlagTestMode = 1;
		break;
	case 8: //CC4
		break;
	case 10: //CC5
		break;
	case 12: //CC6
		break;
	case 14: //Overflow , calculate CPU load, uint in %.
		tHal_CpuScheduler.u8CpuLoad = u32Hal_SchCpuWorkTime * 100 / 0x10000;
		u32Hal_SchCpuWorkTime = 0;
		break;
	default:
		break;
	}

}
/**********************************************************
 * @Brief Hal_Isr_SchRtc
 * 		RTC (Real time clock) interrupt service.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=RTC_VECTOR
__interrupt void Hal_Isr_SchRtc(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) Hal_Isr_SchRtc (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(RTCIV, 16))
	{
	case 0: //No interrupts
		break;
	case 2: //1 sec , update system time & set test 1Hz flag.
		tHal_Time = RTC_A_getCalendarTime( RTC_A_BASE);
		tHal_CpuScheduler.fTestFlag1Hz = 1;
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

/**********************************************************
 * @Brief Hal_Isr_Gpio_P1
 * 		GPIO_P1 interrupt service rotine.
 * 		Active for STB_HW falling edge interrupt , quick mute & reset function.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Hal_Isr_Gpio_P1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Hal_Isr_Gpio_P1 (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(P1IV, 14))
	{
	case 0:		//No interrupt
		break;
	case 2:		//P1.0
		break;
	case 4:		//P1.1, STB falling edge ISR.
#if UART_DEBUG_ON
		PrintString("\e[31m***STB falling edge detect***\r\n\e[30m");
#endif
		DELAY_US(500);
		if (!HAL_GET_STB_IN)
		{
			//Mute Backlight
			Hal_Mem_set8((uint32) &u16Hal_Buf_TestDuty, 0x00, sizeof(u16Hal_Buf_TestDuty));
			Iw7027_updateDuty((uint16*) u16Hal_Buf_TestDuty);

			//Hold CPU till STB get High , if STB = L for 1s, reset Mcu.
			uint16 timeout = 0;
			while (!HAL_GET_STB_IN)
			{
				DELAY_MS(10);
				timeout++;
				if (timeout > 100)
				{
					Hal_Mcu_reset();
				}
			}
		}

		P1IFG &= ~BIT1;

		break;
	case 6:		//P1.2
		break;
	case 8:		//P1.3
		break;
	case 10:	//P1.4
		break;
	case 12:	//P1.5
		break;
	case 14:	//P1.6
		break;
	case 16:	//P1.7
		break;
	default:
		break;
	}

}

/**********************************************************
 * @Brief Hal_Isr_SpiSlave_Cs
 * 		Timer_A0 Interrupt service.
 * 		Use TA0.2 as Spi Slave Cs pin Both Edge Trigger.
 *
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Hal_Isr_SpiSlave_Cs(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Hal_Isr_SpiSlave_Cs (void)
#else
#error Compiler not supported!
#endif
{
	//Edge count to calculate SpiSlave cs frequency
	static uint8 SpiSlave_CsEdgeCount;

	switch (__even_in_range(TA0IV, 14))
	{
	case 0: // No interrupt
		break;
	case 2: // TA0.1
		break;
	case 4: // TA0.2 ,PWM1/SPI Slave CS
		//Falling edge = START of frame
		Hal_SpiSlave_stopRx();
		tHal_CpuScheduler.fTaskFlagSpiRx = 1;
		Hal_SpiSlave_startRx();
		//Sync PWM out
		Hal_PwmOut_sync();
		//Calulate freq
		SpiSlave_CsEdgeCount++;
		Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
		break;
	case 6: //TA0.3
		break;
	case 8: //TA0.4
		break;
	case 10: //TA0.5
		break;
	case 12: // reserved
		break;
	case 14: // overflow
		//Calculate Spi Rx CS frequencey
		tHal_BoardInfo.u8SpiRxFreq = (SpiSlave_CsEdgeCount - 1) / 2;
		SpiSlave_CsEdgeCount = 0;
		break;
	default:
		break;
	}
}

/**********************************************************
 * @Brief Hal_Isr_I2cSlave
 * 		USCI_B0 I2C mode interrupt service.
 * 		Buffer I2C Slave data to certain RAM address.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void Hal_Isr_I2cSlave(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) Hal_Isr_I2cSlave (void)
#else
#error Compiler not supported!
#endif
{
	static uint32 op_add;
	static uint32 rxcount;
	static uint32 txcount;

	switch (__even_in_range(UCB0IV, 12))
	{
	case 0:	    	// Vector  0: No interrupts
		break;
	case 2:	    	// Vector  2: ALIFG
		break;
	case 4:	    	// Vector  4: NACKIFG
		break;
	case 6:	    	// Vector  6: STTIFG
		rxcount = 0;
		txcount = 0;
		break;
	case 8:	    	// Vector  8: STPIFG
		//Set I2C task flag.
		tHal_CpuScheduler.fTaskFlagI2c = 1;
		//ISP mode entrance ,check pass word.
		if (u32Hal_Buf_IspPw == 0x20140217)
		{
			//Stop internal Timer
			TB0CTL &= ~MC_3;
			TA0CTL &= ~MC_3;
			RTCCTL01_H |= RTCHOLD_H;

			//Erase ISP Flag
			unsigned long * flash_ptr;
			flash_ptr = (unsigned long *) ISP_EXIT_FLAG_ADDRESS;
			FCTL3 = FWKEY;
			FCTL1 = FWKEY + ERASE;
			*flash_ptr = 0;
			FCTL1 = FWKEY;
			FCTL3 = FWKEY + LOCK;

			//Reboot to ISP
			Hal_Mcu_reset();
		}
		break;
	case 10:	    	// Vector 10: RXIFG
		if (rxcount == 0)
		{
			op_add = UCB0RXBUF;
		}
		else
		{
			HAL_REG8(HAL_I2C_SLAVE_BUFF_OFFSET + op_add + rxcount - 1) = UCB0RXBUF;
		}
		rxcount++;
		break;
	case 12:	    	// Vector 12: TXIFG
		UCB0TXBUF = HAL_REG8(HAL_I2C_SLAVE_BUFF_OFFSET + op_add + txcount);
		txcount++;
		break;
	default:
		break;
	}

}

/**********************************************************
 * @Brief Hal_Isr_Uart
 * 		USCI_A1 UART mode interrupt service.
 * 		Buffer RX bytes & run console program when get "ENTER".
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void Hal_Isr_Uart(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) Hal_Isr_Uart (void)
#else
#error Compiler not supported!
#endif
{
	//Rxed data count
	static uint8 Uart_RxCount;

	switch (__even_in_range(UCA1IV, 4))
	{
	case 0:
		break;                             	// Vector 0 - no interrupt
	case 2:                             	// Vector 2 - RXIFG
	{
		//Read 1 byte
		uint8 rxdata;
		rxdata = USCI_A_UART_receiveData(USCI_A1_BASE);

		//Loop back RX
		USCI_A_UART_transmitData(USCI_A1_BASE, rxdata);

		//Check rxed char.
		switch (rxdata)
		{
		case '\r': //"enter"
#if UART_DEBUG_ON
			//Enable all interrupt to ensure Uart console do not block other interrupt
			_EINT();
			//Disable Uart until
			USCI_A_UART_disableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
			Uart_Console(u8Hal_Buf_UartRx);
			USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
#endif
			Uart_RxCount = 0;
			Hal_Mem_set8((uint32) u8Hal_Buf_UartRx, 0x00, sizeof(u8Hal_Buf_UartRx));
			break;
		case '\b': // "backspace"
			if (Uart_RxCount)
			{
				Uart_RxCount--;
			}
			break;
		default:
			u8Hal_Buf_UartRx[Uart_RxCount] = rxdata;
			Uart_RxCount++;
			break;
		}
		break;
	}

	case 4:
		break;                             // Vector 4 - TXIFG
	default:
		break;
	}
}

/***2.6 External Functions ***/
flag Hal_Mcu_init(void)
{
	flag status = FLAG_SUCCESS;

	//16s watch dog timer ,512k / 32k = 16s
	WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKDIVIDER_512K);
	WDT_A_start(WDT_A_BASE);

	//Ports initial
	Hal_Gpio_init();
	status &= Hal_Clock_init(HAL_CPU_F);
	status &= Hal_Adc_init();

	//Bus initial
	status &= Hal_SpiMaster_init(HAL_SPI_MASTER_SPEED);
	status &= Hal_SpiSlave_init();
	Hal_I2cSlave_init(HAL_I2C_SLAVE_ADD_NORMAL);
	status &= Hal_Uart_init(HAL_UART_BAUDRATE);

	//Check Power & turn on iw7027 power.
#if UART_DEBUG_ON
	PrintString("\e[32m\r\nMCU init start, checking power... ");
#endif
	while (Hal_Adc_getResult(ADC_DC60V) < 0x0200)
	{
		DELAY_MS(50);
	}
	while (Hal_Adc_getResult(ADC_DC13V) < 0x0200)
	{
		DELAY_MS(50);
	}

	//Set Default error handle param
	tHal_BoardErrorParam.u8Dc13vMax = 16;
	tHal_BoardErrorParam.u8Dc13vMin = 10;
	tHal_BoardErrorParam.u8Dc60vMax = 70;
	tHal_BoardErrorParam.u8Dc60vMin = 50;
	tHal_BoardErrorParam.fSpiDataErrorIgnore = 1;
	tHal_BoardErrorParam.fIw7027FaultIgnore = 0;
	tHal_BoardErrorParam.fErrorSaveEn = 0;

#if UART_DEBUG_ON
	PrintString("\r\nDC60VADC = ");
	PrintInt(Hal_Adc_getResult(ADC_DC60V));
	PrintString("  DC13VADC = ");
	PrintInt(Hal_Adc_getResult(ADC_DC13V));
	PrintString("\r\nLast Reset Source: ");
	PrintChar(SYSRSTIV);
	PrintString("\r\nMcu_init finish.\r\n\e[30m");
#endif

	//Enable Interrupt & return.
	__enable_interrupt();
	return status;
}

uint8 Hal_Mcu_checkBoardStatus(Hal_BoardInfo_t *boardinfo, Hal_BoardErrorParam_t *errorparam)
{
	static uint8 iserror;
	uint8 retval = 0;

	//Load Board Hardware Info
	boardinfo->fIw7027Fault = HAL_GET_IW7027_FAULT_IN;
	boardinfo->u8Dc60v = (uint32) Hal_Adc_getResult(ADC_DC60V) * 84 / 0x3FF;
	boardinfo->u8Dc13v = (uint32) Hal_Adc_getResult(ADC_DC13V) * 19 / 0x3FF;
	boardinfo->su8McuTemperature = Hal_Adc_getMcuTemperature();

	//BIT0 : Power error flag
	if ((boardinfo->u8Dc60v > errorparam->u8Dc60vMax) || (boardinfo->u8Dc60v < errorparam->u8Dc60vMin)
			|| (boardinfo->u8Dc13v > errorparam->u8Dc13vMax) || (boardinfo->u8Dc13v < errorparam->u8Dc13vMin))
	{
		retval |= BIT0;
	}

	//BIT1 : IW7027 Error Flag
	if (!boardinfo->fIw7027Fault && !errorparam->fIw7027FaultIgnore)
	{
		retval |= BIT1;
	}

	//BIT2 : Signal Error flag ,only set when fSpiDataErrorIgnore is off
	if (!errorparam->fSpiDataErrorIgnore)
	{
		if ((boardinfo->u8SpiRxFreq < errorparam->u8SpiRxFreqMin) || (!boardinfo->fSpiDataValid))
		{
			retval |= BIT2;
		}
	}

	//Set Error Out
	if (retval)
	{
		HAL_SET_ERROR_OUT_LOW;
		errorparam->u8ErrorType = retval;
//1st time error happen
		if (!iserror)
		{
#if UART_DEBUG_ON
			PrintString("\e[31m\r\nERROR ! TYPE : \e[30m");
			PrintChar(retval);
			PrintEnter();
			PrintArray((uint8 *) &tHal_BoardInfo, sizeof(tHal_BoardInfo));
			PrintEnter();
#endif
			if (errorparam->fErrorSaveEn)
			{
				errorparam->u8ErrorCount = *HAL_ERROR_INFO_FLASH_PTR;
				errorparam->u8ErrorCount++;
				FlashCtl_eraseSegment(HAL_ERROR_INFO_FLASH_PTR);
				FlashCtl_write8((uint8*) errorparam, HAL_ERROR_INFO_FLASH_PTR, sizeof(tHal_BoardErrorParam));
				FlashCtl_write8((uint8*) boardinfo, HAL_ERROR_INFO_FLASH_PTR + 0x20, sizeof(tHal_BoardInfo));
			}

		}
		iserror = 1;

	}
	//No error , clear is error flag & set GPIO.
	else
	{
		if (iserror)
		{
#if UART_DEBUG_ON
			PrintString("\e[32m\r\nERROR Cleard.\e[30m\r\n");
#endif
		}

		HAL_SET_ERROR_OUT_HIGH;
		errorparam->u8ErrorType = retval;
		iserror = 0;
	}
	return retval;
}

void Hal_Mcu_reset(void)
{
	//Set watch dog timer to 512 cpu cycles.
	//YZF 2016/5/16 : This delay is for I2C slave finish send ACK before reset.
	//				  Otherwise , there will be no ack when receiving REBOOT cmd.
	//				  512cycles = 20us @ 25Mhz , ACK @ I2C 100kHz = 10us.
	WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_SMCLK, WDT_A_CLOCKDIVIDER_512);
	WDT_A_start(WDT_A_BASE);
	//Wait 612cyles for watch dog reboot.
	__delay_cycles(512 + 100);
}

void Hal_Mcu_invokeBsl(void)
{
	//Disable ISR & jump to BSL section. BSL verder is TI.
	//Refer to 3.8.1 Starting the BSL From an External Application for BSL application note.
	//delay for MCU modules to finish current work.
	__disable_interrupt();
	__delay_cycles(500000);
	((void (*)()) 0x1000)();
}

void Hal_Gpio_init(void)
{
	//STEP1 : Set default value for all ports = input with pull down resistor
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PA, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PB, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PC, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PD, GPIO_PIN_ALL16);

	/* STEP2 : Set GPIO ( BUS I/Os are set by each init functions )
	 * Pin No. is defined in hal.h , function & default value is set as follow.
	 *
	 * Name			Default		PU/PD	Interrupt	Remark
	 * STB_IN		Input		PU		Falling
	 * IW_POWER_EN	Output H	x		x
	 * FAC_TEST		Input		PU		x
	 * ERROR_OUT	Output L	x		x
	 * LED_G		Output L	x		x			Full strength
	 * IW_FAULT_IN	Input		x		x
	 * IW_CS_0~7	Output H	x		x
	 */

	//STB_IN ; H = BL on / L = BL off
	GPIO_setAsInputPinWithPullUpResistor(GPIO_STB_IN);
	GPIO_clearInterrupt(GPIO_STB_IN);
	GPIO_selectInterruptEdge(GPIO_STB_IN, GPIO_HIGH_TO_LOW_TRANSITION);
	GPIO_enableInterrupt(GPIO_STB_IN);

	//IW_POWER_EN ; H = IW power on / L = IW power off
	GPIO_setOutputLowOnPin(GPIO_IW_POWER_EN);
	GPIO_setAsOutputPin(GPIO_IW_POWER_EN);

	//FAC_TEST ; H = Normal / L = Factory test
	GPIO_setAsInputPin(GPIO_FAC_TEST);

	//ERROR_OUT : H = Normal / L = Error
	GPIO_setOutputLowOnPin(GPIO_ERROR_OUT);
	GPIO_setAsOutputPin(GPIO_ERROR_OUT);

	//LED G : H = LED Green on / L = LED Green off
	GPIO_setOutputHighOnPin(GPIO_LED_G);
	GPIO_setAsOutputPin(GPIO_LED_G);
	GPIO_setDriveStrength(GPIO_LED_G, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);

	//IW_FAULT_IN : H = Normal / L = Error
	GPIO_setAsInputPin(GPIO_IW_FAULT_IN);

	//IW_CS_0 ~ IW_CS_7 : H = Not selected / L = Chip selected
	GPIO_setOutputHighOnPin(GPIO_IW_CS_0);
	GPIO_setAsOutputPin(GPIO_IW_CS_0);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_1);
	GPIO_setAsOutputPin(GPIO_IW_CS_1);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_2);
	GPIO_setAsOutputPin(GPIO_IW_CS_2);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_3);
	GPIO_setAsOutputPin(GPIO_IW_CS_3);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_4);
	GPIO_setAsOutputPin(GPIO_IW_CS_4);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_5);
	GPIO_setAsOutputPin(GPIO_IW_CS_5);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_6);
	GPIO_setAsOutputPin(GPIO_IW_CS_6);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_7);
	GPIO_setAsOutputPin(GPIO_IW_CS_7);
}

flag Hal_Adc_init(void)
{
#ifdef __MSP430_HAS_ADC10_A__
	//Set P6.4 P6.5 as ADC input
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN5);

	/*Initialize the ADC10_A Module
	 * Base Address for the ADC10_A Module
	 * Use internal ADC10_A bit as sample/hold signal to start conversion
	 * USE MODOSC 5MHZ Digital Oscillator as clock source
	 * Use default clock divider of 1
	 */
	ADC10_A_init(ADC10_A_BASE, ADC10_A_SAMPLEHOLDSOURCE_SC, ADC10_A_CLOCKSOURCE_ADC10OSC, ADC10_A_CLOCKDIVIDER_1);
	ADC10_A_enable(ADC10_A_BASE);

	/*Set ADC sample time
	 * Base Address for the ADC10_A Module
	 * Sample/hold for 32 clock cycles
	 * Do not enable Multiple Sampling
	 */
	ADC10_A_setupSamplingTimer(ADC10_A_BASE, ADC10_A_CYCLEHOLD_32_CYCLES, ADC10_A_MULTIPLESAMPLESDISABLE);

	//Configure internal reference
	//If ref generator busy, WAIT with 1s time out
	uint32 timeout = UCS_getSMCLK();
	while (REF_BUSY == Ref_isRefGenBusy(REF_BASE) && timeout--)
	{
		;
	}
	if (timeout == 0)
	{
		return FLAG_FAIL;
	}
	//Select internal ref = 1.5V
	Ref_setReferenceVoltage(REF_BASE, REF_VREF1_5V);
	//Internal Reference ON
	Ref_enableReferenceVoltage(REF_BASE);

	return FLAG_SUCCESS;
#endif

#ifdef __MSP430_HAS_ADC12_PLUS__
	return FLAG_SUCCESS;
#endif

}

uint16 Hal_Adc_getResult(uint8 port)
{
#ifdef __MSP430_HAS_ADC10_A__
	//Configure Memory Buffer
	/*
	 * Base Address for the ADC10_A Module
	 * Use input A0
	 * Use positive reference of Internally generated Vref
	 * Use negative reference of AVss
	 */
	ADC10_A_configureMemory(ADC10_A_BASE, port, ADC10_A_VREFPOS_INT, ADC10_A_VREFNEG_AVSS);

	//Start Conversion
	ADC10_A_startConversion(ADC10_A_BASE, ADC10_A_SINGLECHANNEL);

	while (ADC10_A_isBusy(ADC10_A_BASE))
	{
		;
	}

	//Stop conversion when conversion finish.
	ADC10_A_disableConversions(ADC10_A_BASE, ADC10_A_COMPLETECONVERSION);

	return ADC10_A_getResults(ADC10_A_BASE);
#endif

#ifdef __MSP430_HAS_ADC12_PLUS__
	return 0x201;
#endif
}

int8 Hal_Adc_getMcuTemperature(void)
{
#ifdef __MSP430_HAS_ADC10_A__
	uint16 adcval;
	int32_t temperature;

	//Select Internal temp sensor , internal 1.5V referance
	ADC10_A_configureMemory(ADC10_A_BASE, ADC10_A_INPUT_TEMPSENSOR, ADC10_A_VREFPOS_INT, ADC10_A_VREFNEG_AVSS);

	//Start Conversion
	ADC10_A_startConversion(ADC10_A_BASE, ADC10_A_SINGLECHANNEL);

	while (ADC10_A_isBusy(ADC10_A_BASE))
	{
		;
	}

	//Stop conversion when conversion finish.
	ADC10_A_disableConversions(ADC10_A_BASE, ADC10_A_COMPLETECONVERSION);

	//Calculate temperature according to internal caliberation infor.
	adcval = ADC10_A_getResults(ADC10_A_BASE);
	temperature = (((int32_t) adcval - HAL_ADCCAL_15V_30C) * (85 - 30)) / (HAL_ADCCAL_15V_85C - HAL_ADCCAL_15V_30C) + 30;

	//return temperature
	return ((int8) temperature);
#endif

#ifdef __MSP430_HAS_ADC12_PLUS__
	return 30;
#endif
}

flag Hal_Clock_init(uint32 cpu_speed)
{
	//Set VCore = 3 for 24MHz clock
	PMM_setVCore(PMM_CORE_LEVEL_3);

	//Set DCO FLL reference = REFO
	UCS_initClockSignal(UCS_FLLREF, UCS_REFOCLK_SELECT, UCS_CLOCK_DIVIDER_1);
	//Set ACLK = REFO
	UCS_initClockSignal(UCS_ACLK, UCS_REFOCLK_SELECT, UCS_CLOCK_DIVIDER_1);

	//Set Ratio and Desired MCLK Frequency  and initialize DCO
	UCS_initFLLSettle(cpu_speed / 1000, cpu_speed / 32768);

	return FLAG_SUCCESS;
}

flag Hal_SpiMaster_init(uint32 spi_speed)
{
	//P4.1 = MOSI , P4.2 = MISO , P4.3 = CLK
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3);

	//Set CS pins to H.
	GPIO_setOutputHighOnPin(GPIO_IW_CS_0);
	GPIO_setAsOutputPin(GPIO_IW_CS_0);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_1);
	GPIO_setAsOutputPin(GPIO_IW_CS_1);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_2);
	GPIO_setAsOutputPin(GPIO_IW_CS_2);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_3);
	GPIO_setAsOutputPin(GPIO_IW_CS_3);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_4);
	GPIO_setAsOutputPin(GPIO_IW_CS_4);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_5);
	GPIO_setAsOutputPin(GPIO_IW_CS_5);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_6);
	GPIO_setAsOutputPin(GPIO_IW_CS_6);
	GPIO_setOutputHighOnPin(GPIO_IW_CS_7);
	GPIO_setAsOutputPin(GPIO_IW_CS_7);

	//Initialize Master
	uint8 ucb1returnValue = 0x00;

	USCI_B_SPI_initMasterParam param_ucb1 =
	{ 0 };
	param_ucb1.selectClockSource = USCI_B_SPI_CLOCKSOURCE_SMCLK;
	param_ucb1.clockSourceFrequency = UCS_getSMCLK();
	param_ucb1.desiredSpiClock = spi_speed;
	param_ucb1.msbFirst = USCI_B_SPI_MSB_FIRST;
	param_ucb1.clockPhase = USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
	param_ucb1.clockPolarity = USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;

	ucb1returnValue = USCI_B_SPI_initMaster(USCI_B1_BASE, &param_ucb1);
	if (FLAG_FAIL == ucb1returnValue)
	{
		return FLAG_FAIL;
	}
	//Enable SPI module
	USCI_B_SPI_enable(USCI_B1_BASE);

	//Wait for slave to initialize
	__delay_cycles(100);

	return FLAG_SUCCESS;
}

void Hal_SpiMaster_setCsPin(uint8 chipsel)
{
	if (chipsel & BIT0)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_0);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_0);
	}

	if (chipsel & BIT1)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_1);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_1);
	}

	if (chipsel & BIT2)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_2);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_2);
	}

	if (chipsel & BIT3)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_3);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_3);
	}

	if (chipsel & BIT4)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_4);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_4);
	}

	if (chipsel & BIT5)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_5);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_5);
	}

	if (chipsel & BIT6)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_6);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_6);
	}

	if (chipsel & BIT7)
	{
		GPIO_setOutputLowOnPin(GPIO_IW_CS_7);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_IW_CS_7);
	}

}

uint8 Hal_SpiMaster_sendMultiByte(uint8 *txdata, uint16 length)
{
	//Send multiple bytes till count down
	while (length--)
	{
		//TX buffer ready?
		while (!USCI_B_SPI_getInterruptStatus(USCI_B1_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT))
		{
			;
		}
		//Transmit Data to slave
		USCI_B_SPI_transmitData(USCI_B1_BASE, *(txdata++));
	}

	while (USCI_B_SPI_isBusy(USCI_B1_BASE))
	{
		;
	}

	//Return last byte received.
	return USCI_B_SPI_receiveData(USCI_B1_BASE);
}

uint8 Hal_SpiMaster_sendSingleByte(uint8 txdata)
{
	//TX buffer ready?
	while (!USCI_B_SPI_getInterruptStatus(USCI_B1_BASE, USCI_B_SPI_TRANSMIT_INTERRUPT))
	{
		;
	}
	//Transmit Data to slave
	USCI_B_SPI_transmitData(USCI_B1_BASE, txdata);

	while (USCI_B_SPI_isBusy(USCI_B1_BASE))
	{
		;
	}
	//Return last byte received.
	return USCI_B_SPI_receiveData(USCI_B1_BASE);

}

flag Hal_SpiSlave_init(void)
{
	//P2.7 = CLK , P3.3 = MOSI , P3.4 = MISO
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN7);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN3);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN4);

	//P1.3 = CS , User Timer A0.2 ,trigger both rising /falling edge
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN3);

	//Start timer in continuous mode sourced by ACLK
	Timer_A_initContinuousModeParam initTimerA0Param =
	{ 0 };
	initTimerA0Param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	initTimerA0Param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	initTimerA0Param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
	initTimerA0Param.timerClear = TIMER_A_DO_CLEAR;
	initTimerA0Param.startTimer = true;
	Timer_A_initContinuousMode(TIMER_A0_BASE, &initTimerA0Param);

	//Initialize TA0.2 capture mode as CS pin for SPI module , both edge interrupt.
	Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
	Timer_A_initCaptureModeParam capparam =
	{ 0 };
	capparam.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
	capparam.captureMode = TIMER_A_CAPTUREMODE_FALLING_EDGE;
	capparam.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
	capparam.synchronizeCaptureSource = TIMER_A_CAPTURE_ASYNCHRONOUS;//TIMER_A_CAPTURE_SYNCHRONOUS;TIMER_A_CAPTURE_ASYNCHRONOUS
	capparam.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
	capparam.captureOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
	Timer_A_initCaptureMode(TIMER_A0_BASE, &capparam);

	//Set DMA for Spi slave
	DMA_initParam dma0param =
	{ 0 };
	dma0param.channelSelect = DMA_CHANNEL_0;
	dma0param.transferModeSelect = DMA_TRANSFER_SINGLE;
	dma0param.transferSize = 256;
	dma0param.triggerSourceSelect = DMA_TRIGGERSOURCE_16;
	dma0param.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
	dma0param.triggerTypeSelect = DMA_TRIGGER_HIGH;
	DMA_init(&dma0param);

	//Source = UCA0RXBUF , Destination = &rxbuff
	DMA_setSrcAddress(DMA_CHANNEL_0, USCI_A_SPI_getReceiveBufferAddressForDMA(USCI_A0_BASE), DMA_DIRECTION_UNCHANGED);
	DMA_setDstAddress(DMA_CHANNEL_0, (uint32) u8Hal_Buf_SpiSlaveRx, DMA_DIRECTION_INCREMENT);

	//Init Spi slave
	uint8 uca0_returnValue = FLAG_FAIL;
	uca0_returnValue = USCI_A_SPI_initSlave(
	USCI_A0_BASE, USCI_A_SPI_MSB_FIRST,
	USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
	USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH);

	if (FLAG_FAIL == uca0_returnValue)
	{
		return FLAG_FAIL;
	}
	//Enable SPI Module
	USCI_A_SPI_enable(USCI_A0_BASE);

	return FLAG_SUCCESS;
}

void Hal_SpiSlave_startRx(void)
{
	//Reset DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
	DMA_enableTransfers(DMA_CHANNEL_0);
	//Enable SPI_RX
	//NOTE 20150923 : P2.7 is SPI_CLK pin.
	//Only turn on SPI when SPI is free.
	//Add this to avoid some MainBoard reset timming problem
	if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7))
	{
		USCI_A_SPI_enable(USCI_A0_BASE);
	}
}

void Hal_SpiSlave_stopRx(void)
{
	//Disable SPI_RX
	USCI_A_SPI_disable(USCI_A0_BASE);
	//Disable DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
}

void Hal_SpiSlave_disable(void)
{
	//Disable Cs isr
	Timer_A_disableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Disable SPI RX
	USCI_A_SPI_disable(USCI_A0_BASE);
}

void Hal_SpiSlave_enable(void)
{
	//Enable Cs isr
	Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Enable SPI Tx
	USCI_A_SPI_enable(USCI_A0_BASE);
}

void Hal_I2cSlave_init(uint8 slaveaddress)
{
	//P3.0 = SDA , P3.1 = SCL
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN0);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN1);

	//Initialize I2C as a slave device
	USCI_B_I2C_initSlave(USCI_B0_BASE, slaveaddress);

	//Specify transmit mode
	USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_RECEIVE_MODE);

	//Enable I2C Module to start operations
	USCI_B_I2C_enable(USCI_B0_BASE);

	//Enable interrupts
	USCI_B_I2C_clearInterrupt(USCI_B0_BASE,
	USCI_B_I2C_START_INTERRUPT +
	USCI_B_I2C_STOP_INTERRUPT +
	USCI_B_I2C_RECEIVE_INTERRUPT +
	USCI_B_I2C_TRANSMIT_INTERRUPT);
	USCI_B_I2C_enableInterrupt(USCI_B0_BASE,
	USCI_B_I2C_START_INTERRUPT +
	USCI_B_I2C_STOP_INTERRUPT +
	USCI_B_I2C_RECEIVE_INTERRUPT +
	USCI_B_I2C_TRANSMIT_INTERRUPT);

	//Clear Buffer
	Hal_Mem_set8((uint32) &u8Hal_Buf_I2cSlave, 0x00, sizeof(u8Hal_Buf_I2cSlave));

}

flag Hal_Uart_init(uint32 baudrate)
{
	//P4.4 = TX , P4.5 =RX
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);

	//Baudrate = UART_BAUDRATE , clock freq = SMCLK_F
	//Clock devider value refer to application note.
	USCI_A_UART_initParam param =
	{ 0 };
	param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar = UCS_getSMCLK() / HAL_UART_BAUDRATE;
	param.firstModReg = 0;
	param.secondModReg = 5;
	param.parity = USCI_A_UART_NO_PARITY;
	param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
	param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
	param.uartMode = USCI_A_UART_MODE;
	param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

	if (FLAG_FAIL == USCI_A_UART_init(USCI_A1_BASE, &param))
	{
		return FLAG_FAIL;
	}

	//Enable UART module for operation
	USCI_A_UART_enable(USCI_A1_BASE);

	//Enable Receive Interrupt
	USCI_A_UART_clearInterrupt(USCI_A1_BASE,
	USCI_A_UART_RECEIVE_INTERRUPT);
	USCI_A_UART_enableInterrupt(USCI_A1_BASE,
	USCI_A_UART_RECEIVE_INTERRUPT);

	return FLAG_SUCCESS;
}

void Hal_Uart_sendSingleByte(uint8 data)
{
	USCI_A_UART_transmitData(USCI_A1_BASE, data);
}

void Hal_PwmOut_init(uint8 initfreq, uint16 delay)
{
	//P2.0 as TA1.1 output
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0);

	//Start Timer in up/down mode
	Timer_A_initUpDownModeParam initUpDownParam =
	{ 0 };
	initUpDownParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	initUpDownParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	initUpDownParam.timerPeriod = 0;
	initUpDownParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	initUpDownParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
	initUpDownParam.timerClear = TIMER_A_DO_CLEAR;
	initUpDownParam.startTimer = false;
	Timer_A_initUpDownMode(TIMER_A1_BASE, &initUpDownParam);

	// PWM1 output
	Timer_A_outputPWMParam pwm1param =
	{ 0 };
	pwm1param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	pwm1param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	pwm1param.compareOutputMode = TIMER_A_OUTPUTMODE_SET_RESET;
	pwm1param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
	pwm1param.dutyCycle = delay;

	//if 0 ,it's mute fcuntion
	if (initfreq == 0)
	{
		pwm1param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
	}
	//not 0 ,set frequency.
	else
	{
		pwm1param.timerPeriod = UCS_getACLK() / initfreq / 2;
	}

	Timer_A_outputPWM(TIMER_A1_BASE, &pwm1param);
	Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UPDOWN_MODE);
}

void Hal_PwmOut_sync(void)
{
	Timer_A_clear(TIMER_A1_BASE);
}

void Hal_Mem_set8(uint32 memadd, uint8 value, uint16 size)
{
	uint8 buf = value;

	DMA_initParam dma2param =
	{ 0 };
	dma2param.channelSelect = DMA_CHANNEL_2;
	dma2param.transferModeSelect = DMA_TRANSFER_BLOCK;
	dma2param.transferSize = size;
	dma2param.triggerSourceSelect = DMA_TRIGGERSOURCE_0;
	dma2param.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
	dma2param.triggerTypeSelect = DMA_TRIGGER_HIGH;
	DMA_init(&dma2param);

	//Source = UCA0RXBUF , Destination = &rxbuff
	DMA_setSrcAddress(DMA_CHANNEL_2, (uint32) &buf, DMA_DIRECTION_UNCHANGED);
	DMA_setDstAddress(DMA_CHANNEL_2, memadd, DMA_DIRECTION_INCREMENT);

	//Start Transfer
	DMA_enableTransfers(DMA_CHANNEL_2);
	DMA_startTransfer(DMA_CHANNEL_2);
}

void Hal_Mem_set16(uint32 memadd, uint16 value, uint16 size)
{
	uint16 buf = value;

	DMA_initParam dma2param =
	{ 0 };
	dma2param.channelSelect = DMA_CHANNEL_2;
	dma2param.transferModeSelect = DMA_TRANSFER_BLOCK;
	dma2param.transferSize = size;
	dma2param.triggerSourceSelect = DMA_TRIGGERSOURCE_0;
	dma2param.transferUnitSelect = DMA_SIZE_SRCWORD_DSTWORD;
	dma2param.triggerTypeSelect = DMA_TRIGGER_HIGH;
	DMA_init(&dma2param);

	//Source = UCA0RXBUF , Destination = &rxbuff
	DMA_setSrcAddress(DMA_CHANNEL_2, (uint32) &buf, DMA_DIRECTION_UNCHANGED);
	DMA_setDstAddress(DMA_CHANNEL_2, memadd, DMA_DIRECTION_INCREMENT);

	//Start Transfer
	DMA_enableTransfers(DMA_CHANNEL_2);
	DMA_startTransfer(DMA_CHANNEL_2);
}

void Hal_Mem_copy(uint32 target_add, uint32 source_add, uint16 size)
{
	DMA_initParam dma2param =
	{ 0 };
	dma2param.channelSelect = DMA_CHANNEL_2;
	dma2param.transferModeSelect = DMA_TRANSFER_BLOCK;
	dma2param.transferSize = size;
	dma2param.triggerSourceSelect = DMA_TRIGGERSOURCE_0;
	dma2param.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
	dma2param.triggerTypeSelect = DMA_TRIGGER_HIGH;
	DMA_init(&dma2param);

	//Source = UCA0RXBUF , Destination = &rxbuff
	DMA_setSrcAddress(DMA_CHANNEL_2, source_add, DMA_DIRECTION_INCREMENT);
	DMA_setDstAddress(DMA_CHANNEL_2, target_add, DMA_DIRECTION_INCREMENT);

	//Start Transfer
	DMA_enableTransfers(DMA_CHANNEL_2);
	DMA_startTransfer(DMA_CHANNEL_2);
}

void Hal_Sch_init(void)
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
	tHal_CpuScheduler.u16CpuTickPeriod = 32;	//1ms	1kHz
	tHal_CpuScheduler.u16GpioCheckPeriod = 327;	//10ms	100Hz
	tHal_CpuScheduler.u16TestModePeriod = 547;	//16.7ms 60Hz
	tHal_CpuScheduler.fSystemResetN = 1;	//System On
	tHal_CpuScheduler.fLocalDimmingOn = 1;	//Local Dimming On

}

void Hal_Sch_CpuOff(void)
{
	//Sum up CPU on time .
	if (TB0R > u16Hal_SchCpuOnMark)
	{
		u32Hal_SchCpuWorkTime += TB0R - u16Hal_SchCpuOnMark;
	}
	u16Hal_SchCpuOnMark = 0;
	//Turn off CPU .
	__bis_SR_register(LPM0_bits);
}

inline void Hal_Flash_eraseSegment(uint8_t *flash_ptr)
{
	do
	{
		FlashCtl_eraseSegment(flash_ptr);
	} while (!FlashCtl_performEraseCheck(flash_ptr, 128));
}

inline void Hal_Flash_write(uint8 *data_ptr, uint8 *flash_ptr, uint16 count)
{
	FlashCtl_write8(data_ptr, flash_ptr, count);
}
