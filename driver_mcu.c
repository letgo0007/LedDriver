/******************************************************************************
 * @file 	[driver_scheduler.c]
 *
 * MCU hardware driver.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/

/***1 Includes ***************************************************************/

#include "driver_mcu.h"

#include "app_dpl.h"
#include "driver_iw7027.h"
#include "driver_scheduler.h"
#include "driver_uartdebug.h"

/***2.1 Internal Marcos ******************************************************/
//XT1 frequency, if no external XT1,set to 0
#define BOARD_XT1_F						(0)
//XT2 frequency, if no external XT2,set to 0
#define BOARD_XT2_F						(0)
//High speed Sub_System_Clock
#define BOARD_SMCLK_F					(BOARD_CPU_F)
//Low speed Assist_Clock
#define BOARD_ACLK_F					(32768)
//I2C Slave addess for [NORMAL] mode ,7bit mode
#define BOARD_I2C_SLAVE_ADD_NORMAL		(0x14)
//I2C Slave addess for [ISP] mode ,7bit mode
#define ISP_I2C_SLAVE_ADDRESS			(0x1C)
//I2C buffer start address for [NORMAL] mode .
#define BOARD_I2C_BUF_ADD_NORMAL		(0x4000)
//I2C buffer start address for [ISP] mode .
#define BOARD_I2C_BUF_ADD_ISP			(0x3C00)
//SPI master speed (unit in Hz)
#define BOARD_SPI_MASTER_SPEED			(4000000)
//Bound Rate of UART
#define BOARD_UART_BAUDRATE				(115200)
//Temperature Sensor Calibration value for 30C
#define BOARD_ADCCAL_15V_30C  			*((uint16 *)0x1A1A)
//Temperature Sensor Calibration value for 85C
#define BOARD_ADCCAL_15V_85C  			*((uint16 *)0x1A1C)

//ISP mode I2C slave address
#define ISP_I2C_SLAVE_ADDRESS		(0x1C)
//ISP exit flag address on flash
#define ISP_EXIT_FLAG_ADDRESS		(0x1900)
//ISP exit password value ,must = 32bit.
#define ISP_EXIT_PASSWORD32			(0x20140217)

/***2.2 Internal Struct ******/

/***2.3 Internal Variables ***/

//ISP->NORMAL password in flash
//Default error detection parameters.
const ErrorParam Default_ErrorParam =
{ .eDc13vMax = 16, .eDc13vMin = 10, .eDc60vMax = 70, .eDc60vMin = 50, .eSpiRxFreqMin = 20, .eSpiDataErrorIgnore = 1,
		.eIw7027FaultIgnore = 0, .eErrorSaveEn = 0, };

//Write ISP_INIT_EXIT_FLAG to flash to exit isp mode on 1st run.
//Note that need open the project Properties - Debug - MSP43x Options - Erase Options = Enable Erase Flash & Info
#pragma LOCATION(ISP_INIT_EXIT_FLAG,ISP_EXIT_FLAG_ADDRESS)
volatile static const uint32 ISP_INIT_EXIT_FLAG = 0x20140217;

/***2.4 External Variables ***/

//Local Dimming duty information buffer
uint16 HwBuf_InputDuty[128] =
{ 0 };
uint16 HwBuf_OutputDuty[128] =
{ 0 };
uint16 HwBuf_TestDuty[128] =
{ 0 };
//Spi Slave hardware buffer
uint8 HwBuf_SpiSlaveRx[256] =
{ 0 };
//Uart rx hardware buffer.
uint8 HwBuf_UartRx[256] =
{ 0 };

/******************************************************************************
 * Set I2C Slave Map Struct
 * Put Variables those need I2C access to the RAM address of
 * BOARD_I2C_BUF_ADD_NORMAL ~ BOARD_I2C_BUF_ADD_NORMAL + 0xFF .
 * Using the #pragma location = ADDRESS compiler command to appoint address.
 * Refer to TI compiler #pragma cmd list file :
 * http://www.ti.com/lit/ug/slau132l/slau132l.pdf
 *
 * Name            			origin    	length
 * ----------------------  	--------  	---------
 * SysParam_Schedule		0x00		0x20
 * SysParam_BoardInfo		0x20		0x10
 * SysParam_Error			0x30		0x10
 * SysParam_Iw7027			0x40		0x30
 * SysParam_Dpl				0x70		0x30
 * HwBuf_I2cSlave			0xA0		0x50
 * SysParam_Version			0xF0		0x08
 * SysParam_IspPassword		0xF8		0x08
 * ----------------------  	--------  	---------
 * 							total		0x100
 ******************************************************************************/
#pragma LOCATION(SysParam_Schedule , BOARD_I2C_BUF_ADD_NORMAL + 0x00)
#pragma LOCATION(SysParam_BoardInfo , BOARD_I2C_BUF_ADD_NORMAL + 0x20)
#pragma LOCATION(SysParam_Error , BOARD_I2C_BUF_ADD_NORMAL + 0x30)
#pragma LOCATION(SysParam_Iw7027 , BOARD_I2C_BUF_ADD_NORMAL + 0x40)
#pragma LOCATION(SysParam_Dpl , BOARD_I2C_BUF_ADD_NORMAL + 0x70)
#pragma LOCATION(HwBuf_I2cSlave , BOARD_I2C_BUF_ADD_NORMAL + 0xA0)
#pragma LOCATION(SysParam_Version , BOARD_I2C_BUF_ADD_NORMAL + 0xF0)
#pragma LOCATION(SysParam_IspPassword , BOARD_I2C_BUF_ADD_NORMAL + 0xF8)

Scheduler SysParam_Schedule =
{ 0 };

BoardInfo SysParam_BoardInfo =
{ 0 };

ErrorParam SysParam_Error =
{ 0 };

Iw7027Param SysParam_Iw7027 =
{ 0 };

DPL_Prama SysParam_Dpl =
{ 0 };

uint8 HwBuf_I2cSlave[0x50] =
{ 0 };

uint64 SysParam_Version =
{ BOARD_VERSION };

uint64 SysParam_IspPassword =
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
 * Isr_Gpio_P1				0x4800
 * Isr_SpiSlave_Cs			0x4900
 * Isr_I2cSlave				0x4A00
 * Isr_Uart					0x4D00
 * Isr_Scheduler_TimerB0	0x4E00					@driver_scheduler.c
 * Isr_Scheduler_Rtc		0x4F00					@driver_scheduler.c
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
//Place ISR & Boot
#pragma LOCATION(Isr_Gpio_P1,0x4800)
#pragma LOCATION(Isr_SpiSlave_Cs,0x4900)
#pragma LOCATION(Isr_I2cSlave,0x4A00)
#pragma LOCATION(Isr_Uart,0x4D00)
#pragma LOCATION(_system_pre_init,0x4480)

/**********************************************************
 * @Brief SetVcoreUp
 * 		Set Mcu Vcore Up for higher cpu speed.Refer to TI sample code.
 * 		Set Vcore level up 1 step up at a time another.
 * 		Use volatile inline to force compiler to build this function in
 * 		_system_pre_init()
 * @Param
 * 		level : available from 1~3 , refer to device specsheet for more info.
 * @Return
 * 		NONE
 **********************************************************/
volatile inline void SetVcoreUp(unsigned int level)
{
	// Open PMM registers for write
	PMMCTL0_H = PMMPW_H;
	// Set SVS/SVM high side new level
	SVSMHCTL = SVSHE + SVSHRVL0 * level + SVMHE + SVSMHRRL0 * level;
	// Set SVM low side to new level
	SVSMLCTL = SVSLE + SVMLE + SVSMLRRL0 * level;
	// Wait till SVM is settled
	while ((PMMIFG & SVSMLDLYIFG) == 0)
		;
	// Clear already set flags
	PMMIFG &= ~(SVMLVLRIFG + SVMLIFG);
	// Set VCore to new level
	PMMCTL0_L = PMMCOREV0 * level;
	// Wait till new level reached
	if ((PMMIFG & SVMLIFG))
		while ((PMMIFG & SVMLVLRIFG) == 0)
			;
	// Set SVS/SVM low side to new level
	SVSMLCTL = SVSLE + SVSLRVL0 * level + SVMLE + SVSMLRRL0 * level;
	// Lock PMM registers for write access
	PMMCTL0_H = 0x00;
}

/**********************************************************
 * @Brief _system_pre_init
 * 		System low level initial before RAM intialization & main().
 * 		Use as isp mode boot loader.
 * 		DO NOT call any togher functions . If have to ,build the fucntion
 * 		with volatile inline . If this function is erased during ISP
 * 		process , it will cause system failure. Note that *,/ operation are
 * 		also external fuctions , do not use *,/ operation in
 * @Param
 * 		NONE
 * @Return
 * 		0	: omit RAM initialization.
 * 		1	: do RAM initialization.
 **********************************************************/
int _system_pre_init(void)
{
	//Watch dog to 1sec .Clk source = ACLK , length = 32k.
	WDTCTL = WDTPW + WDTCNTCL + WDTHOLD + WDTSSEL_1 + WDTIS_4;

	//All gpio reset to Input with Pull Down.
	PAOUT = 0;
	PBOUT = 0;
	PCOUT = 0;
	PDOUT = 0;
	PADIR = 0;
	PBDIR = 0;
	PCDIR = 0;
	PDDIR = 0;
	PAREN = 1;
	PBREN = 1;
	PCREN = 1;
	PDREN = 1;

	//Disable ISR because Interrupt vetore @ 0xffd0~0xffff will also be erased in ISP mode.
	__disable_interrupt();

	//Check password on info flash to decide whether go to main().
	if (ISP_INIT_EXIT_FLAG == ISP_EXIT_PASSWORD32) //Password correct , go to main()
	{
		return 1;
	}
	else //Password wrong , run ISP function.
	{
		//Initialize buffers .
		//The flash segment size = 512 for MSP430 device ,so data buffer max size =512.
		//Flash Address is 32bit , address byte buffer size = 4 bytes .
		uint8 data_buff[512] =
		{ 0 };
		uint8 op_add_buff[4] =
		{ 0 };
		uint32 op_add = 0;
		uint8 *op_ptr = 0;
		uint16 txcount = 0;
		uint16 rxcount = 0;

		//Initialize CPU CLK to 25MHz
		SetVcoreUp(0x01);
		SetVcoreUp(0x02);
		SetVcoreUp(0x03);

		UCSCTL3 = SELREF_2;                       // Set DCO FLL reference = REFO
		UCSCTL4 |= SELA_2;                        // Set ACLK = REFO

		__bis_SR_register(SCG0);                  // Disable the FLL control loop
		UCSCTL0 = 0x0000;                         // Set lowest possible DCOx, MODx
		UCSCTL1 = DCORSEL_7;                      // Select DCO range 50MHz operation
		UCSCTL2 = FLLD_1 + 762;                   // Set DCO Multiplier for 25MHz
												  // (N + 1) * FLLRef = Fdco
												  // (762 + 1) * 32768 = 25MHz
												  // Set FLL Div = fDCOCLK/2
		__bic_SR_register(SCG0);                  // Enable the FLL control loop
		__delay_cycles(782000);

		// Loop until XT1,XT2 & DCO stabilizes - In this case only DCO has to stabilize
		do
		{
			UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG);
			// Clear XT2,XT1,DCO fault flags
			SFRIFG1 &= ~OFIFG;                      // Clear fault flags
		} while (SFRIFG1 & OFIFG);                   // Test oscillator fault flag

		//P4.6 = ERROR_OUT , P4.7 = LED_G , P3.0 = SDA ,P3.1 = SCL
		P4OUT |= BIT7 + BIT6;
		P4DIR |= BIT7 + BIT6;
		P3SEL |= BIT0 + BIT1;

		//Initialize I2C slave in ISP mode address ,without ISR function.
		UCB0CTL1 |= UCSWRST;
		UCB0CTL0 = UCMODE_3 + UCSYNC;
		UCB0I2COA = ISP_I2C_SLAVE_ADDRESS;
		UCB0CTL1 &= ~UCSWRST;

		while (1)
		{
			//Feed watch dog
			uint8 newWDTStatus = (WDTCTL & 0x00FF) | WDTCNTCL;
			WDTCTL = WDTPW + newWDTStatus;

			//YZF 2016/6/2 : Buffer UCB0IFG first to avoid UCB0IFG modified during processing .
			//				 This make sure I2C events are handled in order.
			//				 For some master device , RX/STOP , TX/START may come very close in time.
			uint16 UCB0IFG_BUFF = UCB0IFG;

			//I2C Slave RX Event (Master send bytes)
			if (UCB0IFG_BUFF & UCRXIFG)
			{
				//Buffer I2C slave data
				if (rxcount < 3)	// 0~2 bytes are operation address bytes.
				{
					op_add_buff[2 - rxcount] = UCB0RXBUF;

				}
				else	// 3+ bytes are data bytes.
				{
					data_buff[rxcount - 3] = UCB0RXBUF;
				}

				rxcount++;
				UCB0IFG &= ~UCRXIFG;
			}

			//I2C Slave START Event (Master call slave address)
			if (UCB0IFG_BUFF & UCSTTIFG)
			{
				//Calculate operation address & pointer , for "Restart" conditon.
				op_add = (*((volatile uint32 *) &op_add_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				//Reset count
				txcount = 0;
				rxcount = 0;
				UCB0IFG &= ~UCSTTIFG;
			}

			//I2C Slave STOP Event (Master send STOP)
			if (UCB0IFG_BUFF & UCSTPIFG)
			{
				//Calculate operation address & pointer
				op_add = (*((volatile uint32 *) &op_add_buff[0]));
				op_ptr = (uint8 *) (op_add & 0x0FFFFF);

				/* Flash operation according to opp_add
				 * Address				Operation
				 * -----------------	----------
				 * 0x104400~0x124400 	ERASE
				 * 0x004400~0x024400	WRITE
				 * 0xFFFFFF				EXIT ISP
				 *
				 */
				if ((op_add >= 0x104400) && (op_add <= 0x124400))	//ERASE
				{
					FCTL3 = FWKEY;
					FCTL1 = FWKEY + ERASE;
					*op_ptr = 0xFF;
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;
				}
				if ((rxcount > 2) && (op_add >= 0x004400) && (op_add <= 0x024400))	//WRITE
				{
					uint16 i;
					uint8 * flash_ptr;
					flash_ptr = (uint8 *) op_add;

					FCTL3 = FWKEY;
					FCTL1 = FWKEY + WRT;
					for (i = 0; i < rxcount - 3; i++)
					{
						*(flash_ptr + i) = data_buff[i];
					}
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;
				}
				if (op_add == 0xFFFFFF)	//EXIT ISP
				{
					//Write ISP EXIT Password.
					unsigned long * flash_ptr;
					flash_ptr = (unsigned long *) ISP_EXIT_FLAG_ADDRESS;
					//Erase segment first
					FCTL3 = FWKEY;
					FCTL1 = FWKEY + ERASE;
					*flash_ptr = 0;
					//Long-word(32bit) write
					FCTL1 = FWKEY + BLKWRT;
					*flash_ptr = ISP_EXIT_PASSWORD32;
					//Lock Flash
					FCTL1 = FWKEY;
					FCTL3 = FWKEY + LOCK;

					//Reboot
					PMMCTL0 |= PMMSWBOR;
				}
				UCB0IFG &= ~UCSTPIFG;
			}

			//I2C Slave TX Event (Master Read)
			if (UCB0IFG_BUFF & UCTXIFG)
			{
				UCB0TXBUF = op_ptr[txcount];
				txcount++;
			}
		}	//End of while (1)
	}
}

/**********************************************************
 * @Brief Isr_Gpio_P1
 * 		GPIO_P1 interrupt service rotine.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Isr_Gpio_P1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Isr_Gpio_P1 (void)
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
		if (!HW_GET_STB_IN)
		{
			//Mute Backlight
			Mem_set8((uint32) &HwBuf_TestDuty, 0x00, sizeof(HwBuf_TestDuty));
			Iw7027_updateDuty((uint16*) HwBuf_TestDuty);

			//Hold CPU till STB get High , if STB = L for 1s, reset Mcu.
			uint16 timeout = 0;
			while (!HW_GET_STB_IN)
			{
				DELAY_MS(10);
				timeout++;
				if (timeout > 100)
				{
					Mcu_reset();
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
 * @Brief Isr_SpiSlave_Cs
 * 		Timer_A0 ISR ,use as Spi Slave Cs pin .
 * 		Both Edge Trigger.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Isr_SpiSlave_Cs(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Isr_SpiSlave_Cs (void)
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
		SpiSlave_stopRx();
		SysParam_Schedule.taskFlagSpiRx = 1;
		SpiSlave_startRx();
		//Sync PWM out
		PwmOut_sync();
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
		SysParam_BoardInfo.bSpiRxFreq = (SpiSlave_CsEdgeCount - 1) / 2;
		SpiSlave_CsEdgeCount = 0;
		break;
	default:
		break;
	}
}


/**********************************************************
 * @Brief Isr_I2cSlave
 * 		USCI_B0 I2C mode interrupt service.
 * 		Buffer I2C Slave data to certain RAM address.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void Isr_I2cSlave(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) Isr_I2cSlave (void)
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
		SysParam_Schedule.taskFlagI2c = 1;
		//ISP mode entrance ,check pass word.
		if (SysParam_IspPassword == 0x20140217)
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
			Mcu_reset();
		}
		break;
	case 10:	    	// Vector 10: RXIFG
		if (rxcount == 0)
		{
			op_add = UCB0RXBUF;
		}
		else
		{
			HWREG8(BOARD_I2C_BUF_ADD_NORMAL + op_add + rxcount - 1) = UCB0RXBUF;
		}
		rxcount++;
		break;
	case 12:	    	// Vector 12: TXIFG
		UCB0TXBUF = HWREG8(BOARD_I2C_BUF_ADD_NORMAL + op_add + txcount);
		txcount++;
		break;
	default:
		break;
	}

}

/**********************************************************
 * @Brief Isr_Uart
 * 		USCI_A1 UART mode interrupt service.
 * 		Buffer RX bytes & run console program when get "ENTER".
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void Isr_Uart(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) Isr_Uart (void)
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
			Uart_Console(HwBuf_UartRx);
			USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
#endif
			Uart_RxCount = 0;
			Mem_set8((uint32) HwBuf_UartRx, 0x00, sizeof(HwBuf_UartRx));
			break;
		case '\b': // "backspace"
			if (Uart_RxCount)
			{
				Uart_RxCount--;
			}
			break;
		default:
			HwBuf_UartRx[Uart_RxCount] = rxdata;
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
flag Mcu_init(void)
{
	//16s watch dog timer ,512k / 32k = 16s
	WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_ACLK, WDT_A_CLOCKDIVIDER_512K);
	WDT_A_start(WDT_A_BASE);

	//Ports initial
	Gpio_init();
	Clock_init(BOARD_CPU_F);
	Adc_init();

	//Bus initial
	SpiMaster_init(BOARD_SPI_MASTER_SPEED);
	SpiSlave_init();
	I2cSlave_init(BOARD_I2C_SLAVE_ADD_NORMAL);
	Uart_init(BOARD_UART_BAUDRATE);
#if UART_DEBUG_ON
	PrintString("\e[32m\r\nMCU init start, checking power... ");
#endif

	//Check Power & turn on iw7027 power.
	while (Adc_getResult(HW_ADCPORT_DC60V) < 0x0200)
	{
		DELAY_MS(50);
	}
	while (Adc_getResult(HW_ADCPORT_DC13V) < 0x0200)
	{
		DELAY_MS(50);
	}

	//Set default param.
	SysParam_Error = Default_ErrorParam;

#if UART_DEBUG_ON
	PrintString("\r\nDC60VADC = ");
	PrintInt(Adc_getResult(HW_ADCPORT_DC60V));
	PrintString("  DC13VADC = ");
	PrintInt(Adc_getResult(HW_ADCPORT_DC13V));
	PrintString("\r\nLast Reset Source: ");
	PrintChar(SYSRSTIV);
	PrintString("\r\nMcu_init finish.\r\n\e[30m");
#endif

	__enable_interrupt();
	return FLAG_SUCCESS;
}

uint8 Mcu_checkBoardStatus(BoardInfo *boardinfo, ErrorParam *errorparam)
{
	static uint8 iserror;
	uint8 retval = 0;

	//Load Board Hardware Info
	boardinfo->bIw7027Falut = HW_GET_IW7027_FAULT_IN;
	boardinfo->bD60V = (uint32) Adc_getResult(HW_ADCPORT_DC60V) * 84 / 0x3FF;
	boardinfo->bD13V = (uint32) Adc_getResult(HW_ADCPORT_DC13V) * 19 / 0x3FF;
	boardinfo->bTemprature = Adc_getMcuTemperature();

	//BIT0 : Power error flag
	if ((boardinfo->bD60V > errorparam->eDc60vMax) || (boardinfo->bD60V < errorparam->eDc60vMin)
			|| (boardinfo->bD13V > errorparam->eDc13vMax) || (boardinfo->bD13V < errorparam->eDc13vMin))
	{
		retval |= BIT0;
	}

	//BIT1 : IW7027 Error Flag
	if (!boardinfo->bIw7027Falut && !errorparam->eIw7027FaultIgnore)
	{
		retval |= BIT1;
	}

	//BIT2 : Signal Error flag ,only set when eSpiDataErrorIgnore is off
	if (!errorparam->eSpiDataErrorIgnore)
	{
		if ((boardinfo->bSpiRxFreq < errorparam->eSpiRxFreqMin) || (!boardinfo->bSpiRxValid))
		{
			retval |= BIT2;
		}
	}

	//Set Error Out
	if (retval)
	{
		HW_SET_ERROR_OUT_LOW;
		errorparam->eErrorType = retval;
//1st time error happen
		if (!iserror)
		{
#if UART_DEBUG_ON
			PrintString("\e[31m\r\nERROR ! TYPE : \e[30m");
			PrintChar(retval);
			PrintEnter();
			PrintArray((uint8 *) &SysParam_BoardInfo, sizeof(SysParam_BoardInfo));
			PrintEnter();
#endif
			if (errorparam->eErrorSaveEn)
			{
				errorparam->eCount = *BOARD_ERROR_INFO_FLASH_PTR;
				errorparam->eCount++;
				FlashCtl_eraseSegment(BOARD_ERROR_INFO_FLASH_PTR);
				FlashCtl_write8((uint8*) errorparam, BOARD_ERROR_INFO_FLASH_PTR, sizeof(SysParam_Error));
				FlashCtl_write8((uint8*) boardinfo, BOARD_ERROR_INFO_FLASH_PTR + 0x20, sizeof(SysParam_BoardInfo));
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

		HW_SET_ERROR_OUT_HIGH;
		errorparam->eErrorType = retval;
		iserror = 0;
	}
	return retval;
}

void Mcu_reset(void)
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

void Mcu_invokeBsl(void)
{
	//Disable ISR & jump to BSL section.
	//Refer to 3.8.1 Starting the BSL From an External Application for BSL application note.
	//delay for MCU modules to finish current work.
	__disable_interrupt();
	__delay_cycles(500000);
	((void (*)()) 0x1000)();
}

void Gpio_init(void)
{
	//STEP1 : Set default value for all ports = input with pull down resistor
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PA, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PB, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PC, GPIO_PIN_ALL16);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PD, GPIO_PIN_ALL16);

	//STEP2 : Set initial GPIO statis
	//P1.0 ACLK_OUT , ACLK test point
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN0);
	//P1.1 STB_IN, GPIO input with pull up , falling edge interrupt
	//	[0] = Backlight off
	//	[1] = Backlight on
	//	[Falling Edge] = Immediate shut down
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
	GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
	GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
	GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
	//P1.2 NC
	//P1.3 SPISLAVE_CS ,
	//P1.4 PWM2
	//P1.5 NC
	//P1.6 IW7027_POWER_ON , GPIO_OUT
	//	[0] = IW7027 13V off (default)
	//	[1] = IW7027 13V on
	// Note 2016/3/2	: 	Default set IW7027 power off to avoid some dip problem.
	// 						IW7027 is forced to Power on reset together with MCU
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN6);
	GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN6);
	//P1.7 NC

	//P2.0 Vsync_Out
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN0);
	//P2.1 NC
	//P2.2 SMCLK_OUT , SMCLK test point
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN2);
	//P2.3 NC
	//P2.4 NC
	//P2.5 NC
	//P2.6 TEST_MODE_IN , input pin with external pull up
	//	[0] = run factory test mode
	//	[1] = normal mode
	GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN6);
	//P2.7 SPISLAVE_CLK

	//P3.0 I2CSLAVE_SDA
	//P3.1 I2CSLAVE_SCL
	//P3.2 NC
	//P3.3 SPISLAVE_MOSI
	//P3.4 SPISLAVE_MISO
	//P3.5 NC , not available for MSP430F5247
	//P3.6 NC , not available for MSP430F5247
	//P3.7 NC , not available for MSP430F5247

	//P4.0 NC
	//P4.1 SPIMASTER_MOSI
	//P4.2 SPIMASTER_MISO
	//P4.3 SPIMASTER_CLK
	//P4.4 UART_TX
	//P4.5 UART_RX
	//P4.6 ERROR_OUT ,Error status output
	//	[0] = Error (default)
	//	[1] = Normal working
	GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6);
	GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN6);
	//P4.7 LED_G , Green led output for test.
	//	[0] = not possiable
	//	[1] = Board Initializing (default)
	//	[Flash] = normal working
	GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
	GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
	GPIO_setDriveStrength(GPIO_PORT_P4, GPIO_PIN7, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);

	//P5.0 NC
	//P5.1 NC
	//P5.2 X2IN
	//P5.3 X2OUT
	//P5.4 X1IN
	//P5.5 X2OUT
	//P5.6 NC , not available for MSP430F5247
	//P5.7 NC , not available for MSP430F5247

	//P6.0 IW7027_FAULT_IN
	// 	[0] IW7027 fault
	//	[1] IW7027 normal
	GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);
	//P6.1 NC
	//P6.2 NC
	//P6.3 NC
	//P6.4 DC60V_DET , ADC4
	//P6.5 DC13V_DET , ADC5
	//P6.6 NC
	//P6.7 NC

	//P7.0 ~ P7.4 SPIMASTER_CS_0 ,output pin
	//	[0] chip select ,spi transfer enable
	//	[1]	chip unselect ,spi transfer disable
	GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2 + GPIO_PIN3 + GPIO_PIN4);
	//P7.6  NC
	//P7.6  NC , not available for MSP430F5247
	//P7.7  NC , not available for MSP430F5247

}

flag Adc_init(void)
{
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
}

uint16 Adc_getResult(uint8 port)
{
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
}

int8 Adc_getMcuTemperature(void)
{
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
	temperature = (((int32_t) adcval - BOARD_ADCCAL_15V_30C) * (85 - 30)) / (BOARD_ADCCAL_15V_85C - BOARD_ADCCAL_15V_30C) + 30;

	//return temperature
	return ((int8) temperature);
}

flag Clock_init(uint32 cpu_speed)
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

flag SpiMaster_init(uint32 spi_speed)
{
	//P4.1 = MOSI , P4.2 = MISO , P4.3 = CLK
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN2);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3);

	//CS = P7.0 ~ P7.4 , set default high on pin.
	GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2 | GPIO_PIN3 | GPIO_PIN4);

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

void SpiMaster_setCsPin(uint8 chipsel)
{
	// Chip No.0 , GPIO = P7.0
	if (chipsel & BIT0)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
	}
	// Chip No.1, GPIO = P7.1
	if (chipsel & BIT1)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN1);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN1);
	}
	// Chip No.2, GPIO = P7.2
	if (chipsel & BIT2)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN2);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN2);
	}
	// Chip No.3, GPIO = P7.3
	if (chipsel & BIT3)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN3);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN3);
	}
	// Chip No.4, GPIO = P7.4
	if (chipsel & BIT4)
	{
		GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN4);
	}
	else
	{
		GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN4);
	}
}

uint8 SpiMaster_sendMultiByte(uint8 *txdata, uint16 length)
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
//PrintChar(*txdata);
		USCI_B_SPI_transmitData(USCI_B1_BASE, *(txdata++));
	}

	while (USCI_B_SPI_isBusy(USCI_B1_BASE))
	{
		;
	}

	//PrintEnter();
	//Return last byte received.
	return USCI_B_SPI_receiveData(USCI_B1_BASE);
}

uint8 SpiMaster_sendSingleByte(uint8 txdata)
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

flag SpiSlave_init(void)
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
	DMA_setDstAddress(DMA_CHANNEL_0, (uint32) HwBuf_SpiSlaveRx, DMA_DIRECTION_INCREMENT);

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

void SpiSlave_startRx(void)
{
	//Reset DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
	DMA_enableTransfers(DMA_CHANNEL_0);
	//Enable SPI_RX
	//NOTE 20150923 : SPI Slave turn on after check SPI_RX_CLK= 0£¬(SPI setting is SCLK = 0 when buss free)
	//Add this to avoid some MainBoard reset timming problem
	if (!GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7))
	{
		USCI_A_SPI_enable(USCI_A0_BASE);
	}
}

void SpiSlave_stopRx(void)
{
	//Disable SPI_RX
	USCI_A_SPI_disable(USCI_A0_BASE);
	//Disable DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
}

void SpiSlave_disable(void)
{
	//Disable Cs isr
	Timer_A_disableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Disable SPI RX
	USCI_A_SPI_disable(USCI_A0_BASE);
}

void SpiSlave_enable(void)
{
	//Enable Cs isr
	Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Enable SPI Tx
	USCI_A_SPI_enable(USCI_A0_BASE);
}

void I2cSlave_init(uint8 slaveaddress)
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
	Mem_set8((uint32) &HwBuf_I2cSlave, 0x00, sizeof(HwBuf_I2cSlave));

}

flag Uart_init(uint32 baudrate)
{
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);

	//Baudrate = UART_BAUDRATE , clock freq = SMCLK_F
	USCI_A_UART_initParam param =
	{ 0 };
	param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
	param.clockPrescalar = UCS_getSMCLK() / BOARD_UART_BAUDRATE;
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

void PwmOut_init(uint8 initfreq, uint16 delay)
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

void PwmOut_sync(void)
{
	Timer_A_clear(TIMER_A1_BASE);
}

void Mem_set8(uint32 memadd, uint8 value, uint16 size)
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

void Mem_set16(uint32 memadd, uint16 value, uint16 size)
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

void Mem_copy(uint32 target_add, uint32 source_add, uint16 size)
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
