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
 ******************************************************************************/

/***1 Includes ***************/

#include "driver_mcu.h"

#include <adc10_a.h>
#include <dma.h>
#include <flashctl.h>
#include <in430.h>
#include <inc/hw_memmap.h>
#include <intrinsics.h>
#include <msp430f5249.h>
#include <msp430f5xx_6xxgeneric.h>
#include <pmm.h>
#include <ref.h>
#include <stdbool.h>
#include <stdint.h>
#include <timer_a.h>
#include <ucs.h>
#include <usci_a_spi.h>
#include <usci_a_uart.h>
#include <usci_b_i2c.h>
#include <usci_b_spi.h>
#include <wdt_a.h>

#include "app_dpl.h"
#include "driver_iw7027.h"
#include "driver_scheduler.h"
#include "driver_uartdebug.h"

/***2.1 Internal Marcos ******/

#define BOARD_XT1_F						(0)				//XT1 frequency, if no external XT1,set to 0
#define BOARD_XT2_F						(0)				//XT2 frequency, if no external XT2,set to 0
#define BOARD_SMCLK_F					(BOARD_CPU_F)	//High speed Sub_System_Clock
#define BOARD_ACLK_F					(32768)			//Low speed Assist_Clock
#define BOARD_I2C_SLAVE_ADD_NORMAL		(0x14)			//I2C Slave addess for [NORMAL] mode ,7bit mode
#define BOARD_I2C_SLAVE_ADD_ISP			(0x1C)			//I2C Slave addess for [ISP] mode ,7bit mode
#define BOARD_I2C_BUF_ADD_NORMAL		(0x4000)		//I2C buffer start address for [NORMAL] mode .
#define BOARD_I2C_BUF_ADD_ISP			(0x3C00)		//I2C buffer start address for [ISP] mode .
#define BOARD_SPI_MASTER_SPEED			(4000000)		//SPI master speed (unit in Hz)
#define BOARD_UART_BAUDRATE				(115200)		//Bound Rate of UART
#define BOARD_ADCCAL_15V_30C  			*((uint16 *)0x1A1A)	//Temperature Sensor Calibration value for 30C
#define BOARD_ADCCAL_15V_85C  			*((uint16 *)0x1A1C)	//Temperature Sensor Calibration value for 85C

#define ISP_ENTRANCE_PASSWORD_ADDRESS	(0xF8)
#define ISP_EXIT_PASSWORD_ADDRESS		(0xF000)
#define ISP_INITIAL_ADDRESS				(0xF200)
#define ISP_ISR_ADDRESS					(0xF400)
#define ISP_PASSWORD					(0x20140217)
/***2.2 Internal Struct ******/

/***2.3 Internal Variables ***/
#pragma location=ISP_EXIT_PASSWORD_ADDRESS
const uint32 Isp_exit_pw = ISP_PASSWORD;

const ErrorParam Default_ErrorParam =
{
/*DC13V*/.eDc13vMax = 16, .eDc13vMin = 10,
/*DC60V*/.eDc60vMax = 70, .eDc60vMin = 50,
/*SPI*/.eSpiRxFreqMin = 20, .eSpiDataErrorIgnore = 1,
/*IW7027*/.eIw7027FaultIgnore = 0,
/*Error Save*/.eErrorSaveEn = 0, };

/***2.4 External Variables ***/
uint16 System_InputDutyBuff[128] =
{ 0 };
uint16 System_OutputDutyBuff[128] =
{ 0 };
uint16 System_ManualDutyBuff[128] =
{ 0 };
uint8 SpiSlave_RxBuff[256] =
{ 0 };
uint8 Uart_RxBuff[256] =
{ 0 };

/******************************************************************************
 * Set I2C Slave Map Struct
 * Put Variables those need I2C access to the RAM address of
 * BOARD_I2C_BUF_ADD_NORMAL ~ BOARD_I2C_BUF_ADD_NORMAL + 0xFF .
 * Using the #pragma location = ADDRESS compiler command to appoint address.
 * Refer to TI compiler #pragma cmd list file :
 * http://www.ti.com/lit/ug/slau132l/slau132l.pdf
 *
 * Name            				origin    	length
 * ----------------------  		--------  	---------
 * System_Schedule				0x00		0x20
 * System_BoardInfo				0x20		0x10
 * System_ErrorParam			0x30		0x10
 * System_Iw7027Param			0x40		0x30
 * System_DplParam				0x70		0x30
 * I2cSlave_SpecialFuncBuff		0xA0		0x50
 * System_Version				0xF0		0x08
 * System_Isp_Password			0xF8		0x08
 * ----------------------  		--------  	---------
 * 								total		0x100
 ******************************************************************************/
#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0x00
Scheduler System_Schedule =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0x20
BoardInfo System_BoardInfo =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0x30
ErrorParam System_ErrorParam =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0x40
Iw7027Param System_Iw7027Param =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0x70
DPL_Prama System_DplParam =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0xA0
uint8 I2cSlave_SpecialFuncBuff[0x50] =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0xF0
uint64 System_Version =
{ 0 };

#pragma location = BOARD_I2C_BUF_ADD_NORMAL + 0xF8
uint64 System_Isp_Password =
{ 0 };

/***2.5 Internal Functions ***/

#pragma location=ISP_INITIAL_ADDRESS
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

	WDTCTL = WDTPW + WDTHOLD; // Stop WDT
	__disable_interrupt();

	//If pass word not correct , init I2C slave in ISP mode.
	if (Isp_exit_pw != 0x20140217)
	{
		//P4.6 = ERROR_OUT , P4.7 = LED_G , P3.0 = SDA ,P3.1 = SCL
		P4OUT |= BIT7 + BIT6;
		P4DIR |= BIT7 + BIT6;
		P3SEL |= BIT0 + BIT1;

		//Initialize I2C slave in ISP mode address
		UCB0CTL1 |= UCSWRST;
		UCB0CTL0 = UCMODE_3 + UCSYNC;
		UCB0I2COA = BOARD_I2C_SLAVE_ADD_ISP;
		UCB0CTL1 &= ~UCSWRST;
		UCB0IE |= UCRXIE + UCTXIE + UCSTTIE + UCSTPIE;

		//Clear ISP mode RAM buffer
		uint16 i;
		for (i = 0; i < 512; i++)
		{
			HWREG8( BOARD_I2C_BUF_ADD_ISP + i ) = 0xFF;
		}
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

/***2.6 External Functions ***/
uint8 Mcu_init(void)
{
	//16s watch dog timer
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

	//Set Default & get 1st board status.
	System_ErrorParam = Default_ErrorParam;

#if UART_DEBUG_ON
	PrintString("\e[32m\r\nMcu_init finish.\r\n\e[30m");
#endif

	__enable_interrupt();
	return FLAG_SUCCESS;
}

uint8 Mcu_checkBoardStatus(BoardInfo *boardinfo, ErrorParam *errorparam)
{
	static uint8 iserror;
	uint8 retval = 0;

	//Load Board Hardware Info
	boardinfo->bIw7027Falut = GET_IW7027_FAULT_IN;
	boardinfo->bD60V = (uint32) Adc_getResult(ADCPORT_DC60V) * 84 / 0x3FF;
	boardinfo->bD13V = (uint32) Adc_getResult(ADCPORT_DC13V) * 19 / 0x3FF;
	boardinfo->bTemprature = Adc_getMcuTemperature();

	//BIT0 : Power error flag
	if ((boardinfo->bD60V > errorparam->eDc60vMax) || (boardinfo->bD60V < errorparam->eDc60vMin) || (boardinfo->bD13V > errorparam->eDc13vMax)
			|| (boardinfo->bD13V < errorparam->eDc13vMin))
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
		SET_ERROR_OUT_LOW
		;
		errorparam->eErrorType = retval;
		//1st time error happen
		if (!iserror)
		{
#if UART_DEBUG_ON
			PrintString("\e[31m\r\nERROR ! TYPE : \e[30m");
			PrintChar(retval);
			PrintEnter();
			PrintArray((uint8 *) &System_BoardInfo, sizeof(System_BoardInfo));
			PrintEnter();
#endif
			if (errorparam->eErrorSaveEn)
			{
				errorparam->eCount = *BOARD_ERROR_INFO_FLASH_PTR;
				errorparam->eCount++;
				FlashCtl_eraseSegment(BOARD_ERROR_INFO_FLASH_PTR);
				FlashCtl_write8((uint8*) errorparam, BOARD_ERROR_INFO_FLASH_PTR, sizeof(System_ErrorParam));
				FlashCtl_write8((uint8*) boardinfo, BOARD_ERROR_INFO_FLASH_PTR + 0x20, sizeof(System_BoardInfo));
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

		SET_ERROR_OUT_HIGH
		;
		errorparam->eErrorType = retval;
		iserror = 0;
	}
	return retval;
}

void Mcu_reset(void)
{
	//Set watch dog timer to 64 cpu cycles.
	WDT_A_initWatchdogTimer(WDT_A_BASE, WDT_A_CLOCKSOURCE_SMCLK, WDT_A_CLOCKDIVIDER_512);
	WDT_A_start(WDT_A_BASE);
	//Wait 100cyles for watch dog reboot.
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

uint8 Gpio_init(void)
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

	return FLAG_SUCCESS;
}

//P1 GPIO ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Gpio_ISR_Port1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Gpio_ISR_Port1 (void)
#else
#error Compiler not supported!
#endif
{
	switch (__even_in_range(P1IV, 14))
	{
	case 0:	//No interrupt
		break;
	case 2:	//P1.0
		break;
	case 4:	//P1.1
			//STB ISR
#if UART_DEBUG_ON
		PrintString("\e[31m***STB falling edge detect***\r\n\e[30m");
#endif
		DELAY_US(500);
		if (!GET_STB_IN)
		{
			//Mute Backlight
			Mem_set8((uint32) &System_ManualDutyBuff, 0x00, sizeof(System_ManualDutyBuff));
			Iw7027_updateDuty((uint16*) System_ManualDutyBuff, Iw7027_LedSortMap);

			//Hold CPU till STB get High , if STB = L for 1s, reset Mcu.
			uint16 timeout = 0;
			while (!GET_STB_IN)
			{
				DELAY_MS(10);
				timeout++;
				if (timeout > 100)
				{
					Mcu_reset();
				}
			}
		}

		GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);

		break;
	case 6:			//P1.2
		break;
	case 8:			//P1.3
		break;
	case 10:			//P1.4
		break;
	case 12:			//P1.5
		break;
	case 14:			//P1.6
		break;
	case 16:			//P1.7
		break;
	default:
		break;
	}

}

uint8 Adc_init(void)
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

uint8 Clock_init(uint32 cpu_speed)
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

uint8 SpiMaster_init(uint32 spi_speed)
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

uint8 SpiMaster_sendMultiByte(uint8 *txdata, uint8 length)
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

uint8 SpiSlave_init(void)
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
	capparam.synchronizeCaptureSource = TIMER_A_CAPTURE_ASYNCHRONOUS;	//TIMER_A_CAPTURE_SYNCHRONOUS;TIMER_A_CAPTURE_ASYNCHRONOUS
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
	DMA_setDstAddress(DMA_CHANNEL_0, (uint32) SpiSlave_RxBuff, DMA_DIRECTION_INCREMENT);

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

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void SpiSlave_ISR_CsPin(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) SpiSlave_ISR_CsPin (void)
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
		System_Schedule.taskFlagSpiRx = 1;
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
		System_BoardInfo.bSpiRxFreq = (SpiSlave_CsEdgeCount - 1) / 2;
		SpiSlave_CsEdgeCount = 0;
		break;
	default:
		break;
	}
}

void SpiSlave_startRx(void)
{
	//Reset DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
	DMA_enableTransfers(DMA_CHANNEL_0);
	//Enable SPI_RX
	//NOTE 20150923 : SPI Slave turn on after check SPI_RX_CLK= 0��(SPI setting is SCLK = 0 when buss free)
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

uint8 I2cSlave_init(uint8 slaveaddress)
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

	Mem_set8((uint32) &I2cSlave_SpecialFuncBuff, 0x00, sizeof(I2cSlave_SpecialFuncBuff));
	System_Version = 0x201605250A;
	System_Isp_Password = 0x00000000;
	return FLAG_SUCCESS;
}

#pragma location=0xF400
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void I2cSlave_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) I2cSlave_ISR (void)
#else
#error Compiler not supported!
#endif
{
	static uint32 op_add;
	static uint8 *op_ptr;
	static uint8 op_buff[3];
	static uint32 rxcount;
	static uint32 txcount;

	if (UCB0I2COA == BOARD_I2C_SLAVE_ADD_ISP)
	{	//ISP mode
		switch (__even_in_range(UCB0IV, 12))
		{
		case 0:	// Vector  0: No interrupts
			break;
		case 2:	// Vector  2: ALIFG
			break;
		case 4:	// Vector  4: NACKIFG
			break;
		case 6:	// Vector  6: STTIFG
			//Handle RX isr first when conflict with START.
			if (UCB0IFG & UCRXIFG)
			{
				if (rxcount <= 2)
				{	    	//0~2 bytes are address
					op_buff[rxcount] = UCB0RXBUF;
				}
				else
				{	    	//3~ bytes are data
					HWREG8( BOARD_I2C_BUF_ADD_ISP + rxcount - 3 ) = UCB0RXBUF;
				}
				rxcount++;
				UCB0IFG &= ~ UCRXIFG;
			}

			//Calculate operation adderss.
			op_add = (*((volatile uint32 *) &op_buff[0]));
			//op_add = 0x00010000 * op_buff[0]  + 0x00000100 * op_buff[1]  + op_buff[2] ;
			op_ptr = (uint8 *) (op_add & 0x0FFFFF);

			//Reset count value.
			rxcount = 0;
			txcount = 0;
			break;
		case 8:	    	// Vector  8: STPIFG

			//Handle RX isr first when conflict with STOP.
			if (UCB0IFG & UCRXIFG)
			{
				if (rxcount <= 2)
				{	    	//Firsr 3 bytes is address
					op_buff[rxcount] = UCB0RXBUF;
				}
				else
				{	    	//check address valid;
					HWREG8( BOARD_I2C_BUF_ADD_ISP + rxcount - 3 ) = UCB0RXBUF;
				}
				rxcount++;
				UCB0IFG &= ~ UCRXIFG;
			}
			UCB0IFG &= ~ UCSTPIFG;

			//Calculate operation address.
			op_add = (*((volatile uint32 *) &op_buff[0]));
			//op_add = 0x00010000 * op_buff[0]  + 0x00000100 * op_buff[1]  + op_buff[2] ;
			op_ptr = (uint8 *) (op_add & 0x0FFFFF);

			//Flash control

			__disable_interrupt();	//Disable ISR when flash write.

			if ((op_add >= 0x104400) && (op_add <= 0x124400))
			{	//Erase
				FCTL3 = FWKEY;                            // Clear Lock bit
				FCTL1 = FWKEY + ERASE;                      // Set Erase bit
				*op_ptr = 0xFF;
				FCTL1 = FWKEY;                            // Clear WRT bit
				FCTL3 = FWKEY + LOCK;                       // Set LOCK bit
			}
			if ((rxcount > 2) && (op_add >= 0x004400) && (op_add <= 0x024400))
			{                       //Write
				uint16 i;
				uint8 * Flash_ptr;                     // Initialize Flash pointer
				Flash_ptr = (uint8 *) op_add;

				FCTL3 = FWKEY;                            // Clear Lock bit
				FCTL1 = FWKEY + ERASE;                      // Set Erase bit
				*Flash_ptr = 0;                           // Dummy write to erase Flash seg
				FCTL1 = FWKEY + WRT;                        // Set WRT bit for write operation
				for (i = 0; i < rxcount - 3; i++)
				{
					*Flash_ptr++ = HWREG8(BOARD_I2C_BUF_ADD_ISP + i);        // Write value to flash
				}
				FCTL1 = FWKEY;                            // Clear WRT bit
				FCTL3 = FWKEY + LOCK;                       // Set LOCK bit
			}
			__enable_interrupt();

			//ISP mode special CMDs
			if (op_add == 0xFFFFFF)
			{                       //Reboot CMD
				PMMCTL0 |= PMMSWBOR;
			}

			break;
		case 10:                       // Vector 10: RXIFG
			if (rxcount <= 2)
			{                       //0~2 bytes is address
				op_buff[2 - rxcount] = UCB0RXBUF;
			}
			else
			{                       //3~ bytes are data , buffer to RAM
				HWREG8( BOARD_I2C_BUF_ADD_ISP + rxcount - 3 ) = UCB0RXBUF;
			}
			rxcount++;
			break;
		case 12:                       // Vector 12: TXIFG
			//Return data to I2C Master for READ operation
			UCB0TXBUF = op_ptr[txcount];
			txcount++;
			break;
		default:
			break;
		}
	}

	if (UCB0I2COA == BOARD_I2C_SLAVE_ADD_NORMAL)
	{	    	//Normal Mode
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
			System_Schedule.taskFlagI2c = 1;
			//ISP mode entrance ,check pass word.
			if (System_Isp_Password == 0x20140217)
			{
				//Stop internal Timer
				TB0CTL &= ~MC_3;
				TA0CTL &= ~MC_3;
				RTCCTL01_H |= RTCHOLD_H;
				//Set I2C address to ISP mode
				UCB0I2COA = BOARD_I2C_SLAVE_ADD_ISP;
			}
			break;
		case 10:	    	// Vector 10: RXIFG
			if (rxcount == 0)
			{
				op_add = UCB0RXBUF;
			}
			else
			{
				HWREG8( BOARD_I2C_BUF_ADD_NORMAL + op_add + rxcount - 1) = UCB0RXBUF;
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

}

uint8 Uart_init(uint32 baudrate)
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

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void Uart_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) Uart_ISR (void)
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
	case 2:                                   	// Vector 2 - RXIFG
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
			Uart_Console(Uart_RxBuff);
			USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
#endif
			Uart_RxCount = 0;
			Mem_set8((uint32) Uart_RxBuff, 0x00, sizeof(Uart_RxBuff));
			break;
		case '\b': // "backspace"
			if (Uart_RxCount)
			{
				Uart_RxCount--;
			}
			break;
		default:
			Uart_RxBuff[Uart_RxCount] = rxdata;
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
