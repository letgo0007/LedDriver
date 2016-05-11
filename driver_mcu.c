#include "driver_mcu.h"

#include "driver_scheduler.h"

#define debuglog (1)

uint8_t Board_init(void)
{
	//16s watch dog time for MCU initial
	WDT_A_initWatchdogTimer(WDT_A_BASE,WDT_A_CLOCKSOURCE_ACLK,WDT_A_CLOCKDIVIDER_512K);
	WDT_A_start(WDT_A_BASE);

	//Ports initial
	Gpio_init();
	Clock_init(CPU_F);
	Adc_init();

	//Bus initial
	SpiMaster_init(SPI_MASTER_SPEED);
	SpiSlave_init();
	I2cSlave_init(I2C_SLAVE_ADDRESS);
	Uart_init(UART_BAUDRATE);

#if debuglog
	PrintString("\e[32mBoard_init finish.\r\n\e[30m");
#endif

	WDT_A_resetTimer(WDT_A_BASE);
	return STATUS_SUCCESS;
}

void Board_getBoardInfo(BoardInfo *outputinfo)
{
	uint32_t adctemp ;

	if( GPIO_getInputPinValue(GPIOPORT_STB_HW,GPIOPIN_STB_HW) )
	{
		outputinfo->boardStb = 1;
	}
	else
	{
		outputinfo->boardStb = 0;
	}

	if( GPIO_getInputPinValue(GPIOPORT_IW7027_FAULT_IN,GPIOPIN_IW7027_FAULT_IN) )
	{
		outputinfo->boardIw7027Falut = 1;
	}
	else
	{
		outputinfo->boardIw7027Falut = 0;
	}

	adctemp = Adc_getResult(ADCPORT_DC60V);
	outputinfo->boardD60V =  adctemp * 84 / 0x3FF ;

	adctemp = Adc_getResult(ADCPORT_DC13V);
	outputinfo->boardD13V =  adctemp * 19 / 0x3FF ;

	outputinfo->boardTemprature = Adc_getMcuTemperature();

}

void Board_reset(void)
{
	//Set watch dog timer to 64 cpu cycles.
	WDT_A_initWatchdogTimer(WDT_A_BASE,WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKDIVIDER_512);
	WDT_A_start(WDT_A_BASE);
	//Wait 100cyles for watch dog reboot.
	__delay_cycles(512+100);
}

uint8_t Gpio_init(void)
{
	//Set default value for all ports = input with pull down resistor
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PA,0xFF);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PB,0xFF);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PC,0xFF);
	GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_PD,0xFF);


	//P1.0 ACLK_OUT , ACLK test point
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1 , GPIO_PIN0);
	//P1.1 STB_IN, GPIO input with pull up , falling edge interrupt
	//	[0] = Backlight off
	//	[1] = Backlight on
	//	[Falling Edge] = Immediate shut down
	GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1 , GPIO_PIN1);
	GPIO_clearInterrupt(GPIO_PORT_P1 , GPIO_PIN1);
	GPIO_selectInterruptEdge(GPIO_PORT_P1 , GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION );
	GPIO_enableInterrupt(GPIO_PORT_P1 , GPIO_PIN1);
	//P1.2 NC
	//P1.3 SPISLAVE_CS ,
	//P1.4 PWM2
	//P1.5 NC
	//P1.6 IW7027_POWER_ON , GPIO_OUT
	//	[0] = IW7027 13V off (default)
	//	[1] = IW7027 13V on
	GPIO_setOutputHighOnPin(GPIO_PORT_P1 , GPIO_PIN6);
	GPIO_setAsOutputPin(GPIO_PORT_P1 , GPIO_PIN6);
	//P1.7 NC


	//P2.0 Vsync_Out
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2 , GPIO_PIN0);
	//P2.1 NC
	//P2.2 SMCLK_OUT , SMCLK test point
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2 , GPIO_PIN2);
	//P2.3 NC
	//P2.4 NC
	//P2.5 NC
	//P2.6 TEST_MODE_IN , input pin with external pull up
	//	[0] = run factory test mode
	//	[1] = normal mode
	GPIO_setAsInputPin(GPIO_PORT_P2 , GPIO_PIN6);
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
	//	[0] = Normal working
	//	[1] = Error (default)
	GPIO_setOutputHighOnPin(GPIO_PORT_P4 , GPIO_PIN6);
	GPIO_setAsOutputPin(GPIO_PORT_P4 , GPIO_PIN6);
	//P4.7 LED_G , Green led output for test.
	//	[0] = not possiable
	//	[1] = Board Initializing (default)
	//	[Flash] = normal working
	GPIO_setOutputHighOnPin(GPIO_PORT_P4 , GPIO_PIN7);
	GPIO_setAsOutputPin(GPIO_PORT_P4 , GPIO_PIN7);
	GPIO_setDriveStrength(GPIO_PORT_P4 , GPIO_PIN7, GPIO_FULL_OUTPUT_DRIVE_STRENGTH);

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
	GPIO_setAsInputPin(GPIO_PORT_P6 , GPIO_PIN0);
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
	GPIO_setOutputHighOnPin(GPIO_PORT_P7 , GPIO_PIN0+GPIO_PIN1+GPIO_PIN2+GPIO_PIN3+GPIO_PIN4);
	GPIO_setAsOutputPin(GPIO_PORT_P7 , GPIO_PIN0+GPIO_PIN1+GPIO_PIN2+GPIO_PIN3+GPIO_PIN4);
	//P7.6  NC
	//P7.6  NC , not available for MSP430F5247
	//P7.7  NC , not available for MSP430F5247

	return STATUS_SUCCESS;
}

uint8_t Adc_init(void)
{
    //Set P6.4 P6.5 as ADC input
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6 , GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6 , GPIO_PIN5);

    /*Initialize the ADC10_A Module
     * Base Address for the ADC10_A Module
     * Use internal ADC10_A bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC10_A_init(ADC10_A_BASE,ADC10_A_SAMPLEHOLDSOURCE_SC,ADC10_A_CLOCKSOURCE_ADC10OSC,ADC10_A_CLOCKDIVIDER_1);
    ADC10_A_enable(ADC10_A_BASE);

    /*Set ADC sample time
     * Base Address for the ADC10_A Module
     * Sample/hold for 32 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC10_A_setupSamplingTimer(ADC10_A_BASE,ADC10_A_CYCLEHOLD_32_CYCLES,ADC10_A_MULTIPLESAMPLESDISABLE);

    //Configure internal reference
    //If ref generator busy, WAIT with 1s time out
    uint32_t timeout = UCS_getSMCLK();
    while(REF_BUSY == Ref_isRefGenBusy(REF_BASE) && timeout --)
    {
        ;
    }
    if(timeout == 0)
    {
    	return STATUS_FAIL;
    }
    //Select internal ref = 1.5V
    Ref_setReferenceVoltage(REF_BASE , REF_VREF1_5V);
    //Internal Reference ON
    Ref_enableReferenceVoltage(REF_BASE);

    return STATUS_SUCCESS;
}

uint16_t Adc_getResult(uint8_t port)
{
    //Configure Memory Buffer
    /*
     * Base Address for the ADC10_A Module
     * Use input A0
     * Use positive reference of Internally generated Vref
     * Use negative reference of AVss
     */
    ADC10_A_configureMemory(ADC10_A_BASE, port ,ADC10_A_VREFPOS_INT,ADC10_A_VREFNEG_AVSS);

    //Start Conversion
    ADC10_A_startConversion(ADC10_A_BASE,ADC10_A_SINGLECHANNEL);

    while(ADC10_A_isBusy(ADC10_A_BASE))
    {
    	;
    }

    //Stop conversion when conversion finish.
    ADC10_A_disableConversions(ADC10_A_BASE ,ADC10_A_COMPLETECONVERSION);

    return ADC10_A_getResults (ADC10_A_BASE) ;
}

uint8_t Adc_getMcuTemperature(void)
{
	uint16_t adcval;
	uint32_t temperature;

	//Select Internal temp sensor , internal 1.5V referance
    ADC10_A_configureMemory(ADC10_A_BASE, ADC10_A_INPUT_TEMPSENSOR ,ADC10_A_VREFPOS_INT,ADC10_A_VREFNEG_AVSS);

    //Start Conversion
    ADC10_A_startConversion(ADC10_A_BASE, ADC10_A_SINGLECHANNEL);

    while(ADC10_A_isBusy(ADC10_A_BASE))
    {
    	;
    }

    //Stop conversion when conversion finish.
    ADC10_A_disableConversions(ADC10_A_BASE ,ADC10_A_COMPLETECONVERSION);

    //Calculate temperature according to internal caliberation infor.
    adcval = ADC10_A_getResults (ADC10_A_BASE) ;
    temperature = (((uint32_t)adcval - ADCCAL_15V_30C) * (85 - 30)) / (ADCCAL_15V_85C - ADCCAL_15V_30C) + 30;

    //return temperature
	return  ( (uint8_t) temperature);
}

//P1 GPIO ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Gpio_isrPort1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) Gpio_isrPort1 (void)
#else
#error Compiler not supported!
#endif
{
	switch(__even_in_range(P1IV,14))
	{
		case 0://No interrupt
			break;
		case 2://P1.0
			//STB ISR
			PrintString("STB falling edge detect\r\n");
			GPIO_clearInterrupt	(GPIO_PORT_P1 , GPIO_PIN1);
			break;
		case 4://P1.1
			break;
		case 6://P1.2
			break;
		case 8://P1.3
			break;
		case 10://P1.4
			break;
		case 12://P1.5
			break;
		case 14://P1.6
			break;
		case 16://P1.7
			break;
		default:
			break;
	}

}

void 	Gpio_check(void)
{
	;
}

uint8_t Clock_init(uint32_t cpu_speed)
{
    //Set VCore = 3 for 24MHz clock
    PMM_setVCore(PMM_CORE_LEVEL_3);

    //Set DCO FLL reference = REFO
    UCS_initClockSignal(UCS_FLLREF,UCS_REFOCLK_SELECT,UCS_CLOCK_DIVIDER_1);
    //Set ACLK = REFO
    UCS_initClockSignal(UCS_ACLK,UCS_REFOCLK_SELECT,UCS_CLOCK_DIVIDER_1);

    //Set Ratio and Desired MCLK Frequency  and initialize DCO
    UCS_initFLLSettle(cpu_speed/1000,cpu_speed/32768);

	return STATUS_SUCCESS;
}

uint8_t SpiMaster_init(uint32_t spi_speed)
{
	//P4.1 = MOSI , P4.2 = MISO , P4.3 = CLK
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4 , GPIO_PIN1);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4 , GPIO_PIN2);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4 , GPIO_PIN3);

	//CS = P7.0 ~ P7.4 , set default high on pin.
	GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4);
	GPIO_setAsOutputPin		(GPIO_PORT_P7 , GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4);

    //Initialize Master
	uint8_t ucb1returnValue = 0x00;

    USCI_B_SPI_initMasterParam param_ucb1 = {0};
    param_ucb1.selectClockSource 	= USCI_B_SPI_CLOCKSOURCE_SMCLK;
    param_ucb1.clockSourceFrequency = UCS_getSMCLK();
    param_ucb1.desiredSpiClock 		= spi_speed;
    param_ucb1.msbFirst 			= USCI_B_SPI_MSB_FIRST;
    param_ucb1.clockPhase 			= USCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
    param_ucb1.clockPolarity 		= USCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;

    ucb1returnValue = USCI_B_SPI_initMaster(USCI_B1_BASE, &param_ucb1);
    if(STATUS_FAIL == ucb1returnValue)
    {
        return STATUS_FAIL;
    }
    //Enable SPI module
    USCI_B_SPI_enable(USCI_B1_BASE);

    //Wait for slave to initialize
    __delay_cycles(100);

    return STATUS_SUCCESS;
}

void SpiMaster_setCsPin(uint8_t chipsel)
{
	// Chip No.0 , GPIO = P7.0
	if(chipsel & BIT0)
	{
		GPIO_setOutputLowOnPin	(GPIO_PORT_P7 , GPIO_PIN0);
	}
	else
	{
		GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN0);
	}
	// Chip No.1, GPIO = P7.1
	if(chipsel & BIT1)
	{
		GPIO_setOutputLowOnPin	(GPIO_PORT_P7 , GPIO_PIN1);
	}
	else
	{
		GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN1);
	}
	// Chip No.2, GPIO = P7.2
	if(chipsel & BIT2)
	{
		GPIO_setOutputLowOnPin	(GPIO_PORT_P7 , GPIO_PIN2);
	}
	else
	{
		GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN2);
	}
	// Chip No.3, GPIO = P7.3
	if(chipsel & BIT3)
	{
		GPIO_setOutputLowOnPin	(GPIO_PORT_P7 , GPIO_PIN3);
	}
	else
	{
		GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN3);
	}
	// Chip No.4, GPIO = P7.4
	if(chipsel & BIT4)
	{
		GPIO_setOutputLowOnPin	(GPIO_PORT_P7 , GPIO_PIN4);
	}
	else
	{
		GPIO_setOutputHighOnPin	(GPIO_PORT_P7 , GPIO_PIN4);
	}
}

uint8_t SpiMaster_sendMultiByte(uint8_t *txdata , uint8_t length )
{
	//Send multiple bytes till count down
	while(length--)
	{
	    //TX buffer ready?
	    while(!USCI_B_SPI_getInterruptStatus(USCI_B1_BASE,USCI_B_SPI_TRANSMIT_INTERRUPT))
	    {
	        ;
	    }
	    //Transmit Data to slave
		//PrintChar(*txdata);
	    USCI_B_SPI_transmitData(USCI_B1_BASE, *(txdata++) );
	}


	while ( USCI_B_SPI_isBusy(USCI_B1_BASE) )
	{
		;
	}

	//PrintEnter();
	//Return last byte received.
	return USCI_B_SPI_receiveData(USCI_B1_BASE);
}

uint8_t SpiMaster_sendSingleByte(uint8_t txdata)
{
    //TX buffer ready?
    while(!USCI_B_SPI_getInterruptStatus(USCI_B1_BASE,USCI_B_SPI_TRANSMIT_INTERRUPT))
    {
        ;
    }
    //Transmit Data to slave
    USCI_B_SPI_transmitData(USCI_B1_BASE, txdata );

	while ( USCI_B_SPI_isBusy(USCI_B1_BASE) )
	{
		;
	}
	//Return last byte received.
	return 	USCI_B_SPI_receiveData(USCI_B1_BASE);

}

uint8_t SpiSlave_init(void)
{
	//P2.7 = CLK , P3.3 = MOSI , P3.4 = MISO
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,GPIO_PIN7);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN3);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN4);

    //P1.3 = CS , User Timer A0.2 ,trigger both rising /falling edge
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,GPIO_PIN3);

    //Start timer in continuous mode sourced by ACLK
    Timer_A_initContinuousModeParam initTimerA0Param = {0};
    initTimerA0Param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    initTimerA0Param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    initTimerA0Param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    initTimerA0Param.timerClear = TIMER_A_DO_CLEAR;
    initTimerA0Param.startTimer = true;
    Timer_A_initContinuousMode(TIMER_A0_BASE, &initTimerA0Param);


    //Initialize TA0.2 capture mode as CS pin for SPI module , both edge interrupt.
    Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
    Timer_A_initCaptureModeParam capparam = {0};
    capparam.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;
    capparam.captureMode = TIMER_A_CAPTUREMODE_FALLING_EDGE;
    capparam.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    capparam.synchronizeCaptureSource = TIMER_A_CAPTURE_ASYNCHRONOUS ;//TIMER_A_CAPTURE_SYNCHRONOUS;TIMER_A_CAPTURE_ASYNCHRONOUS
    capparam.captureInterruptEnable=TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    capparam.captureOutputMode=TIMER_A_OUTPUTMODE_OUTBITVALUE;
    Timer_A_initCaptureMode(TIMER_A0_BASE, &capparam);

	//Set DMA for Spi slave
	DMA_initParam dma0param = {0};
	dma0param.channelSelect = DMA_CHANNEL_0;
	dma0param.transferModeSelect = DMA_TRANSFER_SINGLE;
	dma0param.transferSize = 256;
	dma0param.triggerSourceSelect = DMA_TRIGGERSOURCE_16;
	dma0param.transferUnitSelect = DMA_SIZE_SRCBYTE_DSTBYTE;
	dma0param.triggerTypeSelect = DMA_TRIGGER_HIGH;
	DMA_init(&dma0param);

	//Source = UCA0RXBUF , Destination = &rxbuff
	DMA_setSrcAddress(DMA_CHANNEL_0,USCI_A_SPI_getReceiveBufferAddressForDMA(USCI_A0_BASE),DMA_DIRECTION_UNCHANGED);
	DMA_setDstAddress(DMA_CHANNEL_0,(uint32_t)SpiSlave_RxBuff, DMA_DIRECTION_INCREMENT);

	//Init Spi slave
    uint8_t uca0_returnValue = STATUS_FAIL;
    uca0_returnValue = USCI_A_SPI_initSlave
    		(
    		USCI_A0_BASE,USCI_A_SPI_MSB_FIRST,
			USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT,
			USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
            );
    if(STATUS_FAIL == uca0_returnValue)
    {
        return STATUS_FAIL;
    }
    //Enable SPI Module
    USCI_A_SPI_enable(USCI_A0_BASE);

    return STATUS_SUCCESS;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void SpiSlave_CsPin_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) SpiSlave_CsPin_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//Edge count to calculate SpiSlave cs frequency
	static uint8_t SpiSlave_CsEdgeCount ;

    switch(__even_in_range(TA0IV,14))
    {
        case  0: // No interrupt
        	break;
        case  2: // TA0.1
        	break;
        case  4: // TA0.2 ,PWM1/SPI Slave CS
	        //Falling edge = START of frame
			SpiSlave_stopRx();
			System_Schedule.taskFlagSpiRx = 1;
			SpiSlave_startRx();
			//Sync PWM out
			PwmOut_sync();
			//Calulate freq
			SpiSlave_CsEdgeCount ++;
            Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
    	    break;
        case  6: //TA0.3
    	    break;
        case  8: //TA0.4
    	    break;
        case 10: //TA0.5
        	break;
        case 12: // reserved
        	break;
        case 14: // overflow
        	//Calculate Spi Rx CS frequencey
        	System_BoardInfo.boardSpiRxFreq = (SpiSlave_CsEdgeCount-1)/2 ;
        	SpiSlave_CsEdgeCount = 0;
    		break;
        default: break;
    }
}

void SpiSlave_startRx(void)
{
	//Reset DMA
	DMA_disableTransfers(DMA_CHANNEL_0);
	DMA_enableTransfers(DMA_CHANNEL_0);
	//Enable SPI_RX
	//NOTE 20150923 : SPI Slave turn on after check SPI_RX_CLK= 0£¬(SPI setting is SCLK = 0 when buss free)
	//Add this to avoid some MainBoard reset timming problem
    if(!GPIO_getInputPinValue(GPIO_PORT_P2 , GPIO_PIN7))
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
	Timer_A_disableCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Disable SPI RX
	USCI_A_SPI_disable(USCI_A0_BASE);
}

void SpiSlave_enable(void)
{
	//Enable Cs isr
	Timer_A_enableCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
	//Enable SPI Tx
	USCI_A_SPI_enable(USCI_A0_BASE);
}


uint8_t I2cSlave_init(uint8_t slaveaddress)
{
	//P3.0 = SDA , P3.1 = SCL
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN0);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3,GPIO_PIN1);

	//Initialize I2C as a slave device
	USCI_B_I2C_initSlave	(USCI_B0_BASE,slaveaddress/2);

	//Specify transmit mode
	USCI_B_I2C_setMode		(USCI_B0_BASE,USCI_B_I2C_RECEIVE_MODE);

	//Enable I2C Module to start operations
	USCI_B_I2C_enable		(USCI_B0_BASE);

	//Enable interrupts
	USCI_B_I2C_clearInterrupt(USCI_B0_BASE,
								  	USCI_B_I2C_START_INTERRUPT +
									USCI_B_I2C_STOP_INTERRUPT +
									USCI_B_I2C_RECEIVE_INTERRUPT +
									USCI_B_I2C_TRANSMIT_INTERRUPT
	                              );
	USCI_B_I2C_enableInterrupt(USCI_B0_BASE,
	    							USCI_B_I2C_START_INTERRUPT +
									USCI_B_I2C_STOP_INTERRUPT +
									USCI_B_I2C_RECEIVE_INTERRUPT +
									USCI_B_I2C_TRANSMIT_INTERRUPT
	                               );
	return STATUS_SUCCESS;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B0_VECTOR
__interrupt void I2cSlave_isr(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B0_VECTOR))) I2cSlave_isr (void)
#else
#error Compiler not supported!
#endif
{
	static uint8_t I2cSlave_OperationOffset;
	static uint8_t I2cSlave_RxCount;
	static uint8_t I2cSlave_TxCount;

    switch(__even_in_range(UCB0IV,12))
    {
    case  0:// Vector  0: No interrupts
    	break;
    case  2:// Vector  2: ALIFG
    	break;
    case  4:// Vector  4: NACKIFG
    	break;
    case  6:// Vector  6: STTIFG
	    I2cSlave_RxCount=0;
	    I2cSlave_TxCount=0;
        break;
    case  8:// Vector  8: STPIFG
    	System_Schedule.taskFlagI2c = 0;
	    break;
    case 10:// Vector 10: RXIFG
        if(I2cSlave_RxCount==0)
        {
        	I2cSlave_OperationOffset= USCI_B_I2C_slaveGetData(USCI_B0_BASE);
        }
	  	else
	  	{
	        I2cSlave_Map[ I2cSlave_OperationOffset + I2cSlave_RxCount - 1] = USCI_B_I2C_slaveGetData(USCI_B0_BASE);
	  	}

        I2cSlave_RxCount++;
	    break;
    case 12:// Vector 12: TXIFG
    	USCI_B_I2C_slavePutData( USCI_B0_BASE, I2cSlave_Map[ I2cSlave_OperationOffset + I2cSlave_TxCount ]);
	    I2cSlave_TxCount++;
        break;
    default:
    	break;
    }
}

uint8_t Uart_init(uint32_t baudrate)
{
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4 , GPIO_PIN4);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1 , GPIO_PIN5);

    //Baudrate = UART_BAUDRATE , clock freq = SMCLK_F
    USCI_A_UART_initParam param = {0};
    param.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = UCS_getSMCLK()/UART_BAUDRATE;
    param.firstModReg = 0;
    param.secondModReg = 5;
    param.parity = USCI_A_UART_NO_PARITY;
    param.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    param.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    param.uartMode = USCI_A_UART_MODE;
    param.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;

    if(STATUS_FAIL == USCI_A_UART_init(USCI_A1_BASE, &param))
    {
        return STATUS_FAIL;
    }

    //Enable UART module for operation
    USCI_A_UART_enable(USCI_A1_BASE);

    //Enable Receive Interrupt
    USCI_A_UART_clearInterrupt(USCI_A1_BASE,
                               USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A1_BASE,
                                USCI_A_UART_RECEIVE_INTERRUPT);

    return STATUS_SUCCESS;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void Uart_isr(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) Uart_isr (void)
#else
#error Compiler not supported!
#endif
{
	//Rxed data count
	static uint8_t Uart_RxCount;

    switch(__even_in_range(UCA1IV,4))
    {
        case 0:break;                             	// Vector 0 - no interrupt
        case 2:                                   	// Vector 2 - RXIFG
        {
        	//Read 1 byte
        	uint8_t rxdata;
        	rxdata = USCI_A_UART_receiveData(USCI_A1_BASE);

        	//Loop back RX
	        USCI_A_UART_transmitData(USCI_A1_BASE,rxdata);

	        //Check rxed char.
	        switch(rxdata)
	        {
	        	case '\r': //"enter"
#if 1
	        		//Enable all interrupt to ensure Uart console do not block other interrupt
	        	    _EINT();
	        	    //Disable Uart until
	        	    USCI_A_UART_disableInterrupt(USCI_A1_BASE,USCI_A_UART_RECEIVE_INTERRUPT);
		        	Uart_Console();
		            USCI_A_UART_enableInterrupt(USCI_A1_BASE,USCI_A_UART_RECEIVE_INTERRUPT);
#endif
		        	Uart_RxCount = 0;
		        	memset(Uart_RxBuff,0x00,256);
		        	break;
	        	case '\b':// "backspace"
	        		if(Uart_RxCount)
	        		{
		        		Uart_RxCount--;
	        		}
	        		break;
	        	default :
	        		Uart_RxBuff[Uart_RxCount] = rxdata;
	        		Uart_RxCount ++;
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

void PwmOut_init(uint8_t initfreq ,uint16_t delay)
{
	//P2.0 as TA1.1 output
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN0);

	//Start Timer in up/down mode
	Timer_A_initUpDownModeParam initUpDownParam = {0};
	initUpDownParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
	initUpDownParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	initUpDownParam.timerPeriod = 0;
	initUpDownParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
	initUpDownParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
	initUpDownParam.timerClear = TIMER_A_DO_CLEAR;
	initUpDownParam.startTimer = false;
	Timer_A_initUpDownMode(TIMER_A1_BASE, &initUpDownParam);

	// PWM1 output
	Timer_A_outputPWMParam pwm1param ={0};
	pwm1param.clockSource =TIMER_A_CLOCKSOURCE_ACLK;
	pwm1param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
	pwm1param.compareOutputMode = TIMER_A_OUTPUTMODE_SET_RESET;
	pwm1param.compareRegister =TIMER_A_CAPTURECOMPARE_REGISTER_1;
	pwm1param.dutyCycle = delay ;


	//if 0 ,it's mute fcuntion
	if(initfreq == 0)
	{
		pwm1param.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
	}
	//not 0 ,set frequency.
	else
	{
		pwm1param.timerPeriod = UCS_getACLK()/initfreq/2 ;
	}


	Timer_A_outputPWM (TIMER_A1_BASE , &pwm1param);
	Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UPDOWN_MODE);
}

void PwmOut_sync(void)
{
	Timer_A_clear(TIMER_A1_BASE);
}
