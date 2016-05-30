/******************************************************************************
 * @file 	[driver_mcu.h]
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
#ifndef DRIVER_MCU_H_
#define DRIVER_MCU_H_
/***1 Includes ***************/

#include "driverlib.h"
#include "std.h"

/***2.1 External Macros ******/
#define BOARD_VERSION					(0x1605240A)					//Software Version Info
#define BOARD_CPU_F 					(24000000)						//CPU working frequency (Hz)
#define BOARD_ERROR_INFO_FLASH_PTR		((uint8 *)0x1800)				//Error Info flash address
#define DELAY_US(x) 					__delay_cycles((uint64)(BOARD_CPU_F*(uint64)x/1000000))
#define DELAY_MS(x) 					__delay_cycles((uint64)(BOARD_CPU_F*(uint64)x/1000))

//ADC Ports
#define ADCPORT_DC60V					(4)
#define ADCPORT_DC13V					(5)
#define ADCPORT_TEMPSENSOR				(10)

//GPIO Ports access macro ,refer to GPIO hardware define .

#define GET_STB_IN						(GPIO_getInputPinValue(GPIO_PORT_P1 , GPIO_PIN1))
#define GET_IW7027_FAULT_IN				(GPIO_getInputPinValue(GPIO_PORT_P6 , GPIO_PIN0))
#define SET_IW7027_POWER_ON				(GPIO_setOutputHighOnPin(GPIO_PORT_P1 , GPIO_PIN6))
#define SET_IW7027_POWER_OFF			(GPIO_setOutputLowOnPin(GPIO_PORT_P1 , GPIO_PIN6))
#define SET_LED_G_ON					(GPIO_setOutputHighOnPin(GPIO_PORT_P4 , GPIO_PIN7))
#define SET_LED_G_OFF					(GPIO_setOutputLowOnPin(GPIO_PORT_P4 , GPIO_PIN7))
#define TOGGLE_LED_G					(GPIO_toggleOutputOnPin(GPIO_PORT_P4 , GPIO_PIN7))
#define SET_ERROR_OUT_HIGH				(GPIO_setOutputHighOnPin(GPIO_PORT_P4 , GPIO_PIN6))
#define SET_ERROR_OUT_LOW				(GPIO_setOutputLowOnPin(GPIO_PORT_P4 , GPIO_PIN6))
#define WATCHDOG_RESET					(WDT_A_resetTimer(WDT_A_BASE))
/***2.2 External Structures **/

//Board I/O information structure.
typedef struct BoardInfo
{
	//[0x00~0xFF] Board D60V voltage ,unit in V.
	uint8 bD60V;
	//[0x00~0xFF] Board D13V voltage ,unit in V.
	uint8 bD13V;
	//[-127 ~ 127] Board temperature ,unit in C. Note it is signed value ,has negtive value.
	int8 bTemprature;
	//IW7027_FAULT_IN Gpio value
	flag bIw7027Falut;
	//Spi slave input frame rate.
	uint8 bSpiRxFreq;
	//Spi slave data format check result.
	flag bSpiRxValid;
	uint8 reserved[0x0A];
} BoardInfo;

//Parameters structure for Error handle
typedef struct ErrorParam
{
	//[0x00~0xFF] Error amount , stored in flash info section.
	uint8 eCount;
	//[0x00] = No error	[BIT0] = Power error [BIT1] = IW7027 open short error [BIT2] = Spi Rx Signal error
	uint8 eErrorType;
	//[0x00~0xFF] Dc60V high limit , unit in V.
	uint8 eDc60vMax;
	//[0x00~0xFF] Dc60V low limit , unit in V.
	uint8 eDc60vMin;
	//[0x00~0xFF] Dc13V high limit,unit in V.
	uint8 eDc13vMax;
	//[0x00~0xFF] Dc13V low limit,unit in V.
	uint8 eDc13vMin;
	//[0x00~0xFF] Spi Frame rate low limit,unit in Hz.
	uint8 eSpiRxFreqMin;
	//[1] Ignore Spi data error ,both format & frequency error.
	flag eSpiDataErrorIgnore;
	//[1] Ignore IW7027_FAULT_IN error.
	flag eIw7027FaultIgnore;
	//[0x00] = No error	[BIT0] = Open error [BIT1] = Short error [BIT2] = D-S Short error
	uint8 eIw7027ErrorType;
	//[1] Save Error Param & Board info to flash
	flag eErrorSaveEn;
	//RESERVED
	uint8 reserved[0x05];
} ErrorParam;

/***2.3 External Variables ***/
//Interface Global Variables - Paramters
extern BoardInfo System_BoardInfo;
extern ErrorParam System_ErrorParam;

//Interface Global Variables - Duty buffers
extern uint16 System_InputDutyBuff[128];
extern uint16 System_OutputDutyBuff[128];
extern uint16 System_ManualDutyBuff[128];

//Interface Global Variables - Hardware interface buffers
extern uint8 SpiSlave_RxBuff[256];
extern uint8 Uart_RxBuff[256];
extern uint8 I2cSlave_SpecialFuncBuff[];

/***2.4 External Functions ***/

//Function Calls
/**********************************************************
 * @Brief Board_init
 * 		Initialize MCU & board hardware .
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: MCU Normal
 * 		FLAG_FAIL		: MCU or Hardware demage
 **********************************************************/
extern flag Mcu_init(void);

/**********************************************************
 * @Brief Board_reset
 * 		Reboot function call with certain delay .
 * 		NOTE 2015/3/5 : This delay is set to ensure I2C finish send ACK when revieve REBOOT CMD.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Mcu_reset(void);

/**********************************************************
 * @Brief Board_reset
 * 		Check Hardware status
 * @Param
 * 		*outputinfo : Output BoardInfo struct for other function to use.
 * 		*errorparam	: Error handle parameter struct
 * @Return
 * 		FLAG_SUCCESS: board function ok
 * 		FLAG_FAIL	: board in error status, ERROR_OUT is set
 **********************************************************/
extern uint8 Mcu_checkBoardStatus(BoardInfo *outputinfo, ErrorParam *errorparam);

/**********************************************************
 * @Brief Clock_init
 * 		Set System clock .
 * 		MCLK (main clock) is set accroding to cpu_speed.
 * 		SMCLK (Sub main clock)  = MCLK
 * 		ACLK (Assist clock) = 32768Hz
 * @Param
 * 		cpu_speed : Target CPU speed. unit in Hz
 * @Return
 * 		FLAG_SUCCESS 	: Clock normally set.
 * 		FLAG_FAIL		: Clock init fail , cristal or power error.
 **********************************************************/
extern flag Clock_init(uint32 cpu_speed);

/**********************************************************
 * @Brief Gpio_init
 * 		Initial GPIO ,set default status .
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: GPIO initial success.
 **********************************************************/
extern void Gpio_init(void);

/**********************************************************
 * @Brief Adc_init
 * 		Initial ADC ports ,set clock & referance.
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: ADC initial success.
 * 		FLAG_FAIL		: Referance error .
 **********************************************************/
extern flag Adc_init(void);

/**********************************************************
 * @Brief Adc_init
 * 		Get ADC value from selected ADC port .
 * @Param
 * 		port : ADC Port number
 * @Return
 * 		ADC_Value , 0~0x3FF (10bit)
 **********************************************************/
extern uint16 Adc_getResult(uint8 port);

/**********************************************************
 * @Brief Adc_init
 * 		Get MCU temperature from internal temp sensor
 * @Param
 * 		NONE
 * @Return
 * 		Temprature value , unit in C .
 **********************************************************/
extern int8 Adc_getMcuTemperature(void);

/**********************************************************
 * @Brief SpiMaster_init
 * 		Initialize Spi Master to selected speed .
 * @Param
 * 		spi_speed : spi master speed ,unit in Hz.
 * @Return
 * 		FLAG_SUCCESS
 **********************************************************/
extern flag SpiMaster_init(uint32 spi_speed);

/**********************************************************
 * @Brief SpiMaster_setCsPin
 * 		Ouput SPIMASTER_CS for spi master ,control GPIO matrix to control multiple slaves.
 * @Param
 * 		chipsel : select with chip is activated, valid from IW_0~IW_N & IW_ALL.
 * 					if chipsel & BIT0 = 1 , SPIMASTER_CS_0 output LOW.
 * 					if chipsel & BIT1 = 0 , SPIMASTER_CS_1 output HIGH.
 * @Return
 * 		NONE
 **********************************************************/
extern void SpiMaster_setCsPin(uint8 chipsel);

/**********************************************************
 * @Brief SpiMaster_sendMultiByte
 * 		Send multiple byte through Spi Master.
 * @Param
 * 		*txdata : Pointer to data to be transferd.
 * 		length	: Spi output length ,unit in BYTE.
 * @Return
 * 		readvalue : last returned value from MISO.
 **********************************************************/
extern uint8 SpiMaster_sendMultiByte(uint8 *txdata, uint16 length);

/**********************************************************
 * @Brief SpiMaster_sendMultiByte
 * 		Send single byte through Spi Master.
 * @Param
 * 		txdata : Byte to be transferd.
 * @Return
 * 		readvalue : returned value from MISO.
 **********************************************************/
extern uint8 SpiMaster_sendSingleByte(uint8 txdata);

/**********************************************************
 * @Brief SpiSlave_init
 * 		Initialize SpiSlave .
 * 		Spi Slave received data is bufferd to SpiSlave_RxBuff by DMA0 (direct memory access) .
 * 		The maximum buffer size is 256 .
 * 		Oversized data will be ignored.
 * 		CS pin is set to as PWM input with both edge trigger interrupt.
 * 		Falling edge is START of a frame.
 * 		Rising edge is STOP of a frame.
 * 		Frame buffer will not be automaticly reset to 0.
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS
 **********************************************************/
extern flag SpiSlave_init(void);

/**********************************************************
 * @Brief SpiSlave_startRx
 * 		Spi Slave frame start .
 * 		This function generall should be put in SpiSlave Cs falling edge ISR.
 * 		Reset DMA0 & enable Spi Slave , data will be bufferd to SpiSlave_RxBuff[0].
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void SpiSlave_startRx(void);

/**********************************************************
 * @Brief SpiSlave_stopRx
 * 		Spi Slave frame stop function .
 * 		This function generall should be put in SpiSlave Cs rising edge ISR.
 * 		DMA0 & Spi_slave are both disabled to avoid receive wrong data.
 * 		DMA0 count is reseted.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void SpiSlave_stopRx(void);

/**********************************************************
 * @Brief SpiSlave_enable
 * 		Enable Spi CS interrpt & SpiSlave.
 * 		SpiSlave_RxBuff will start update data.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void SpiSlave_enable(void);

/**********************************************************
 * @Brief SpiSlave_enable
 * 		Disable Spi CS interrpt & SpiSlave.
 * 		SpiSlave_RxBuff will nolonger update.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void SpiSlave_disable(void);

/******************************************************************************
 * @Brief I2cSlave_init
 * 		Initialize I2C slave with seleted I2C address.
 * 		I2C slave routine is handled by I2cSlave_ISR.
 * @Param
 * 		slaveaddress : Slave address in 7bit mode without W/R bit.
 * @Return
 * 		NONE
 *****************************************************************************/
extern void I2cSlave_init(uint8 slaveaddress);

/******************************************************************************
 * @Brief Uart_init
 * 		Initialize Uart module.
 * @Param
 * 		baudrate : baudrate , selectable value : 115200 ,9600
 * @Return
 * 		FLAG_SUCCESS
 *****************************************************************************/
extern flag Uart_init(uint32 baudrate);

/******************************************************************************
 * @Brief PwmOut_init
 * 		Initialize & start PWM output.
 * @Param
 * 		initfreq :	frequency unit in Hz.
 * 					If set to 0 ,output is muted.
 * 		delay	: Rising edge delay from start . unit in 30.5us (ACLK frequency)
 * @Return
 * 		FLAG_SUCCESS
 *****************************************************************************/
extern void PwmOut_init(uint8 initfreq, uint16 delay);

/******************************************************************************
 * @Brief PwmOut_Sync
 * 		Reset PWM timer to synchronize pwm phase.
 * 		Use this function to synchronize pwm output from pwm input.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 *****************************************************************************/
extern void PwmOut_sync(void);

/******************************************************************************
 * @Brief Mem_set8
 * 		Set RAM to certain value (8bit) with DMA support.
 * 		The speed is higher than memset() in string.h , do not hold cpu.
 * @Param
 * 		memadd	: Target Memory address
 * 		value	: 8bit value to set.
 * 		size	: Memory size ,uint in byte(8bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Mem_set8(uint32 memadd, uint8 value, uint16 size);

/******************************************************************************
 * @Brief Mem_set16
 * 		Set RAM to certain value (16bit) with DMA support.
 * 		The speed is higher than memset() in string.h , do not hold cpu.
 * @Param
 * 		memadd	: Target Memory address
 * 		value	: 16bit value to set.
 * 		size	: Memory size ,uint in word(16bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Mem_set16(uint32 memadd, uint16 value, uint16 size);

/******************************************************************************
 * @Brief Mem_copy
 * 		Copy RAM function with DMA support.
 * 		The speed is higher than memcpy() in string.h , do not hold cpu.
 * @Param
 * 		target_add	: Target Memory address
 * 		source_add	: Source Memory address
 * 		size		: Memory size ,uint in byte(8bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Mem_copy(uint32 target_add, uint32 source_add, uint16 size);
#endif /* DRIVER_MCU_H_ */
