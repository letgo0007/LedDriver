/******************************************************************************
 * @file 	[hal.h]
 *
 * Hardware Abstract Layer for LED Driver Board.
 *
 * Copyright (c) 2016 SHARP CORPORATION
 *
 * @change 	[DATE]	 [EDITOR] 		[MODEL] [TYPE] 	[COMMENT]
 * ----------------------------------------------------------------------------
 * 1		20160527 Yang Zhifang	ALL		Init	Initial Version
 *
 *****************************************************************************/
#ifndef HAL_H_
#define HAL_H_
/***1 Includes ***************************************************************/

#include "driverlib.h"
#include "std.h"

/***2.1 External Macros ******************************************************/
//Firmware Version
#define HAL_VERSION						(0x1606080A)
//CPU working frequency (Hz)
#define HAL_CPU_F 						(24000000)
//Error Info flash address
#define HAL_ERROR_INFO_FLASH_PTR		((uint8 *)0x1800)
//Software delay Marcos
#define DELAY_US(x) 					__delay_cycles((uint64)(HAL_CPU_F*(uint64)x/1000000))
#define DELAY_MS(x) 					__delay_cycles((uint64)(HAL_CPU_F*(uint64)x/1000))
//ADC define
#define ADC_DC60V						4
#define ADC_DC13V						5

#ifdef __MSP430F5529__
//GPIO NO. define for MSP430F5247
#define GPIO_STB_IN						GPIO_PORT_P1,GPIO_PIN1
#define GPIO_IW_POWER_EN				GPIO_PORT_P1,GPIO_PIN6
#define GPIO_FAC_TEST					GPIO_PORT_P2,GPIO_PIN6
#define GPIO_ERROR_OUT					GPIO_PORT_P1,GPIO_PIN0
#define GPIO_LED_R						GPIO_PORT_P1,GPIO_PIN0
#define GPIO_LED_G						GPIO_PORT_P4,GPIO_PIN7
#define GPIO_IW_FAULT_IN				GPIO_PORT_P2,GPIO_PIN1
#define GPIO_IW_CS_0					GPIO_PORT_P6,GPIO_PIN0
#define GPIO_IW_CS_1					GPIO_PORT_P6,GPIO_PIN1
#define GPIO_IW_CS_2					GPIO_PORT_P6,GPIO_PIN2
#define GPIO_IW_CS_3					GPIO_PORT_P6,GPIO_PIN3
#define GPIO_IW_CS_4					GPIO_PORT_P6,GPIO_PIN4
#define GPIO_IW_CS_5					GPIO_PORT_P7,GPIO_PIN0
#define GPIO_IW_CS_6					GPIO_PORT_P3,GPIO_PIN6
#define GPIO_IW_CS_7					GPIO_PORT_P3,GPIO_PIN5
#endif

#ifdef __MSP430F5247__
//GPIO NO. define for MSP430F5247
#define GPIO_STB_IN						1,1
#define GPIO_IW_POWER_EN				1,6
#define GPIO_FAC_TEST					2,6
#define GPIO_ERROR_OUT					4,6
#define GPIO_LED_R						4,6
#define GPIO_LED_G						4,7
#define GPIO_IW_FAULT_IN				6,0
#define GPIO_IW_CS_0					7,0
#define GPIO_IW_CS_1					7,1
#define GPIO_IW_CS_2					7,2
#define GPIO_IW_CS_3					7,3
#define GPIO_IW_CS_4					7,4
#define GPIO_IW_CS_5					7,5
#define GPIO_IW_CS_6					7,6
#define GPIO_IW_CS_7					7,7
#endif

//GPIO operation macro
#define HAL_GET_STB_IN					(GPIO_getInputPinValue(GPIO_STB_IN))
#define HAL_GET_IW7027_FAULT_IN			(GPIO_getInputPinValue(GPIO_IW_FAULT_IN))
#define HAL_SET_IW7027_POWER_ON			(GPIO_setOutputHighOnPin(GPIO_IW_POWER_EN))
#define HAL_SET_IW7027_POWER_OFF		(GPIO_setOutputLowOnPin(GPIO_IW_POWER_EN))
#define HAL_SET_LED_G_ON				(GPIO_setOutputHighOnPin(GPIO_LED_G))
#define HAL_SET_LED_G_OFF				(GPIO_setOutputLowOnPin(GPIO_LED_G))
#define HAL_TOGGLE_LED_G				(GPIO_toggleOutputOnPin(GPIO_LED_G))
#define HAL_SET_ERROR_OUT_HIGH			(GPIO_setOutputHighOnPin(GPIO_ERROR_OUT))
#define HAL_SET_ERROR_OUT_LOW			(GPIO_setOutputLowOnPin(GPIO_ERROR_OUT))
#define HAL_WATCHDOG_RESET				(WDT_A_resetTimer(WDT_A_BASE))
#define HAL_WATCHDOG_HOLD				(WDT_A_hold(WDT_A_BASE))



/***2.2 External Structures **/

//Board I/O information structure.
typedef struct Hal_BoardInfo_t
{
	//[0x00~0xFF] Board D60V voltage ,unit in V.
	uint8 u8Dc60v;
	//[0x00~0xFF] Board D13V voltage ,unit in V.
	uint8 u8Dc13v;
	//[-127 ~ 127] Board temperature ,unit in C. Note it is signed value ,has negtive value.
	int8 su8McuTemperature;
	//Spi slave input frame rate.
	uint8 u8SpiRxFreq;
	//Spi slave data format check result.
	flag fSpiDataValid;
	//IW7027_FAULT_IN Gpio value
	flag fIw7027Fault;
	uint8 RESERVED[0x02];
} Hal_BoardInfo_t;

//Parameters structure for Error handle
typedef struct Hal_BoardErrorParam_t
{
	//[0x00~0xFF] Error amount , stored in flash info section.
	uint8 u8ErrorCount;
	//[0x00] = No error	[BIT0] = Power error [BIT1] = IW7027 open short error [BIT2] = Spi Rx Signal error
	uint8 u8ErrorType;
	//[0x00~0xFF] Dc60V high limit , unit in V.
	uint8 u8Dc60vMax;
	//[0x00~0xFF] Dc60V low limit , unit in V.
	uint8 u8Dc60vMin;
	//[0x00~0xFF] Dc13V high limit,unit in V.
	uint8 u8Dc13vMax;
	//[0x00~0xFF] Dc13V low limit,unit in V.
	uint8 u8Dc13vMin;
	//[0x00~0xFF] Spi Frame rate low limit,unit in Hz.
	uint8 u8SpiRxFreqMin;
	//[1] Ignore Spi data error ,both format & frequency error.
	flag fSpiDataErrorIgnore;
	//[1] Ignore IW7027_FAULT_IN error.
	flag fIw7027FaultIgnore;
	//[0x00] = No error	[BIT0] = Open error [BIT1] = Short error [BIT2] = D-S Short error
	uint8 u8Iw7027ErrorType;
	//[1] Save Error Param & Board info to flash
	flag fErrorSaveEn;
	//RESERVED
	uint8 RESERVED[0x05];
} Hal_BoardErrorParam_t;

typedef struct Hal_CpuScheduler_t
{
	//[1] Normal Working [0] Reboot
	flag fSystemResetN;
	//[1] Local Dimming Mode [0] Manual Mode
	flag fLocalDimmingOn;
	//[0x0001~0xFFFF] Period for CPU wake up , unit in 30.5us
	uint16 u16CpuTickPeriod;
	//[0x0001~0xFFFF] Period for board status check , unit in 30.5us
	uint16 u16GpioCheckPeriod;
	//[0x0001~0xFFFF] Peroid for Manual mode duty update , unit in 30.5us
	uint16 u16TestModePeriod;
	//[1]Flag for Spi Rxed task ,triggered by Spi CS rising edge.
	flag fTaskFlagSpiRx;
	//[1]Flag for Spi Tx task , triggerd by Spi Rx data check OK.
	flag fTaskFlagSpiTx;
	//[1]Flag for I2c slave event ,triggerd by I2C STOP .
	flag fTaskFlagI2c;
	//[1]Flag for board check task , controled by u16GpioCheckPeriod.
	flag fTaskFlagGpioCheck;
	//[1]Flag for Manual Mode duty update task , controled by u16TestModePeriod.
	flag fTaskFlagTestMode;
	//[1]Test flag of 1s
	flag fTestFlag1Hz;
	//[1]Test flag of 60Hz
	flag fTestFlag60Hz;
	//[0~100]Cpu working load, unit in % .
	uint8 u8CpuLoad;
	//[0x0001~0xFFFF] Cpu wake up tick count
	uint16 u16CpuTickCount;
	//RESERVED
	uint8 RESERVED[0x0D];
} Hal_CpuScheduler_t;

typedef Calendar Hal_Time;

/***2.3 External Variables ***/

//Interface Global Variables - Paramters
extern Hal_BoardInfo_t tHal_BoardInfo;
extern Hal_BoardErrorParam_t tHal_BoardErrorParam;
extern Hal_CpuScheduler_t tHal_CpuScheduler;
extern Hal_Time tHal_Time;

//Interface Global Variables - Duty buffers
extern uint16 u16Hal_Buf_InputDuty[128];
extern uint16 u16Hal_Buf_OutputDuty[128];
extern uint16 u16Hal_Buf_TestDuty[128];

//Interface Global Variables - Hardware interface buffers
extern uint8 u8Hal_Buf_SpiSlaveRx[256];
extern uint8 u8Hal_Buf_UartRx[256];
extern uint8 u8Hal_Buf_I2cSlave[];

/***2.4 External Functions ***/

/**********************************************************
 * @Brief Board_init
 * 		Initialize MCU & board hardware .
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: MCU Normal
 * 		FLAG_FAIL		: MCU or Hardware demage
 **********************************************************/
extern flag Hal_Mcu_init(void);

/**********************************************************
 * @Brief Board_reset
 * 		Reboot function call with certain delay .
 * 		NOTE 2015/3/5 : This delay is set to ensure I2C finish send ACK when revieve REBOOT CMD.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_Mcu_reset(void);

/**********************************************************
 * @Brief Board_reset
 * 		Check Hardware status
 * @Param
 * 		*outputinfo : Output Hal_BoardInfo_t struct for other function to use.
 * 		*errorparam	: Error handle parameter struct
 * @Return
 * 		FLAG_SUCCESS: board function ok
 * 		FLAG_FAIL	: board in error status, ERROR_OUT is set
 **********************************************************/
extern uint8 Hal_Mcu_checkBoardStatus(Hal_BoardInfo_t *outputinfo, Hal_BoardErrorParam_t *errorparam);

/**********************************************************
 * @Brief Hal_Clock_init
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
extern flag Hal_Clock_init(uint32 cpu_speed);

/**********************************************************
 * @Brief Hal_Gpio_init
 * 		Initial GPIO ,set default status .
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: GPIO initial success.
 **********************************************************/
extern void Hal_Gpio_init(void);

/**********************************************************
 * @Brief Hal_Adc_init
 * 		Initial ADC ports ,set clock & referance.
 * @Param
 * 		NONE
 * @Return
 * 		FLAG_SUCCESS 	: ADC initial success.
 * 		FLAG_FAIL		: Referance error .
 **********************************************************/
extern flag Hal_Adc_init(void);

/**********************************************************
 * @Brief Hal_Adc_init
 * 		Get ADC value from selected ADC port .
 * @Param
 * 		port : ADC Port number
 * @Return
 * 		ADC_Value , 0~0x3FF (10bit)
 **********************************************************/
extern uint16 Hal_Adc_getResult(uint8 port);

/**********************************************************
 * @Brief Hal_Adc_init
 * 		Get MCU temperature from internal temp sensor.
 * @Param
 * 		NONE
 * @Return
 * 		Temprature value , unit in C .
 **********************************************************/
extern int8 Hal_Adc_getMcuTemperature(void);

/**********************************************************
 * @Brief Hal_SpiMaster_init
 * 		Initialize Spi Master to selected speed .
 * @Param
 * 		spi_speed : spi master speed ,unit in Hz.
 * @Return
 * 		FLAG_SUCCESS
 **********************************************************/
extern flag Hal_SpiMaster_init(uint32 spi_speed);

/**********************************************************
 * @Brief Hal_SpiMaster_setCsPin
 * 		Ouput SPIMASTER_CS for spi master ,control GPIO matrix to control multiple slaves.
 * @Param
 * 		chipsel : select with chip is activated, valid from IW_0~IW_N & IW_ALL.
 * 					if chipsel & BIT0 = 1 , SPIMASTER_CS_0 output LOW.
 * 					if chipsel & BIT1 = 0 , SPIMASTER_CS_1 output HIGH.
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_SpiMaster_setCsPin(uint8 chipsel);

/**********************************************************
 * @Brief Hal_SpiMaster_sendMultiByte
 * 		Send multiple byte through Spi Master.
 * @Param
 * 		*txdata : Pointer to data to be transferd.
 * 		length	: Spi output length ,unit in BYTE.
 * @Return
 * 		readvalue : last returned value from MISO.
 **********************************************************/
extern uint8 Hal_SpiMaster_sendMultiByte(uint8 *txdata, uint16 length);

/**********************************************************
 * @Brief Hal_SpiMaster_sendMultiByte
 * 		Send single byte through Spi Master.
 * @Param
 * 		txdata : Byte to be transferd.
 * @Return
 * 		readvalue : returned value from MISO.
 **********************************************************/
extern uint8 Hal_SpiMaster_sendSingleByte(uint8 txdata);

/**********************************************************
 * @Brief Hal_SpiSlave_init
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
extern flag Hal_SpiSlave_init(void);

/**********************************************************
 * @Brief Hal_SpiSlave_startRx
 * 		Spi Slave frame start .
 * 		This function generall should be put in SpiSlave Cs falling edge ISR.
 * 		Reset DMA0 & enable Spi Slave , data will be bufferd to SpiSlave_RxBuff[0].
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_SpiSlave_startRx(void);

/**********************************************************
 * @Brief Hal_SpiSlave_stopRx
 * 		Spi Slave frame stop function .
 * 		This function generall should be put in SpiSlave Cs rising edge ISR.
 * 		DMA0 & Spi_slave are both disabled to avoid receive wrong data.
 * 		DMA0 count is reseted.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_SpiSlave_stopRx(void);

/**********************************************************
 * @Brief Hal_SpiSlave_enable
 * 		Enable Spi CS interrpt & SpiSlave.
 * 		SpiSlave_RxBuff will start update data.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_SpiSlave_enable(void);

/**********************************************************
 * @Brief Hal_SpiSlave_enable
 * 		Disable Spi CS interrpt & SpiSlave.
 * 		SpiSlave_RxBuff will nolonger update.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
extern void Hal_SpiSlave_disable(void);

/******************************************************************************
 * @Brief Hal_I2cSlave_init
 * 		Initialize I2C slave with seleted I2C address.
 * 		I2C slave routine is handled by I2cSlave_ISR.
 * @Param
 * 		slaveaddress : Slave address in 7bit mode without W/R bit.
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Hal_I2cSlave_init(uint8 slaveaddress);

/******************************************************************************
 * @Brief Hal_Uart_init
 * 		Initialize Uart module.
 * @Param
 * 		baudrate : baudrate , selectable value : 115200 ,9600
 * @Return
 * 		FLAG_SUCCESS
 *****************************************************************************/
extern flag Hal_Uart_init(uint32 baudrate);

/******************************************************************************
 * @Brief Hal_Uart_sendSingleByte
 * 		Transmit Single byte using UART
 * @Param
 * 		data	: byte to be send.
 * @Return
 * 		NONE
 *****************************************************************************/
void Hal_Uart_sendSingleByte(uint8 data);

/******************************************************************************
 * @Brief Hal_PwmOut_init
 * 		Initialize & start PWM output.
 * @Param
 * 		initfreq :	frequency unit in Hz.
 * 					If set to 0 ,output is muted.
 * 		delay	: Rising edge delay from start . unit in 30.5us (ACLK frequency)
 * @Return
 * 		FLAG_SUCCESS
 *****************************************************************************/
extern void Hal_PwmOut_init(uint8 initfreq, uint16 delay);

/******************************************************************************
 * @Brief PwmOut_Sync
 * 		Reset PWM timer to synchronize pwm phase.
 * 		Use this function to synchronize pwm output from pwm input.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Hal_PwmOut_sync(void);

/******************************************************************************
 * @Brief Hal_Mem_set8
 * 		Set RAM to certain value (8bit) with DMA support.
 * 		The speed is higher than memset() in string.h , do not hold cpu.
 * @Param
 * 		memadd	: Target Memory address
 * 		value	: 8bit value to set.
 * 		size	: Memory size ,uint in byte(8bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Hal_Mem_set8(uint32 memadd, uint8 value, uint16 size);

/******************************************************************************
 * @Brief Hal_Mem_set16
 * 		Set RAM to certain value (16bit) with DMA support.
 * 		The speed is higher than memset() in string.h , do not hold cpu.
 * @Param
 * 		memadd	: Target Memory address
 * 		value	: 16bit value to set.
 * 		size	: Memory size ,uint in word(16bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Hal_Mem_set16(uint32 memadd, uint16 value, uint16 size);

/******************************************************************************
 * @Brief Hal_Mem_copy
 * 		Copy RAM function with DMA support.
 * 		The speed is higher than memcpy() in string.h , do not hold cpu.
 * @Param
 * 		target_add	: Target Memory address
 * 		source_add	: Source Memory address
 * 		size		: Memory size ,uint in byte(8bit)
 * @Return
 * 		NONE
 *****************************************************************************/
extern void Hal_Mem_copy(uint32 target_add, uint32 source_add, uint16 size);

/**********************************************************
 * @Brief Hal_Sch_init
 * 		Initialize TIMERB0 & RTC as scheduler.
 * 		Set default scheduler peroid.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Hal_Sch_init(void);

/**********************************************************
 * @Brief Hal_Sch_CpuOff
 * 		Turn off CPU with CPU load mark function.
 * 		Use this at the end of a loop.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void Hal_Sch_CpuOff(void);

/**********************************************************
 * @Brief Hal_Flash_eraseSegment
 * 		Flash segment erase function with erase check.
 * 		The segment size is 128B for info flash (0x1800~0x1980)
 * 		512B for normal flash.
 * @Param
 * 		flash_ptr : Pointer of flash to be erased.
 * @Return
 * 		NONE
 **********************************************************/
inline void Hal_Flash_eraseSegment(uint8_t *flash_ptr);

/**********************************************************
 * @Brief Hal_Flash_write
 * 		Multiple Flash write operation.
 * 		Use Hal_Flash_eraseSegment to erase flash before write
 * @Param
 * 		data_ptr 	: Pointer of data
 * 		flash_ptr 	: Pointer of flash
 * 		count		: byte length.
 * @Return
 * 		NONE
 **********************************************************/
inline void Hal_Flash_write(uint8 *data_ptr, uint8 *flash_ptr, uint16 count);

#endif /* HAL_H_ */
