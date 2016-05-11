#ifndef DRIVER_MCU_H_
#define DRIVER_MCU_H_

#include "app_uartdebug.h"
#include "driverlib.h"
#include "string.h"

//Board Hardware Const
#define SOFTWARE_VERSION				(0x1605030A)							//Software Version Info
#define HARDWARE_VERSION				(0x704E5200)							//Hardware Version Info
#define CPU_F 							(24000000)								//CPU working frequency (Hz)
#define XT1_F							(0)										//XT1 frequency, if no external XT1,set to 0
#define XT2_F							(0)										//XT2 frequency, if no external XT2,set to 0
#define SMCLK_F							(CPU_F)									//High speed Sub_System_Clock
#define ACLK_F							(32768)									//Low speed Assist_Clock
#define I2C_SLAVE_ADDRESS				(0x28)									//I2C Slave addess
#define SPI_MASTER_SPEED				(6000000)								//SPI master speed (Unit in Hz)
#define UART_BAUDRATE					(115200)								//Bound Rate of UART


//Marcos
#define delay_us(x) 					__delay_cycles((long)(CPU_F*(double)x/1000000))	//delay unit by us (CPU block)
#define delay_ms(x) 					__delay_cycles((long)(CPU_F*(double)x/1000))	//delay unit by ms (CPU block)

//MCU hardware setting
#define ADCCAL_15V_30C  				*((unsigned int *)0x1A1A)				//Temperature Sensor Calibration value -30 C, see device data sheet
#define ADCCAL_15V_85C  				*((unsigned int *)0x1A1C)				//Temperature Sensor Calibration value 85 C, see device data sheet
#define ADCPORT_DC60V					(ADC10_A_INPUT_A4)
#define ADCPORT_DC13V					(ADC10_A_INPUT_A5)
#define ADCPORT_TEMPSENSOR				(ADC10_A_INPUT_TEMPSENSOR)
#define GPIOPORT_STB_HW					(GPIO_PORT_P1)
#define GPIOPIN_STB_HW					(GPIO_PIN1)
#define GPIOPORT_IW7027_FAULT_IN		(GPIO_PORT_P6)
#define GPIOPIN_IW7027_FAULT_IN			(GPIO_PIN0)
#define GPIOPORT_IW7027_POWER_ON		(GPIO_PORT_P1)
#define GPIOPIN_IW7027_POWER_ON			(GPIO_PIN6)
#define GPIO_PIN_ALL					(0xFFFF)
#define LED_G_ON						(GPIO_setOutputHighOnPin(GPIO_PORT_P4 , GPIO_PIN7));
#define LED_G_OFF						(GPIO_setOutputLowOnPin(GPIO_PORT_P4 , GPIO_PIN7));
#define LED_G_TOGGLE					(GPIO_toggleOutputOnPin(GPIO_PORT_P4 , GPIO_PIN7));

//Struct
typedef struct BoardInfo
{
	uint8_t boardD60V;
	uint8_t boardD13V;
	uint8_t boardTemprature;
	uint8_t boardStb;
	uint8_t boardIw7027Falut;
	uint8_t boardSpiRxFreq;
	uint8_t boardSpiRxValid;
}BoardInfo;

//Global variables
BoardInfo System_BoardInfo ;
uint16_t System_InputDutyBuff[128] ;
uint16_t System_OutputDutyBuff[128] ;
uint16_t System_ManualDutyBuff[128] ;

//Hardware driver interface buffers
uint8_t SpiSlave_RxBuff[256];
uint8_t I2cSlave_Map[256];
uint8_t Uart_RxBuff[256];


//Function Calls
/**********************************************************
 * @Brief Board_init
 * 		Initialize MCU & board hardware .
 * @Param
 * 		NONE
 * @Return
 * 		STATUS_SUCCESS 	: MCU Normal
 * 		STATUS_FAIL		: MCU or Hardware demage
 **********************************************************/
uint8_t Board_init(void);

/**********************************************************
 * @Brief Board_reset
 * 		Reboot function call with certain delay .
 * 		NOTE 2015/3/5 : This delay is set to ensure I2C finish send ACK when revieve REBOOT CMD.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void 	Board_reset(void);

/**********************************************************
 * @Brief Board_reset
 * 		Get Hardware status
 * @Param
 * 		*outputinfo : Traget Struct to store current board information
 * 					refer to BoardInfo struct .
 * @Return
 * 		NONE
 **********************************************************/
void 	Board_getBoardInfo(BoardInfo *outputinfo);

/**********************************************************
 * @Brief Clock_init
 * 		Get Hardware status
 * @Param
 * 		cpu_speed : Target CPU speed.
 * @Return
 * 		STATUS_SUCCESS 	: Clock normally set.
 * 		STATUS_FAIL		: Clock init fail , cristal or power error.
 **********************************************************/
uint8_t Clock_init(uint32_t cpu_speed);

/**********************************************************
 * @Brief Gpio_init
 * 		Initial GPIO ,set default status .
 * @Param
 * 		NONE
 * @Return
 * 		STATUS_SUCCESS 	: GPIO initial success.
 **********************************************************/
uint8_t Gpio_init(void);

/**********************************************************
 * @Brief Adc_init
 * 		Initial ADC ports ,set clock & referance.
 * @Param
 * 		NONE
 * @Return
 * 		STATUS_SUCCESS 	: ADC initial success.
 * 		STATUS_FAIL		: Referance error .
 **********************************************************/
uint8_t Adc_init(void);

/**********************************************************
 * @Brief Adc_init
 * 		Get ADC value from selected ADC port .
 * @Param
 * 		port : ADC Port number
 * @Return
 * 		ADC_Value , 0~0x3FF (10bit)
 **********************************************************/
uint16_t Adc_getResult(uint8_t port);

/**********************************************************
 * @Brief Adc_init
 * 		Get MCU temperature from internal temp sensor
 * @Param
 * 		NONE
 * @Return
 * 		Temprature value , unit in C .
 **********************************************************/
uint8_t Adc_getMcuTemperature(void);

/**********************************************************
 * @Brief SpiMaster_init
 * 		Initialize Spi Master to selected speed .
 * @Param
 * 		spi_speed : spi master speed ,unit in Hz.
 * @Return
 * 		STATUS_SUCCESS
 **********************************************************/
uint8_t SpiMaster_init(uint32_t spi_speed);

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
void 	SpiMaster_setCsPin(uint8_t chipsel);

/**********************************************************
 * @Brief SpiMaster_sendMultiByte
 * 		Send multiple byte through Spi Master.
 * @Param
 * 		*txdata : Pointer to data to be transferd.
 * 		length	: Spi output length ,unit in BYTE.
 * @Return
 * 		readvalue : last returned value from MISO.
 **********************************************************/
uint8_t SpiMaster_sendMultiByte(uint8_t *txdata , uint8_t length );

/**********************************************************
 * @Brief SpiMaster_sendMultiByte
 * 		Send single byte through Spi Master.
 * @Param
 * 		txdata : Byte to be transferd.
 * @Return
 * 		readvalue : returned value from MISO.
 **********************************************************/
uint8_t SpiMaster_sendSingleByte(uint8_t txdata);

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
 * 		STATUS_SUCCESS
 **********************************************************/
uint8_t SpiSlave_init(void);

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
void 	SpiSlave_startRx(void);

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
void 	SpiSlave_stopRx(void);

/**********************************************************
 * @Brief SpiSlave_enable
 * 		Disable Spi CS interrpt & SpiSlave.
 * 		No longer receive any data .
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void 	SpiSlave_enable(void);

/**********************************************************
 * @Brief SpiSlave_enable
 * 		Enable Spi CS interrpt & SpiSlave.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void 	SpiSlave_disable(void);

/**********************************************************
 * @Brief I2cSlave_init
 * 		Initialize I2C slave with seleted I2C address.
 * 		I2C slave routine is handled by I2cSlave_ISR.
 * @Param
 * 		slaveaddress : Slave address in 8bit mode with W/R bit.
 * @Return
 * 		STATUS_SUCCESS
 **********************************************************/
uint8_t I2cSlave_init(uint8_t slaveaddress);

/**********************************************************
 * @Brief Uart_init
 * 		Initialize I2C slave with seleted I2C address.
 * @Param
 * 		baudrate : baudrate
 * @Return
 * 		STATUS_SUCCESS
 **********************************************************/
uint8_t Uart_init(uint32_t baudrate);

/**********************************************************
 * @Brief PwmOut_init
 * 		Initialize & start PWM output.
 * @Param
 * 		initfreq :	frequency unit in Hz.
 * 					If set to 0 ,output is muted.
 * 		delay	: Rising edge delay from start . unit in 30.5us (ACLK frequency)
 * @Return
 * 		STATUS_SUCCESS
 **********************************************************/
void 	PwmOut_init(uint8_t initfreq ,uint16_t delay);

/**********************************************************
 * @Brief PwmOut_Sync
 * 		Reset PWM timer to synchronize pwm phase.
 * 		Use this function to synchronize pwm output from pwm input.
 * @Param
 * 		NONE
 * @Return
 * 		NONE
 **********************************************************/
void 	PwmOut_sync(void);


#endif /* DRIVER_MCU_H_ */
